#pragma once

#include "core/base/plane.hpp"
#include "core/depan/numeric.hpp"
#include "core/depan/transform.hpp"
#include "core/motion/scene_classification.hpp"
#include "core/render/reference.hpp"

#include <array>
#include <optional>
#include <utility>
#include <vector>

namespace neo_mv::depan {

struct Observation {
  std::int64_t x, y;
  float dx, dy;
  std::int64_t sad;
  float base = 1;
};
struct Observations {
  int nx = 0, ny = 0;
  bool eligible = false, masked = false;
  std::int64_t sad_threshold = 0;
  std::vector<Observation> values;
};
struct MaskDescription {
  int width, height;
  std::int64_t frames;
  int bits = 8;
  bool integer = true;
  int ratio_x = 1, ratio_y = 1;
};

namespace analysis_detail {
// Exact nonnegative integer products, followed by a single binary32 rounding.
// Public centers can require 63 bits and their squares more than 64 bits.
struct Integer {
  std::array<std::uint32_t, 4> words{};
  static Integer product(std::uint64_t a, std::uint64_t b) {
    Integer result;
    for (int i = 0; i < 2; ++i) {
      std::uint64_t carry = 0;
      for (int j = 0; j < 2; ++j) {
        const auto value =
            std::uint64_t(std::uint32_t(a >> (32 * i))) * std::uint32_t(b >> (32 * j)) + result.words[i + j] + carry;
        result.words[i + j] = static_cast<std::uint32_t>(value);
        carry = value >> 32;
      }
      result.words[i + 2] = static_cast<std::uint32_t>(carry);
    }
    return result;
  }
  bool bit(int n) const { return ((words[n / 32] >> (n % 32)) & 1u) != 0; }
  float rounded() const {
    int top = 127;
    while (top >= 0 && !bit(top))
      --top;
    if (top < 0)
      return 0.0f;
    const int shift = (std::max)(0, top - 23);
    std::uint32_t significand = 0;
    for (int i = top; i >= shift; --i)
      significand = (significand << 1) | std::uint32_t(bit(i));
    if (shift > 0 && bit(shift - 1)) {
      bool sticky = false;
      for (int i = 0; i < shift - 1; ++i)
        sticky = sticky || bit(i);
      if (sticky || (significand & 1u))
        ++significand;
    }
    // The rounded significand and power-of-two scale are exact in binary64.
    return f32(std::ldexp(static_cast<double>(significand), shift));
  }
};
inline float integer32(std::uint64_t value) {
  return Integer::product(value, 1).rounded();
}
inline float square32(std::uint64_t value) {
  return Integer::product(value, value).rounded();
}
inline void validate(const Observations& observations) {
  if (observations.nx <= 0 || observations.ny <= 0 || observations.sad_threshold < 0 ||
      observations.values.size() != std::uint64_t(observations.nx) * observations.ny)
    throw std::invalid_argument("invalid Depan observation grid");
  for (const auto& value : observations.values)
    if (value.x < 0 || value.y < 0 || value.sad < 0 || !std::isfinite(value.dx) || !std::isfinite(value.dy) ||
        !std::isfinite(value.base) || value.base < 0 || value.base > 255)
      throw std::invalid_argument("invalid Depan observation");
}
inline float residual_x(const Observation& value, const Transform& map) {
  const float x = integer32(static_cast<std::uint64_t>(value.x));
  const float y = integer32(static_cast<std::uint64_t>(value.y));
  return sub(sub(add(add(map.tx, mul(map.u, x)), mul(map.v, y)), x), value.dx);
}
inline float residual_y(const Observation& value, const Transform& map) {
  const float x = integer32(static_cast<std::uint64_t>(value.x));
  const float y = integer32(static_cast<std::uint64_t>(value.y));
  return sub(sub(add(add(map.ty, mul(map.w, x)), mul(map.h, y)), y), value.dy);
}
inline float neighbor_mean(const Observations& observations, std::size_t center, bool vertical) {
  const auto nx = static_cast<std::size_t>(observations.nx);
  const std::array<std::size_t, 8> indices{center - nx - 1, center - nx,     center - nx + 1, center - 1,
                                           center + 1,      center + nx - 1, center + nx,     center + nx + 1};
  const auto component = [&](std::size_t i) {
    return vertical ? observations.values[i].dy : observations.values[i].dx;
  };
  float total = component(indices[0]);
  for (std::size_t i = 1; i < indices.size(); ++i)
    total = add(total, component(indices[i]));
  return div(total, 8.0f);
}
} // namespace analysis_detail

class AnalysisInputPlan {
  AnalysisMetadata metadata_;
  int width_, height_;
  std::int64_t frames_;
  bool masked_;
  SceneClassifier scene_;

public:
  AnalysisInputPlan(AnalysisMetadata metadata, int width, int height, std::int64_t frames, std::int64_t vector_frames,
                    std::int64_t thscd1 = 400, double thscd2 = 51, std::optional<MaskDescription> mask = {})
      : metadata_(metadata), width_(width), height_(height), frames_(frames), masked_(mask.has_value()),
        scene_(scene_descriptor(metadata), thscd1, f32(thscd2)) {
    if (!valid_analysis_metadata(metadata) || (metadata.delta != -1 && metadata.delta != 1) || width <= 0 ||
        height <= 0 || frames < 1 || frames > INT32_MAX || vector_frames < frames || vector_frames > INT32_MAX)
      throw std::invalid_argument("invalid Depan analysis creation description");
    if (mask && (mask->width != width || mask->height != height || mask->frames < frames || mask->frames > INT32_MAX ||
                 mask->bits != 8 || !mask->integer || mask->ratio_x != 1 || mask->ratio_y != 1))
      throw std::invalid_argument("invalid Depan analysis mask description");
  }
  template <class ReadProperty>
  static AnalysisInputPlan from_properties(ReadProperty&& read, int width, int height, std::int64_t frames,
                                           std::int64_t vector_frames, std::int64_t thscd1 = 400, double thscd2 = 51,
                                           std::optional<MaskDescription> mask = {}) {
    const auto field = read_analysis_field(std::forward<ReadProperty>(read), false);
    return {field.metadata, width, height, frames, vector_frames, thscd1, thscd2, mask};
  }
  const AnalysisMetadata& metadata() const { return metadata_; }
  const SceneThresholds& thresholds() const { return scene_.thresholds; }
  bool masked() const { return masked_; }
  std::int64_t vector_index(std::int64_t n) const {
    if (n < 0 || n >= frames_)
      throw std::invalid_argument("Depan analysis frame outside clip");
    return metadata_.delta > 0 ? (std::max)(std::int64_t{0}, n - 1) : n;
  }
  bool eligible(const AnalysisField& field) const {
    if (field.state == FieldState::invalid_metadata)
      return false;
    if (!valid_analysis_metadata(field.metadata) || !same_render_analysis(metadata_, field.metadata))
      throw std::invalid_argument("Depan analysis metadata changed");
    return scene_(field) == 0;
  }
  Observations observe(const AnalysisField& field, std::optional<span2d::Plane<const std::uint8_t>> mask = {}) const {
    const auto& m = metadata_;
    Observations result{m.blocks_x, m.blocks_y, eligible(field), masked_, scene_.thresholds.error, {}};
    if (!result.eligible)
      return result;
    if (masked_) {
      if (!mask)
        throw std::invalid_argument("missing eligible Depan mask frame");
      validate_plane(*mask);
      if (mask->width() != width_ || mask->height() != height_)
        throw std::invalid_argument("Depan mask frame dimensions changed");
    }
    result.values.reserve(field.grid.values.size());
    const float q = div(1.0f, f32(m.pel));
    for (int by = 0; by < m.blocks_y; ++by)
      for (int bx = 0; bx < m.blocks_x; ++bx) {
        const auto x = std::int64_t(bx) * (m.block_width - m.overlap_x) + m.block_width / 2;
        const auto y = std::int64_t(by) * (m.block_height - m.overlap_y) + m.block_height / 2;
        const auto& value = field.grid.values[std::size_t(by) * m.blocks_x + bx];
        const float base =
            masked_ && x < width_ && y < height_ ? f32(mask->row(static_cast<int>(y))[static_cast<int>(x)]) : 1.0f;
        result.values.push_back({x, y, mul(f32(value.vector.x), q), mul(f32(value.vector.y), q), value.error, base});
      }
    return result;
  }
  template <class ReadProperty>
  Observations read(ReadProperty&& properties, std::optional<span2d::Plane<const std::uint8_t>> mask = {}) const {
    return observe(read_analysis_field(std::forward<ReadProperty>(properties), true), mask);
  }
};

struct FitParameters {
  float aspect = 1, error = 15, wrong = 10, zerow = 0.05f;
  bool zoom = true, rotation = true;
};
struct FitResult {
  Transform map;
  float error;
  int iteration = 0;
  bool good = false;
};
struct FitUpdate {
  Transform map;
  float error;
};

inline std::vector<float> select_weights(const Observations& observations, const Transform& map, float wrong,
                                         float zerow, float global) {
  analysis_detail::validate(observations);
  f32(wrong);
  f32(zerow);
  f32(global);
  const int border = observations.masked ? 0 : 4;
  std::vector<float> weights(observations.values.size(), 0.0f);
  for (int by = 0; by < observations.ny; ++by)
    for (int bx = 0; bx < observations.nx; ++bx) {
      const auto index = std::size_t(by) * observations.nx + bx;
      const auto& value = observations.values[index];
      if (bx < border || bx >= observations.nx - border || by < border || by >= observations.ny - border)
        continue;
      if (value.sad > observations.sad_threshold)
        continue;
      if (bx > 0 && bx + 1 < observations.nx && by > 0 && by + 1 < observations.ny) {
        if (std::abs(sub(value.dx, analysis_detail::neighbor_mean(observations, index, false))) > wrong)
          continue;
        if (std::abs(sub(value.dy, analysis_detail::neighbor_mean(observations, index, true))) > wrong)
          continue;
      }
      if (std::abs(analysis_detail::residual_x(value, map)) > global)
        continue;
      if (std::abs(analysis_detail::residual_y(value, map)) > global)
        continue;
      weights[index] = value.dx == 0 && value.dy == 0 ? mul(zerow, value.base) : value.base;
    }
  return weights;
}

struct ScalarResiduals {
  static auto prepare(const Observations& observations, Transform map) {
    return [&observations, map](std::size_t i) {
      return std::array<float, 2>{analysis_detail::residual_x(observations.values[i], map),
                                  analysis_detail::residual_y(observations.values[i], map)};
    };
  }
};
template <class Residuals = ScalarResiduals>
inline FitUpdate fit_update(const Observations& observations, const std::vector<float>& weights, const Transform& map,
                            float aspect, float step, bool zoom, bool rotation) {
  analysis_detail::validate(observations);
  if (weights.size() != observations.values.size() || !std::isfinite(aspect) || aspect <= 0)
    throw std::invalid_argument("invalid Depan fit update inputs");
  const float aspect2 = mul(aspect, aspect);
  if (aspect2 == 0)
    throw std::invalid_argument("zero Depan squared aspect");
  f32(step);
  float n = 0.1f, x2 = 0.1f, y2 = 0.1f, residual = 0.1f;
  float gx = 0, gy = 0, gxx = 0, gyy = 0, gxy = 0, gyx = 0;
  const auto residuals = Residuals::prepare(observations, map);
  for (std::size_t i = 0; i < observations.values.size(); ++i) {
    const auto& value = observations.values[i];
    const float weight = f32(weights[i]);
    const auto errors = residuals(i);
    const float ex = errors[0], ey = errors[1];
    n = add(n, weight);
    x2 = add(x2, mul(analysis_detail::square32(static_cast<std::uint64_t>(value.x)), weight));
    y2 = add(y2, mul(analysis_detail::square32(static_cast<std::uint64_t>(value.y)), weight));
    residual = add(residual, mul(add(mul(ex, ex), mul(ey, ey)), weight));
    gx = add(gx, mul(mul(2.0f, ex), weight));
    gy = add(gy, mul(mul(2.0f, ey), weight));
    if (zoom) {
      gxx = add(gxx, mul(mul(analysis_detail::integer32(2 * static_cast<std::uint64_t>(value.x)), ex), weight));
      gyy = add(gyy, mul(mul(analysis_detail::integer32(2 * static_cast<std::uint64_t>(value.y)), ey), weight));
    }
    if (rotation) {
      gxy = add(gxy, mul(mul(analysis_detail::integer32(2 * static_cast<std::uint64_t>(value.y)), ex), weight));
      gyx = add(gyx, mul(mul(analysis_detail::integer32(2 * static_cast<std::uint64_t>(value.x)), ey), weight));
    }
  }
  gx = div(gx, mul(n, 2.0f));
  gy = div(gy, mul(n, 2.0f));
  gxx = div(gxx, mul(mul(x2, 2.0f), 1.5f));
  gyy = div(gyy, mul(mul(y2, 2.0f), 1.5f));
  gxy = div(gxy, mul(mul(y2, 2.0f), 3.0f));
  gyx = div(gyx, mul(mul(x2, 2.0f), 3.0f));
  const float error = sqrt32(div(residual, n));
  Transform next = map;
  next.tx = sub(map.tx, mul(step, gx));
  next.ty = sub(map.ty, mul(step, gy));
  if (zoom)
    next.u = sub(map.u, mul(mul(step, 0.5f), add(gxx, gyy)));
  next.h = next.u;
  next.v = sub(map.v, mul(mul(step, 0.5f), sub(gxy, div(gyx, aspect2))));
  next.w = mul(mul(-aspect, aspect), next.v);
  return {next, error};
}

template <class Residuals = ScalarResiduals>
inline FitResult fit(const Observations& observations, FitParameters parameters = {}) {
  const auto& p = parameters;
  if (!std::isfinite(p.aspect) || p.aspect <= 0 || mul(p.aspect, p.aspect) == 0)
    throw std::invalid_argument("invalid Depan fit aspect");
  f32(p.error);
  f32(p.wrong);
  f32(p.zerow);
  FitResult result{Transform{}, mul(2.0f, p.error), 0, false};
  if (observations.eligible) {
    analysis_detail::validate(observations);
    std::vector<float> weights;
    weights.reserve(observations.values.size());
    for (const auto& value : observations.values)
      weights.push_back(value.base);
    for (int k = 0; k < 5; ++k) {
      const auto next = fit_update<Residuals>(observations, weights, result.map, p.aspect, 0.3f, false, false);
      result.map = next.map;
      result.error = next.error;
      weights = select_weights(observations, result.map, p.wrong, p.zerow, 1000.0f);
    }
    result.iteration = 100;
    for (int k = 5; k < 100; ++k) {
      const float old_error = result.error;
      const auto next = fit_update<Residuals>(observations, weights, result.map, p.aspect,
                                              k < 8    ? 0.3f
                                              : k < 10 ? 0.6f
                                                       : 1.0f,
                                              p.zoom, p.rotation);
      result.map = next.map;
      result.error = next.error;
      if ((sub(old_error, result.error) < mul(0.01f, 0.5f) && k > 9) || result.error < 0.01f) {
        result.iteration = k;
        break;
      }
      weights = select_weights(observations, result.map, p.wrong, p.zerow, mul(result.error, 2.0f));
    }
  }
  result.good = result.error < p.error;
  return result;
}

} // namespace neo_mv::depan
