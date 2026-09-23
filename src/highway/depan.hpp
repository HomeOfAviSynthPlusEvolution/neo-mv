#pragma once
#include "core/depan/analysis.hpp"
#include "core/depan/sampling.hpp"
#include "highway/depan_rows.hpp"
#include "highway/depan_sampling.hpp"
#include "highway/rows.hpp"
#include <cstring>

namespace neo_mv::depan {
struct HighwayResiduals {
  static std::array<float, 4> adjust(std::array<float, 4> values, const std::array<float, 4>& scales,
                                     const std::array<float, 4>& gradients, std::size_t count) {
    simd::depan_rows::adjust(values.data(), scales.data(), gradients.data(), count, values.data());
    return values;
  }
  struct Rows {
    std::vector<float> ex, ey;
    auto operator()(std::size_t i) const { return std::array<float, 2>{ex[i], ey[i]}; }
  };
  static Rows prepare(const Observations& observations, Transform map) {
    const auto count = observations.values.size();
    std::array<std::vector<float>, 4> rows;
    for (auto& r : rows)
      r.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      const auto& o = observations.values[i];
      rows[0][i] = analysis_detail::integer32(static_cast<std::uint64_t>(o.x));
      rows[1][i] = analysis_detail::integer32(static_cast<std::uint64_t>(o.y));
      rows[2][i] = o.dx;
      rows[3][i] = o.dy;
    }
    Rows result{std::vector<float>(count), std::vector<float>(count)};
    simd::depan_rows::residuals(rows[0].data(), rows[1].data(), rows[2].data(), rows[3].data(), count, map,
                                result.ex.data(), result.ey.data());
    return result;
  }
  static FitSums accumulate(const Observations& observations, const std::vector<float>& weights, Transform map,
                            bool zoom, bool rotation) {
    const auto rows = prepare(observations, map);
    return simd::depan_rows::accumulate(observations, weights, rows.ex.data(), rows.ey.data(), zoom, rotation);
  }
};
template <>
struct FitWorkspace<HighwayResiduals> {
  const Observations& observations;
  std::array<std::vector<float>, 6> rows;
  std::vector<FitGeometry> geometry;
  bool admission_attempted = false;
  explicit FitWorkspace(const Observations& o) : observations(o), geometry(o.values.size()) {
    for (auto& row : rows)
      row.resize(o.values.size());
    for (std::size_t i = 0; i < o.values.size(); ++i) {
      const auto& v = o.values[i];
      const auto x = static_cast<std::uint64_t>(v.x), y = static_cast<std::uint64_t>(v.y);
      rows[0][i] = analysis_detail::integer32(x);
      rows[1][i] = analysis_detail::integer32(y);
      rows[2][i] = v.dx;
      rows[3][i] = v.dy;
      geometry[i] = {analysis_detail::square32(x), analysis_detail::square32(y), analysis_detail::integer32(2 * x),
                     analysis_detail::integer32(2 * y)};
    }
  }
  FitSums accumulate(const std::vector<float>& weights, Transform map, bool zoom, bool rotation) {
    simd::depan_rows::residuals(rows[0].data(), rows[1].data(), rows[2].data(), rows[3].data(),
                                observations.values.size(), map, rows[4].data(), rows[5].data());
    return simd::depan_rows::accumulate(observations, weights, rows[4].data(), rows[5].data(), zoom, rotation,
                                        geometry.data());
  }
  std::vector<float> select(Transform map, float wrong, float zerow, float global,
                            std::vector<std::int8_t>& eligibility, std::vector<float> weights) {
    if (eligibility.size() != observations.values.size())
      throw std::invalid_argument("invalid Depan weight eligibility storage");
    if (!admission_attempted) {
      admission_attempted = true;
      if (!simd::depan_rows::weight_admission(observations, rows[2].data(), rows[3].data(), wrong, eligibility.data()))
        std::fill(eligibility.begin(), eligibility.end(), std::int8_t{-1});
    }
    const bool prepared =
        simd::depan_rows::strict_residuals(rows[0].data(), rows[1].data(), rows[2].data(), rows[3].data(),
                                           observations.values.size(), map, rows[4].data(), rows[5].data());
    return select_weights<true>(observations, map, wrong, zerow, global, &eligibility, std::move(weights),
                                prepared ? rows[4].data() : nullptr, prepared ? rows[5].data() : nullptr);
  }
};
class HighwaySamplingPlan : public SamplingPlan {
public:
  using SamplingPlan::SamplingPlan;
  template <class T>
  void validate(span2d::Plane<const T> source, span2d::Plane<T> output) const {
    validate_storage(source, output);
    for (int y = 0; y < height(); ++y)
      simd::detail::scan(source.row(y).data(), width(), (1u << bits()) - 1);
  }
  template <class T>
  void render(span2d::Plane<const T> source, span2d::Plane<T> output, bool preserve = false) const {
    validate(source, output);
    if (mode() == 1) {
      simd::depan_rows::linear_render(*this, source, output, preserve);
      return;
    }
    std::vector<SamplingCoordinates> coordinates(width());
    if (mode() == 0) {
      for (int y = 0; y < height(); ++y) {
        simd::depan_rows::coordinates(*this, y, coordinates.data());
        for (int x = 0; x < width(); ++x)
          write_sample(source, output.row(y)[x], coordinates[x], preserve);
      }
      return;
    }
    // Coefficients depend only on a quantized fraction, not on the frame.
    static const auto table = [] {
      std::array<std::array<std::int64_t, 4>, 257> result{};
      for (int a = 0; a <= 256; ++a)
        result[a] = cubic_coefficients(a);
      return result;
    }();
    const bool translation = sampling_class() == SamplingClass::translation;
    const auto maximum = std::int64_t((1u << bits()) - 1);
    for (int y = 0; y < height(); ++y) {
      simd::depan_rows::coordinates(*this, y, coordinates.data());
      auto* out = output.row(y).data();
      for (int x = 0; x < width(); ++x) {
        const auto q = coordinates[x];
        if (!(q.i >= 1 && q.i < width() - 2 && q.j >= 1 && q.j < height() - 2)) {
          write_sample(source, out[x], q, preserve);
          continue;
        }
        // Fractions are in [0,1]; scaling by 256 is exact. A subnormal
        // fraction truncates to zero even when the host enables DAZ.
        const auto& cx = table[static_cast<int>(double(q.fx) * 256)];
        const auto& cy = table[static_cast<int>(double(q.fy) * 256)];
        const int ix = static_cast<int>(q.i), iy = static_cast<int>(q.j);
        std::int64_t sum = 0;
        for (int f = 0; f < 4; ++f) {
          const auto* row = source.row(iy + f - 1).data() + ix - 1;
          if (translation) {
            // Per-tap truncation makes the translation rule nonseparable.
            for (int e = 0; e < 4; ++e)
              sum += (cx[e] * cy[f] / 2048) * row[e];
          } else {
            const auto horizontal = cx[0] * row[0] + cx[1] * row[1] + cx[2] * row[2] + cx[3] * row[3];
            sum += horizontal * cy[f];
          }
        }
        if (translation)
          sum += 1024;
        const auto divisor = translation ? 2048 : 4194304;
        const auto value = sum >= 0 ? sum / divisor : -1 - ((-1 - sum) / divisor);
        out[x] = static_cast<T>(std::clamp(value, std::int64_t{0}, maximum));
      }
    }
  }
};
} // namespace neo_mv::depan
