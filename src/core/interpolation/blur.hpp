#pragma once

#include "core/flow/sampling.hpp"
#include "core/mask/numeric.hpp"

namespace neo_mv {

inline int blur_time_coefficient(float blur) {
  if (!std::isfinite(blur) || blur < 0 || blur > 200)
    throw std::invalid_argument("invalid blur percentage");
  const float scaled = blur * 256.0f;
  const float time = scaled / 200.0f;
  return static_cast<int>(time);
}

template <class T>
void ordered_blur_average(const T* samples, std::size_t count, int bits, T* output) {
  mask_detail::validate_storage<T>(bits);
  if (!samples || !output || count < 1 || count > 65537)
    throw std::invalid_argument("invalid blur sample list");
  if (count == 1) {
    std::memmove(output, samples, sizeof(T));
    return;
  }
  if constexpr (std::is_same_v<T, float>) {
    if (!std::isfinite(samples[0]))
      throw std::invalid_argument("non-finite blur sample");
    double sum = double(samples[0]);
    for (std::size_t i = 1; i < count; ++i) {
      if (!std::isfinite(samples[i]))
        throw std::invalid_argument("non-finite blur sample");
      sum = sum + double(samples[i]);
      if (!std::isfinite(sum))
        throw std::overflow_error("non-finite blur accumulation");
    }
    const double value = sum / double(count);
    *output = mask_detail::binary32(value);
  } else {
    const auto maximum = (std::uint32_t{1} << bits) - 1;
    std::int64_t sum = 0;
    for (std::size_t i = 0; i < count; ++i) {
      if (samples[i] > maximum)
        throw std::invalid_argument("blur sample exceeds render precision");
      sum += samples[i];
    }
    *output = static_cast<T>(sum / static_cast<std::int64_t>(count));
  }
}

struct ScalarBlurAverage {
  template <class T>
  void operator()(const T* samples, std::size_t count, int bits, T* output) const {
    ordered_blur_average(samples, count, bits, output);
  }
};

class BlurSamplingPlan {
  RenderPhaseGeometry geometry_;
  int width_, height_, precision_, time_;
  struct Direction {
    int count;
    std::int64_t x, y;
  };
  struct Location {
    int phase, x, y;
  };
  static std::int64_t floor_div(std::int64_t value, int divisor) {
    // These coordinates and products are bounded far away from INT64_MIN.
    return value >= 0 ? value / divisor : -((divisor - 1 - value) / divisor);
  }
  Direction direction(std::int16_t x, std::int16_t y) const {
    const auto zx = std::int64_t(x) * time_, zy = std::int64_t(y) * time_;
    const auto magnitude = std::max(zx < 0 ? -zx : zx, zy < 0 ? -zy : zy);
    const int count = static_cast<int>((magnitude / precision_) / 256);
    return {count, count ? zx / count : 0, count ? zy / count : 0};
  }
  Location location(std::int64_t x, std::int64_t y) const {
    const auto& g = geometry_;
    const auto qx = floor_div(x, g.pel), qy = floor_div(y, g.pel);
    const int phase = static_cast<int>((y - qy * g.pel) * g.pel + x - qx * g.pel);
    const auto sx = qx + g.pad_x, sy = qy + g.pad_y;
    if (sx < 0 || sy < 0 || sx >= g.phases[phase].width || sy >= g.phases[phase].height)
      throw std::invalid_argument("blur trajectory exceeds logical phase domain");
    return {phase, static_cast<int>(sx), static_cast<int>(sy)};
  }
  void validate_field(const DenseFlowField& field) const {
    const auto count = std::uint64_t(width_) * height_;
    if (field.width != width_ || field.height != height_ || field.x.size() != count || field.y.size() != count)
      throw std::invalid_argument("inconsistent dense blur field");
  }

public:
  BlurSamplingPlan(RenderPhaseGeometry geometry, int width, int height, int precision, int time256)
      : geometry_(geometry), width_(width), height_(height), precision_(precision), time_(time256) {
    if (precision < 1 || time256 < 0 || time256 > 256)
      throw std::invalid_argument("invalid blur precision or time coefficient");
    (void)FlowSamplingPlan(geometry, width, height, 0); // Center-only creation admission.
  }
  const RenderPhaseGeometry& geometry() const { return geometry_; }
  int width() const { return width_; }
  int height() const { return height_; }

  template <class Visit>
  std::size_t trajectory(int x, int y, std::int16_t fx, std::int16_t fy, std::int16_t bx, std::int16_t by,
                         Visit&& visit) const {
    if (x < 0 || y < 0 || x >= width_ || y >= height_)
      throw std::invalid_argument("blur pixel outside visible plane");
    const auto qx = std::int64_t(geometry_.pel) * x, qy = std::int64_t(geometry_.pel) * y;
    const auto f = direction(fx, fy), b = direction(bx, by);
    visit(qx, qy);
    for (const auto d : {f, b})
      for (int i = 1; i <= d.count; ++i)
        visit(qx + floor_div(std::int64_t(i) * d.x, 256), qy + floor_div(std::int64_t(i) * d.y, 256));
    return std::size_t(1 + f.count + b.count);
  }

  void preflight(const DenseFlowField& forward, const DenseFlowField& backward) const {
    validate_field(forward);
    validate_field(backward);
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        const auto i = std::size_t(y) * width_ + x;
        trajectory(x, y, forward.x[i], forward.y[i], backward.x[i], backward.y[i],
                   [&](std::int64_t ax, std::int64_t ay) { (void)location(ax, ay); });
      }
  }

  template <class T, class Average = ScalarBlurAverage>
  void sample(const DenseFlowField& forward, const DenseFlowField& backward, const SubpixelPhases<T>& source,
              span2d::Plane<T> output, int bits, Average average = {}) const {
    mask_detail::validate_storage<T>(bits);
    preflight(forward, backward);
    validate_plane(output);
    if (source.pel != geometry_.pel || output.width() != width_ || output.height() != height_)
      throw std::invalid_argument("blur storage geometry mismatch");
    for (int a = 0; a < geometry_.pel * geometry_.pel; ++a) {
      const auto input = source.planes[a];
      validate_plane(input);
      if (input.width() != geometry_.phases[a].width || input.height() != geometry_.phases[a].height ||
          active_rows_overlap(input, output))
        throw std::invalid_argument("blur phase storage mismatch or output alias");
    }
    for (const auto* field : {&forward, &backward})
      for (const auto* component : {&field->x, &field->y}) {
        const auto input = dense_detail::plane(component->data(), width_, height_, component->size());
        if (active_rows_overlap(input, output))
          throw std::invalid_argument("blur output aliases dense input");
      }
    std::vector<T> samples;
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        const auto i = std::size_t(y) * width_ + x;
        const auto f = direction(forward.x[i], forward.y[i]), b = direction(backward.x[i], backward.y[i]);
        samples.resize(std::size_t(1 + f.count + b.count));
        std::size_t index = 0;
        trajectory(x, y, forward.x[i], forward.y[i], backward.x[i], backward.y[i],
                   [&](std::int64_t ax, std::int64_t ay) {
                     const auto at = location(ax, ay);
                     std::memcpy(samples.data() + index++, source.planes[at.phase].row(at.y).data() + at.x, sizeof(T));
                   });
        average(samples.data(), samples.size(), bits, output.row(y).data() + x);
      }
  }
};
} // namespace neo_mv
