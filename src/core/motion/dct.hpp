#pragma once

#include "core/motion/block_sampling.hpp"
#include <vector>

namespace neo_mv {

struct DctDistanceParts { std::int64_t coefficient_l1, dc_abs; };

// MVTools FFT DCT-II distance: quantize each image independently, restore
// the DC weight, then scale the coefficient L1 distance. Workspaces belong
// to one layer/request; no mutable transform state is shared between frames.
class DctWorkspace {
  int width_, height_, maximum_, midpoint_, scale_;
  bool simd_;
  double error_;
  std::vector<std::uint16_t> samples_;
  std::vector<double> input_, rows_, transformed_;
  std::vector<int> source_, reference_;
  void transform(std::vector<int>& output);
  template <class T>
  void load(span2d::Plane<const T> plane) {
    if (plane.width() != width_ || plane.height() != height_)
      throw std::invalid_argument("DCT rectangle size mismatch");
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        const auto value = plane.row(y)[x];
        if constexpr (std::is_floating_point_v<T>) {
          if (!std::isfinite(double(value)) || double(value) != std::floor(double(value)))
            throw std::invalid_argument("DCT sample outside integer pixel range");
        }
        if (value < 0 || value > maximum_)
          throw std::invalid_argument("DCT sample outside integer pixel range");
        samples_[std::size_t(y) * width_ + x] = std::uint16_t(value);
        input_[std::size_t(y) * width_ + x] = double(value);
      }
  }

public:
  DctWorkspace(int width, int height, int bits, bool simd = false);
  template <class T>
  void set_source(span2d::Plane<const T> plane) {
    load(plane);
    transform(source_);
  }
  template <class T>
  DctDistanceParts compare_parts(span2d::Plane<const T> plane) {
    load(plane);
    transform(reference_);
    std::int64_t distance = 0;
    for (std::size_t i = 0; i < source_.size(); ++i)
      distance += std::abs(source_[i] - reference_[i]);
    return {distance, std::abs(source_[0] - reference_[0])};
  }
  std::int64_t scale_distance(DctDistanceParts parts, int dc_weight) const {
    return (parts.coefficient_l1 + (dc_weight - 1) * parts.dc_abs) * scale_ / 2;
  }
  template <class T>
  std::int64_t compare(span2d::Plane<const T> plane) { return scale_distance(compare_parts(plane), 4); }
};

} // namespace neo_mv
