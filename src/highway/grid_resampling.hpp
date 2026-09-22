#pragma once
#include "core/mask/grid_resampling.hpp"
#include "highway/mask_rows.hpp"
#include <vector>
#include "core/base/overwrite.hpp"
namespace neo_mv::simd {
class GridResamplingPlan {
  GridResamplingGeometry geometry_;
  std::int64_t covered_width_, covered_height_;
  bool horizontal_first_;
  std::vector<std::int32_t> left_, right_, weights_, vertical_weights_;
  std::vector<std::int64_t> left_float_, right_float_;
  std::vector<double> remainders_;
  std::vector<grid_detail::Axis> vertical_;

public:
  explicit GridResamplingPlan(GridResamplingGeometry g) : geometry_(g) {
    if (g.blocks_x <= 0 || g.blocks_y <= 0 || g.block_width <= 0 || g.block_height <= 0 || g.overlap_x < 0 ||
        g.overlap_x >= g.block_width || g.overlap_y < 0 || g.overlap_y >= g.block_height || g.width <= 0 ||
        g.height <= 0)
      throw std::invalid_argument("invalid resampling grid geometry");
    covered_width_ = std::int64_t(g.blocks_x) * (g.block_width - g.overlap_x) + g.overlap_x;
    covered_height_ = std::int64_t(g.blocks_y) * (g.block_height - g.overlap_y) + g.overlap_y;
    if (g.width > covered_width_ || g.height > covered_height_)
      throw std::invalid_argument("resampling grid does not cover visible image");
    horizontal_first_ = grid_detail::horizontal_first(covered_width_, covered_height_, g.blocks_x, g.blocks_y);
    left_.reserve(g.width);
    right_.reserve(g.width);
    weights_.reserve(g.width);
    left_float_.reserve(g.width);
    right_float_.reserve(g.width);
    remainders_.reserve(g.width);
    vertical_.reserve(g.height);
    vertical_weights_.reserve(g.height);
    for (int x = 0; x < g.width; ++x) {
      const auto a = grid_detail::axis(x, g.blocks_x, covered_width_);
      left_.push_back(a.first);
      right_.push_back(a.second);
      left_float_.push_back(a.first);
      right_float_.push_back(a.second);
      weights_.push_back(static_cast<std::int32_t>(grid_detail::coefficient(a.remainder, a.denominator)));
      remainders_.push_back(double(a.remainder));
    }
    for (int y = 0; y < g.height; ++y) {
      const auto a = grid_detail::axis(y, g.blocks_y, covered_height_);
      vertical_.push_back(a);
      vertical_weights_.push_back(static_cast<std::int32_t>(grid_detail::coefficient(a.remainder, a.denominator)));
    }
  }
  const GridResamplingGeometry& geometry() const { return geometry_; }

  // Internal Validated calls require admitted precision, dimensions, sample
  // range, and disjoint storage. Public callers retain complete admission.
  template <class T, bool Validated = false>
  void resize(span2d::Plane<const T> input, span2d::Plane<T> output, int bits) const {
    static_assert(supported_sample<T> || std::is_same_v<T, std::int16_t>, "unsupported grid sample");
    const auto& g = geometry_;
    if constexpr (!Validated) {
      if constexpr (std::is_same_v<T, float>) {
        if (bits != 32)
          throw std::invalid_argument("float grid requires 32 bits");
      } else if constexpr (std::is_same_v<T, std::int16_t>) {
        if (bits != 16)
          throw std::invalid_argument("displacement grid requires signed 16 bits");
      } else if (bits < 8 || bits > 16 || (std::is_same_v<T, std::uint8_t> && bits != 8)) {
        throw std::invalid_argument("invalid integer mask precision");
      }
      validate_plane(input);
      validate_plane(output);
      if (input.width() != g.blocks_x || input.height() != g.blocks_y || output.width() != g.width ||
          output.height() != g.height || active_rows_overlap(input, output))
        throw std::invalid_argument("resampling storage mismatch or output alias");
      for (int y = 0; y < input.height(); ++y)
        for (int x = 0; x < input.width(); ++x) {
          if constexpr (std::is_same_v<T, float>) {
            if (!std::isfinite(input.row(y)[x]))
              throw std::invalid_argument("non-finite float grid sample");
          } else if constexpr (std::is_unsigned_v<T>) {
            if (input.row(y)[x] > ((1u << bits) - 1))
              throw std::invalid_argument("mask grid sample exceeds bit depth");
          }
        }
    }
    const auto dx = std::uint64_t(covered_width_) * 2, dy = std::uint64_t(covered_height_) * 2;
    using Sample = std::conditional_t<std::is_same_v<T, float>, double, std::int32_t>;
    OverwriteVector<Sample> top(g.blocks_x), bottom(g.blocks_x);
    int first = -1, second = -1;
    constexpr int offset = std::is_same_v<T, std::int16_t> ? 32768 : 0;
    for (int y = 0; y < g.height; ++y) {
      const auto& axis = vertical_[y];
      if (first != axis.first) {
        for (int x = 0; x < g.blocks_x; ++x)
          if constexpr (std::is_same_v<T, std::int16_t>)
            top[x] = std::int32_t(input.row(axis.first)[x]) + offset;
          else
            top[x] = Sample(input.row(axis.first)[x]);
        first = axis.first;
      }
      if (second != axis.second) {
        for (int x = 0; x < g.blocks_x; ++x)
          if constexpr (std::is_same_v<T, std::int16_t>)
            bottom[x] = std::int32_t(input.row(axis.second)[x]) + offset;
          else
            bottom[x] = Sample(input.row(axis.second)[x]);
        second = axis.second;
      }
      if constexpr (std::is_same_v<T, float>)
        mask_rows::resize(top.data(), bottom.data(), left_float_.data(), right_float_.data(), remainders_.data(),
                          g.width, double(dx), double(dy), double(axis.remainder), output.row(y).data());
      else
        mask_rows::resize(top.data(), bottom.data(), left_.data(), right_.data(), weights_.data(), g.width,
                          vertical_weights_[y], horizontal_first_, output.row(y).data());
    }
  }
};
} // namespace neo_mv::simd
