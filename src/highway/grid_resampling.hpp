#pragma once
#include "core/mask/grid_resampling.hpp"
#include "highway/mask_rows.hpp"
#include <vector>
namespace neo_mv::simd {
class GridResamplingPlan {
  GridResamplingGeometry geometry_;
  std::int64_t covered_width_, covered_height_;

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
  }
  const GridResamplingGeometry& geometry() const { return geometry_; }

  template <class T>
  void resize(span2d::Plane<const T> input, span2d::Plane<T> output, int bits) const {
    static_assert(supported_sample<T> || std::is_same_v<T, std::int16_t>, "unsupported grid sample");
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
    const auto& g = geometry_;
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
    const auto dx = std::uint64_t(covered_width_) * 2, dy = std::uint64_t(covered_height_) * 2;
    if constexpr (!std::is_same_v<T, float>) {
      if (dx > (std::uint64_t{1} << 32) / dy) {
        neo_mv::GridResamplingPlan(g).resize(input, output, bits);
        return;
      }
    }
    std::vector<std::int64_t> left(g.width), right(g.width);
    std::vector<double> remainders(g.width), top(g.blocks_x), bottom(g.blocks_x);
    for (int x = 0; x < g.width; ++x) {
      const auto axis = grid_detail::axis(x, g.blocks_x, covered_width_);
      left[x] = axis.first;
      right[x] = axis.second;
      remainders[x] = double(axis.remainder);
    }
    int first = -1, second = -1;
    constexpr int offset = std::is_same_v<T, std::int16_t> ? 32768 : 0;
    for (int y = 0; y < g.height; ++y) {
      const auto axis = grid_detail::axis(y, g.blocks_y, covered_height_);
      if (first != axis.first) {
        for (int x = 0; x < g.blocks_x; ++x)
          if constexpr (std::is_same_v<T, std::int16_t>)
            top[x] = double(input.row(axis.first)[x]) + offset;
          else
            top[x] = double(input.row(axis.first)[x]);
        first = axis.first;
      }
      if (second != axis.second) {
        for (int x = 0; x < g.blocks_x; ++x)
          if constexpr (std::is_same_v<T, std::int16_t>)
            bottom[x] = double(input.row(axis.second)[x]) + offset;
          else
            bottom[x] = double(input.row(axis.second)[x]);
        second = axis.second;
      }
      mask_rows::resize(top.data(), bottom.data(), left.data(), right.data(), remainders.data(), g.width, double(dx),
                        double(dy), double(axis.remainder), output.row(y).data());
    }
  }
};
} // namespace neo_mv::simd
