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
  std::vector<double> fractions_;
  std::vector<grid_detail::Axis> vertical_;
  std::vector<mask_rows::HorizontalInterval> intervals_;

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
    fractions_.reserve(g.width);
    vertical_.reserve(g.height);
    vertical_weights_.reserve(g.height);
    for (int x = 0; x < g.width; ++x) {
      const auto a = grid_detail::axis(x, g.blocks_x, covered_width_);
      left_.push_back(a.first);
      right_.push_back(a.second);
      weights_.push_back(static_cast<std::int32_t>(grid_detail::coefficient(a.remainder, a.denominator)));
      fractions_.push_back(double(a.remainder) / double(a.denominator));
      if (intervals_.empty() || intervals_.back().left != a.first || intervals_.back().right != a.second)
        intervals_.push_back({x, x + 1, a.first, a.second});
      else
        intervals_.back().end = x + 1;
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
    if constexpr (!std::is_same_v<T, float>) {
      constexpr int bias = std::is_same_v<T, std::int16_t> ? 32768 : 0;
      OverwriteVector<std::int32_t> small(g.blocks_x);
      const auto run = [&](auto& top, auto& bottom) {
        constexpr bool horizontal = std::is_same_v<typename std::decay_t<decltype(top)>::value_type, std::uint16_t>;
        const auto load = [&](int y, auto& row) {
          for (int x = 0; x < g.blocks_x; ++x)
            row[x] = int(input.row(y)[x]) + bias;
        };
        const auto expand = [&](int y, auto& row) {
          load(y, small);
          if (intervals_.size() <= std::size_t(g.width) / 4)
            mask_rows::resize_intervals(small.data(), intervals_.data(), intervals_.size(), weights_.data(), row.data());
          else
            mask_rows::resize_pass(small.data(), nullptr, left_.data(), right_.data(), weights_.data(), g.width, 0,
                                   row.data());
        };
        int first = -1, second = -1;
        for (int y = 0; y < g.height; ++y) {
          const auto& a = vertical_[y];
          if (first != a.first && second == a.first) {
            top.swap(bottom);
            std::swap(first, second);
          }
          if (first != a.first) {
            if constexpr (horizontal)
              expand(a.first, top);
            else
              load(a.first, top);
            first = a.first;
          }
          if (second != a.second) {
            if constexpr (horizontal)
              expand(a.second, bottom);
            else
              load(a.second, bottom);
            second = a.second;
          }
          if constexpr (horizontal)
            mask_rows::resize_vertical(top.data(), bottom.data(), g.width, vertical_weights_[y], output.row(y).data());
          else {
            mask_rows::resize_pass(top.data(), bottom.data(), nullptr, nullptr, nullptr, g.blocks_x, vertical_weights_[y],
                                   small.data());
            mask_rows::resize_pass(small.data(), nullptr, left_.data(), right_.data(), weights_.data(), g.width, 0,
                                   output.row(y).data());
          }
        }
      };
      if (horizontal_first_) {
        OverwriteVector<std::uint16_t> top(g.width), bottom(g.width);
        run(top, bottom);
      } else {
        OverwriteVector<std::int32_t> top(g.blocks_x), bottom(g.blocks_x);
        run(top, bottom);
      }
      return;
    }
    if constexpr (std::is_same_v<T, float>) {
      OverwriteVector<double> top(g.width), bottom(g.width);
      int first = -1, second = -1;
      const auto expand = [&](int y, auto& row) {
        const auto source = input.row(y);
        for (int x = 0; x < g.width; ++x) {
          const double a = fractions_[x];
          row[x] = (1.0 - a) * double(source[left_[x]]) + a * double(source[right_[x]]);
        }
      };
      for (int y = 0; y < g.height; ++y) {
        const auto& a = vertical_[y];
        if (first != a.first && second == a.first) {
          top.swap(bottom);
          std::swap(first, second);
        }
        if (first != a.first) {
          expand(a.first, top);
          first = a.first;
        }
        if (second != a.second) {
          expand(a.second, bottom);
          second = a.second;
        }
        mask_rows::resize_float_vertical(top.data(), bottom.data(), g.width,
                                         double(a.remainder) / double(a.denominator), output.row(y).data());
      }
    }
  }
};
} // namespace neo_mv::simd
