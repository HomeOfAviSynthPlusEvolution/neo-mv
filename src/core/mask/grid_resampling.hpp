#pragma once

#include "core/base/plane.hpp"
#include "core/mask/numeric.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace neo_mv {
struct GridResamplingGeometry {
  int blocks_x, blocks_y, block_width, block_height, overlap_x, overlap_y, width, height;
};

namespace grid_detail {
// The axis-order comparison needs at most 126 bits for covered-width times
// covered-height. Four base-2^32 limbs keep it exact without __int128.
struct Wide {
  std::array<std::uint32_t, 4> words{};
  static Wide product(std::uint64_t a, std::uint64_t b) {
    Wide result;
    for (int i = 0; i < 2; ++i) {
      std::uint64_t carry = 0;
      for (int j = 0; j < 2; ++j) {
        const auto v =
            std::uint64_t(std::uint32_t(a >> (32 * i))) * std::uint32_t(b >> (32 * j)) + result.words[i + j] + carry;
        result.words[i + j] = std::uint32_t(v);
        carry = v >> 32;
      }
      result.words[i + 2] = std::uint32_t(carry);
    }
    return result;
  }
  Wide times(std::uint32_t n) const {
    Wide result;
    std::uint64_t carry = 0;
    for (std::size_t i = 0; i < words.size(); ++i) {
      const auto v = std::uint64_t(words[i]) * n + carry;
      result.words[i] = std::uint32_t(v);
      carry = v >> 32;
    }
    return result;
  }
  void add(const Wide& other) {
    std::uint64_t carry = 0;
    for (std::size_t i = 0; i < words.size(); ++i) {
      const auto v = std::uint64_t(words[i]) + other.words[i] + carry;
      words[i] = std::uint32_t(v);
      carry = v >> 32;
    }
  }
  bool less(const Wide& other) const {
    for (int i = 3; i >= 0; --i)
      if (words[i] != other.words[i])
        return words[i] < other.words[i];
    return false;
  }
};

inline std::uint32_t coefficient(std::uint64_t remainder, std::uint64_t denominator) {
  if (denominator == 0 || remainder >= denominator)
    throw std::invalid_argument("invalid resampling coefficient fraction");
  std::uint32_t result = 0;
  // Extract the fourteen fractional bits without forming Q * remainder.
  // Comparing against denominator - remainder also avoids overflow when
  // doubling the remainder, including the full uint64 denominator range.
  for (int bit = 0; bit < 14; ++bit) {
    result *= 2;
    const auto complement = denominator - remainder;
    if (remainder >= complement) {
      remainder -= complement;
      ++result;
    } else {
      remainder += remainder;
    }
  }
  const auto complement = denominator - remainder;
  return result + (remainder > complement || (remainder == complement && (result & 1u)));
}

inline bool horizontal_first(std::int64_t covered_width, std::int64_t covered_height, int blocks_x, int blocks_y) {
  const auto left = Wide::product(static_cast<std::uint64_t>(covered_width), blocks_y).times(2);
  auto right = Wide::product(static_cast<std::uint64_t>(covered_height), blocks_x);
  right.add(Wide::product(static_cast<std::uint64_t>(covered_width), static_cast<std::uint64_t>(covered_height)));
  return left.less(right);
}

inline std::uint32_t interpolate(std::uint32_t first, std::uint32_t second, std::uint32_t coefficient) {
  // Biased signed samples and unsigned samples both lie in [0,65535].
  // The complementary weights sum to 16384, so the full sum fits uint32.
  return ((16384u - coefficient) * first + coefficient * second + 8192u) >> 14;
}

struct Axis {
  int first, second;
  std::uint64_t denominator, remainder;
};
inline Axis axis(int coordinate, int blocks, std::int64_t covered) {
  const auto denominator = 2 * covered;
  const auto numerator = (2 * std::int64_t(coordinate) + 1) * blocks - covered;
  auto index = numerator / denominator;
  auto remainder = numerator % denominator;
  if (remainder < 0) {
    --index;
    remainder += denominator;
  }
  return {static_cast<int>(std::clamp(index, std::int64_t{0}, std::int64_t(blocks - 1))),
          static_cast<int>(std::clamp(index + 1, std::int64_t{0}, std::int64_t(blocks - 1))),
          static_cast<std::uint64_t>(denominator), static_cast<std::uint64_t>(remainder)};
}
} // namespace grid_detail

class GridResamplingPlan {
  GridResamplingGeometry geometry_;
  std::int64_t covered_width_, covered_height_;
  bool horizontal_first_;

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
    for (int y = 0; y < output.height(); ++y) {
      const auto ay = grid_detail::axis(y, g.blocks_y, covered_height_);
      for (int x = 0; x < output.width(); ++x) {
        const auto ax = grid_detail::axis(x, g.blocks_x, covered_width_);
        const std::array<T, 4> s{input.row(ay.first)[ax.first], input.row(ay.first)[ax.second],
                                 input.row(ay.second)[ax.first], input.row(ay.second)[ax.second]};
        if constexpr (std::is_same_v<T, float>) {
          const double a = double(ax.remainder) / double(ax.denominator),
                       b = double(ay.remainder) / double(ay.denominator);
          const double c = 1.0 - a, e = 1.0 - b;
          const double h0 = c * double(s[0]) + a * double(s[1]);
          const double h1 = c * double(s[2]) + a * double(s[3]);
          const double value = e * h0 + b * h1;
          // Finite float inputs and convex binary64 weights bound all products
          // well below binary64 overflow; reject a non-finite final narrowing.
          output.row(y)[x] = mask_detail::binary32(value);
        } else {
          constexpr int offset = std::is_same_v<T, std::int16_t> ? 32768 : 0;
          std::array<std::uint32_t, 4> biased{};
          for (int i = 0; i < 4; ++i)
            biased[i] = static_cast<std::uint32_t>(int(s[i]) + offset);
          const auto a = grid_detail::coefficient(ax.remainder, ax.denominator);
          const auto b = grid_detail::coefficient(ay.remainder, ay.denominator);
          using grid_detail::interpolate;
          const auto value =
              horizontal_first_
                  ? interpolate(interpolate(biased[0], biased[1], a), interpolate(biased[2], biased[3], a), b)
                  : interpolate(interpolate(biased[0], biased[2], b), interpolate(biased[1], biased[3], b), a);
          output.row(y)[x] = static_cast<T>(int(value) - offset);
        }
      }
    }
  }
};
} // namespace neo_mv
