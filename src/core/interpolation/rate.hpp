#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>

namespace neo_mv {
namespace rate_detail {
// A 32-bit multiplier times a positive 63-bit factor, represented exactly.
struct Product {
  std::uint64_t high, low;
  static Product make(std::uint32_t multiplier, std::uint64_t factor) {
    const auto a = std::uint64_t(multiplier) * std::uint32_t(factor);
    const auto b = std::uint64_t(multiplier) * (factor >> 32);
    const auto low = a + (b << 32);
    return {(b >> 32) + (low < a), low};
  }
  bool less(Product other) const { return high < other.high || (high == other.high && low < other.low); }
};

inline std::int64_t multiply(std::int64_t a, std::int64_t b) {
  if (a > INT64_MAX / b)
    throw std::invalid_argument("reduced frame-rate component exceeds int64");
  return a * b;
}

// Binary search compares exact products instead of forming a possibly
// overflowing numerator. Both callers require a result within signed32.
inline std::int64_t quotient(std::uint32_t multiplier, std::uint64_t factor, std::uint64_t divisor) {
  const auto numerator = Product::make(multiplier, factor);
  std::uint32_t low = 0, high = std::uint32_t(INT32_MAX) + 1;
  if (!numerator.less(Product::make(high, divisor)))
    throw std::invalid_argument("frame-rate mapped count or index exceeds signed32");
  while (high - low > 1) {
    const auto middle = low + (high - low) / 2;
    if (numerator.less(Product::make(middle, divisor)))
      high = middle;
    else
      low = middle;
  }
  return low;
}
} // namespace rate_detail

struct FrameRatePosition {
  std::int64_t left, right;
  int time;
  std::optional<std::int64_t> endpoint;
};

class FrameRatePlan {
  std::int64_t source_frames_, output_num_, output_den_, ratio_num_, ratio_den_, output_frames_;
  std::int32_t distance_;

public:
  FrameRatePlan(std::int64_t frames, std::int64_t input_num, std::int64_t input_den, std::int64_t num = 25,
                std::int64_t den = 1, std::int32_t distance = 1)
      : source_frames_(frames), distance_(distance) {
    if (frames < 1 || frames > INT32_MAX || input_num <= 0 || input_den <= 0 || num < 0 || den < 0 || distance < 1)
      throw std::invalid_argument("invalid frame-rate creation parameters");
    auto common = std::gcd(input_num, input_den);
    input_num /= common;
    input_den /= common;
    if (num == 0 || den == 0) {
      common = std::gcd(input_den, std::int64_t{2});
      output_num_ = rate_detail::multiply(input_num, 2 / common);
      output_den_ = input_den / common;
    } else {
      common = std::gcd(num, den);
      output_num_ = num / common;
      output_den_ = den / common;
    }
    // Each original rate is already reduced. Cross cancellation therefore
    // reduces the entire ratio before either product is computed.
    auto i = input_num, j = input_den, p = output_num_, q = output_den_;
    common = std::gcd(i, p);
    i /= common;
    p /= common;
    common = std::gcd(q, j);
    q /= common;
    j /= common;
    ratio_num_ = rate_detail::multiply(i, q);
    ratio_den_ = rate_detail::multiply(j, p);
    output_frames_ = rate_detail::quotient(static_cast<std::uint32_t>(frames), ratio_den_, ratio_num_);
    if (output_frames_ == 0)
      throw std::invalid_argument("frame-rate conversion produces zero frames");
  }

  std::int64_t output_num() const { return output_num_; }
  std::int64_t output_den() const { return output_den_; }
  std::int64_t output_frames() const { return output_frames_; }
  std::int64_t ratio_num() const { return ratio_num_; }
  std::int64_t ratio_den() const { return ratio_den_; }

  FrameRatePosition position(std::int64_t n) const {
    if (n < 0 || n >= output_frames_)
      throw std::invalid_argument("frame-rate output index outside video");
    const auto left = rate_detail::quotient(static_cast<std::uint32_t>(n), ratio_num_, ratio_den_);
    const auto right = left + std::int64_t(distance_);
    const double product = double(n) * double(ratio_num_);
    const double u = product / double(ratio_den_);
    const double e = u - double(left);
    const double scaled = e * 256.0;
    const double biased = scaled + 0.5;
    const double integral = std::trunc(biased);
    if (!std::isfinite(product) || !std::isfinite(u) || !std::isfinite(e) || !std::isfinite(scaled) ||
        !std::isfinite(biased) || integral < 0 || integral > 256)
      throw std::invalid_argument("frame-rate time coefficient outside domain");
    const int time = static_cast<int>(integral) / distance_;
    std::optional<std::int64_t> endpoint;
    if (time == 0)
      endpoint = std::min(left, source_frames_ - 1);
    else if (time == 256)
      endpoint = std::min(right, source_frames_ - 1);
    return {left, right, time, endpoint};
  }
};
} // namespace neo_mv
