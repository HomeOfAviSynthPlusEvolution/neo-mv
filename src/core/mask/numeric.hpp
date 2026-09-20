#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace neo_mv::mask_detail {

// Convert parameters to binary32 using ties-to-even, including the narrow
// interval above FLT_MAX that still rounds to FLT_MAX. Casting an out-of-range
// binary64 value directly to float need not have defined C++ behavior.
inline float binary32(double value) {
  if (!std::isfinite(value))
    throw std::invalid_argument("mask parameter must be finite");
  const double magnitude = std::abs(value);
  if (magnitude >= 0x1.ffffffp127)
    throw std::invalid_argument("mask parameter rounds outside finite binary32");
  if (magnitude == 0)
    return std::signbit(value) ? -0.0f : 0.0f;
  int exponent = 0;
  std::frexp(magnitude, &exponent);
  const int shift = std::max(exponent - 24, -149);
  const double scaled = std::ldexp(magnitude, -shift);
  double rounded = std::floor(scaled);
  const double remainder = scaled - rounded;
  if (remainder > 0.5 || (remainder == 0.5 && std::fmod(rounded, 2.0) != 0))
    rounded += 1;
  const float result = static_cast<float>(std::ldexp(rounded, shift));
  return std::signbit(value) ? -result : result;
}

template <class T>
void validate_storage(int bits) {
  static_assert(std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::uint16_t> || std::is_same_v<T, float>);
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    if (bits != 8)
      throw std::invalid_argument("mask precision does not match uint8 storage");
  } else if constexpr (std::is_same_v<T, std::uint16_t>) {
    if (bits < 9 || bits > 16)
      throw std::invalid_argument("mask precision does not match uint16 storage");
  } else if (bits != 32) {
    throw std::invalid_argument("mask precision does not match float storage");
  }
}

template <class F>
F checked_power(F base, F exponent) {
  if (!std::isfinite(base) || !std::isfinite(exponent) || base < 0 || exponent < 0)
    throw std::invalid_argument("mask power requires finite nonnegative arguments");
  if (exponent == 0)
    return F{1};
  if (base == 0)
    return F{0};
  if (exponent == 1)
    return base;
  const F result = std::pow(base, exponent);
  if (!std::isfinite(result) || result < 0)
    throw std::overflow_error("mask power result is not finite");
  return result;
}
inline float power(float base, float exponent) {
  return checked_power(base, exponent);
}
inline double power(double base, double exponent) {
  return checked_power(base, exponent);
}

template <class T, class F>
T quantize(F score, int bits) {
  static_assert(std::is_floating_point_v<F>);
  validate_storage<T>(bits);
  if (!std::isfinite(score) || score < 0)
    throw std::invalid_argument("mask score must be finite and nonnegative");
  if constexpr (std::is_same_v<T, float>)
    return binary32(static_cast<double>(std::min(score, F{1})));
  else
    return static_cast<T>(std::min(score, static_cast<F>((std::uint32_t{1} << bits) - 1)));
}

} // namespace neo_mv::mask_detail
