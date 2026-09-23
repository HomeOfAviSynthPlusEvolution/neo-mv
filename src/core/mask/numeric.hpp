#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <cfenv>
#if defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#endif

namespace neo_mv::mask_detail {

// Convert parameters to binary32 using ties-to-even, including the narrow
// interval above FLT_MAX that still rounds to FLT_MAX. Casting an out-of-range
// binary64 value directly to float need not have defined C++ behavior.
inline float binary32_exact(double value) {
  static_assert(sizeof(double) == 8 && sizeof(float) == 4 && std::numeric_limits<double>::is_iec559 &&
                std::numeric_limits<float>::is_iec559);
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  const auto exponent = static_cast<int>((bits >> 52) & 0x7ff);
  if (exponent == 0x7ff)
    throw std::invalid_argument("mask parameter must be finite");
  const int e = exponent - 1023;
  if (e > 127)
    throw std::invalid_argument("mask parameter rounds outside finite binary32");
  std::uint32_t encoded = static_cast<std::uint32_t>(bits >> 32) & 0x80000000u;
  if (e >= -150) {
    // Round the significand directly. This also handles subnormal results and
    // does not depend on the host rounding mode or flush-to-zero settings.
    const auto significand = (bits & 0xfffffffffffffull) | (std::uint64_t{1} << 52);
    const int shift = e >= -126 ? 29 : -e - 97;
    auto rounded = significand >> shift;
    const auto remainder = significand & ((std::uint64_t{1} << shift) - 1);
    const auto half = std::uint64_t{1} << (shift - 1);
    rounded += remainder > half || (remainder == half && (rounded & 1));
    const auto magnitude =
        static_cast<std::uint32_t>(rounded) + (e >= -126 ? static_cast<std::uint32_t>(e + 126) << 23 : 0);
    if (magnitude >= 0x7f800000u)
      throw std::invalid_argument("mask parameter rounds outside finite binary32");
    encoded |= magnitude;
  }
  float result;
  std::memcpy(&result, &encoded, sizeof(result));
  return result;
}

// Keep the common conversion small enough to inline into arithmetic loops.
inline bool nearest_rounding() {
#if defined(__SSE2__) || defined(_M_X64)
  return (_mm_getcsr() & 0x6000u) == 0;
#else
  return std::fegetround() == FE_TONEAREST;
#endif
}
inline float binary32(double value, bool nearest) {
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  const auto exponent = (bits >> 52) & 0x7ff;
  // Input and result are normal and in range; FTZ/DAZ cannot affect them.
  if (exponent >= 897 && exponent < 1150 && nearest) {
#if defined(__SSE2__) || defined(_M_X64)
    return _mm_cvtss_f32(_mm_cvtsd_ss(_mm_setzero_ps(), _mm_set_sd(value)));
#else
    return static_cast<float>(value);
#endif
  }
  return binary32_exact(value);
}
inline float binary32(double value) {
  return binary32(value, nearest_rounding());
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
