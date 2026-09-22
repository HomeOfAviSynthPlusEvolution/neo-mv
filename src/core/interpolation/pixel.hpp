#pragma once

#include "core/base/plane.hpp"
#include "core/mask/numeric.hpp"

#include <vector>

namespace neo_mv {
namespace interpolation_detail {
inline void controls(int time, int forward = 0, int backward = 0) {
  if (time < 0 || time > 256 || forward < 0 || forward > 255 || backward < 0 || backward > 255)
    throw std::invalid_argument("invalid interpolation time or mask");
}
template <class T>
void sample(T value, int bits) {
  mask_detail::validate_storage<T>(bits);
  if constexpr (std::is_same_v<T, float>) {
    if (!std::isfinite(value))
      throw std::invalid_argument("non-finite interpolation sample");
  } else if (value > ((std::uint32_t{1} << bits) - 1)) {
    throw std::invalid_argument("interpolation sample exceeds bit depth");
  }
}
inline float multiply(float a, float b) {
  return mask_detail::binary32(double(a) * double(b));
}
inline float add(float a, float b) {
  return mask_detail::binary32(double(a) + double(b));
}
inline float divide(float a) {
  return mask_detail::binary32(double(a) / 256.0);
}
inline std::uint64_t biased_divide(std::uint64_t value) {
  return (value + 256) / 256;
}
} // namespace interpolation_detail

template <class T, bool Validated = false>
T interpolation_basic(T A, T C, T A0, T C0, int mF, int mB, int t, int bits) {
  using namespace interpolation_detail;
  if constexpr (!Validated) {
    controls(t, mF, mB);
    for (const auto value : {A, C, A0, C0})
      sample(value, bits);
  }
  if constexpr (std::is_same_v<T, float>) {
    const auto innerU = divide(multiply(float(mF), add(multiply(C, float(256 - mB)), multiply(float(mB), A0))));
    const auto U = divide(add(multiply(A, float(256 - mF)), innerU));
    const auto innerV = divide(multiply(float(mB), add(multiply(A, float(256 - mF)), multiply(float(mF), C0))));
    const auto V = divide(add(multiply(C, float(256 - mB)), innerV));
    return divide(add(multiply(U, float(256 - t)), multiply(V, float(t))));
  } else {
    const auto a = std::uint64_t(A), c = std::uint64_t(C);
    const auto U = biased_divide(a * (256 - mF) + biased_divide(mF * (c * (256 - mB) + std::uint64_t(mB) * A0)));
    const auto V = biased_divide(c * (256 - mB) + biased_divide(mB * (a * (256 - mF) + std::uint64_t(mF) * C0)));
    return static_cast<T>((U * (256 - t) + V * t) / 256 - 1);
  }
}

template <class T, bool Validated = false>
T interpolation_extra(T A, T C, T E, T K, int mF, int mB, int t, int bits) {
  using namespace interpolation_detail;
  if constexpr (!Validated) {
    controls(t, mF, mB);
    for (const auto value : {A, C, E, K})
      sample(value, bits);
  }
  // std::min/max retain their first operand at an equal-value comparison.
  const auto lo = std::min(A, C), hi = std::max(A, C);
  const auto CK = std::max(lo, std::min(K, hi)), CE = std::max(lo, std::min(E, hi));
  if constexpr (std::is_same_v<T, float>) {
    const auto U = divide(add(multiply(CK, float(mF)), multiply(A, float(256 - mF))));
    const auto V = divide(add(multiply(CE, float(mB)), multiply(C, float(256 - mB))));
    return divide(add(multiply(U, float(256 - t)), multiply(V, float(t))));
  } else {
    const auto U = biased_divide(std::uint64_t(CK) * mF + std::uint64_t(A) * (256 - mF));
    const auto V = biased_divide(std::uint64_t(CE) * mB + std::uint64_t(C) * (256 - mB));
    return static_cast<T>((U * (256 - t) + V * t) / 256 - 1);
  }
}

template <class T>
T interpolation_blend(T first, T second, int t, int bits) {
  using namespace interpolation_detail;
  controls(t);
  sample(first, bits);
  sample(second, bits);
  if constexpr (std::is_same_v<T, float>)
    return divide(add(multiply(first, float(256 - t)), multiply(second, float(t))));
  else
    return static_cast<T>((std::uint64_t(first) * (256 - t) + std::uint64_t(second) * t) / 256);
}

template <class T>
void interpolation_blend_planes(span2d::Plane<const T> first, span2d::Plane<const T> second, span2d::Plane<T> output,
                                int t, int bits) {
  interpolation_detail::controls(t);
  mask_detail::validate_storage<T>(bits);
  validate_plane(first);
  validate_plane(second);
  validate_plane(output);
  if (first.width() != output.width() || first.height() != output.height() || second.width() != output.width() ||
      second.height() != output.height() || active_rows_overlap(first, output) || active_rows_overlap(second, output))
    throw std::invalid_argument("fallback blend storage mismatch or output alias");
  // Keep caller storage untouched if a required zero-weight sample or any
  // intermediate is invalid. The two source images may alias each other.
  const auto count = std::uint64_t(output.width()) * output.height();
  if (count > std::vector<T>().max_size())
    throw std::overflow_error("fallback blend storage size is unrepresentable");
  std::vector<T> values(static_cast<std::size_t>(count));
  for (int y = 0; y < output.height(); ++y)
    for (int x = 0; x < output.width(); ++x)
      values[std::size_t(y) * output.width() + x] = interpolation_blend(first.row(y)[x], second.row(y)[x], t, bits);
  for (int y = 0; y < output.height(); ++y)
    std::copy_n(values.data() + std::size_t(y) * output.width(), output.width(), output.row(y).data());
}
} // namespace neo_mv
