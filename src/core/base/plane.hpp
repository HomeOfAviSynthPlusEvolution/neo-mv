#pragma once

#include <dualsynth/span2d.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace neo_mv {

template <class T>
inline constexpr bool supported_sample =
    std::is_same_v<std::remove_const_t<T>, std::uint8_t> || std::is_same_v<std::remove_const_t<T>, std::uint16_t> ||
    std::is_same_v<std::remove_const_t<T>, float>;

// The caller owns live T objects in each active row for the entire use of the
// view. extent_bytes bounds the address span, not the accessibility of row gaps.
// Validate raw byte stride before span2d can divide or narrow it.
template <class T>
span2d::Plane<T> checked_plane(T* data, std::int32_t width, std::int32_t height, std::ptrdiff_t stride_bytes,
                               std::size_t extent_bytes) {
  static_assert(supported_sample<T>, "unsupported sample storage");
  if (!data || width <= 0 || height <= 0 || stride_bytes <= 0)
    throw std::invalid_argument("invalid plane dimensions, address or stride");
  const auto address = reinterpret_cast<std::uintptr_t>(data);
  const auto stride = static_cast<std::uintmax_t>(stride_bytes);
  const auto row_bytes = static_cast<std::uintmax_t>(width) * sizeof(T);
  if (address % alignof(T) != 0 || stride % sizeof(T) != 0 || stride < row_bytes)
    throw std::invalid_argument("invalid plane alignment or row stride");
  if (stride / sizeof(T) > static_cast<std::uintmax_t>(INT32_MAX))
    throw std::overflow_error("plane element stride exceeds span2d range");
  const auto limit = static_cast<std::uintmax_t>(std::numeric_limits<std::ptrdiff_t>::max());
  if (row_bytes > limit || static_cast<std::uintmax_t>(height - 1) > (limit - row_bytes) / stride)
    throw std::overflow_error("plane address span is unrepresentable");
  const auto extent = static_cast<std::uintmax_t>(height - 1) * stride + row_bytes;
  if (extent > extent_bytes || extent > std::numeric_limits<std::uintptr_t>::max() - address)
    throw std::invalid_argument("plane exceeds declared storage extent");
  return span2d::make_plane(data, width, height, stride_bytes);
}

template <class T>
void validate_plane(span2d::Plane<T> plane) {
  // This validates a view already constructed from a representable element
  // stride. It cannot recover an invalid byte stride truncated by its caller.
  const auto bytes = static_cast<std::int64_t>(plane.stride()) * static_cast<std::int64_t>(sizeof(T));
  if (bytes <= 0 || bytes > std::numeric_limits<std::ptrdiff_t>::max())
    throw std::invalid_argument("plane byte stride is not representable");
  checked_plane(plane.data(), plane.width(), plane.height(), static_cast<std::ptrdiff_t>(bytes),
                std::numeric_limits<std::size_t>::max());
}

template <class A, class B>
bool active_rows_overlap(span2d::Plane<A> a, span2d::Plane<B> b) {
  // Both views must already be validated. Gaps are not part of either image.
  std::int32_t y = 0, z = 0;
  const auto a_base = reinterpret_cast<std::uintptr_t>(a.data());
  const auto b_base = reinterpret_cast<std::uintptr_t>(b.data());
  while (y < a.height() && z < b.height()) {
    const auto ab = a_base + static_cast<std::uintptr_t>(y) * a.stride_bytes();
    const auto bb = b_base + static_cast<std::uintptr_t>(z) * b.stride_bytes();
    const auto ae = ab + static_cast<std::uintptr_t>(a.width()) * sizeof(A);
    const auto be = bb + static_cast<std::uintptr_t>(b.width()) * sizeof(B);
    if (ab < be && bb < ae)
      return true;
    if (ae <= bb)
      ++y;
    else
      ++z;
  }
  return false;
}

} // namespace neo_mv
