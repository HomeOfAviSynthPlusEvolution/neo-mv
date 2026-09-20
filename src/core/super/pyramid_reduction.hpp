#pragma once

#include "core/base/plane.hpp"

#include <cmath>

namespace neo_mv {
namespace reduction_detail {
template <class T>
using Accumulator = std::conditional_t<std::is_same_v<T, float>, float, std::uint64_t>;

template <class T>
T rounded(Accumulator<T> value, int shift) {
  if constexpr (std::is_same_v<T, float>) {
    if (!std::isfinite(value))
      throw std::invalid_argument("non-finite pyramid reduction intermediate");
    return value / static_cast<float>(1U << shift);
  } else {
    return static_cast<T>((value + (std::uint64_t{1} << (shift - 1))) >> shift);
  }
}

template <class T, class Read>
T filter_axis(Read read, int output_index, int output_size, int filter) {
  using A = Accumulator<T>;
  const int center = 2 * output_index;
  const auto at = [&](int offset) -> A {
    return read(center + offset);
  };
  if (output_index == 0 || output_index == output_size - 1)
    return rounded<T>(at(0) + at(1), 1);
  if (filter == 1)
    return rounded<T>((at(-1) + A(3) * (at(0) + at(1))) + at(2), 3);
  return rounded<T>(at(-2) + ((at(3) + A(10) * (at(0) + at(1))) + A(5) * (at(-1) + at(2))), 5);
}
} // namespace reduction_detail

// Source is the complete padded integer plane. origin_x/y locate its working
// origin. Output contains only the new working rectangle; extend_border follows.
// Filters 1/2 require caller-owned scratch of exactly 2*output.width by output.height.
// Filter 0 neither requires nor touches scratch. All active writable regions are disjoint.
template <class T>
void reduce_pyramid(span2d::Plane<const T> source, std::int32_t origin_x, std::int32_t origin_y,
                    span2d::Plane<T> output, int filter, span2d::Plane<T> scratch = {}) {
  validate_plane(source);
  validate_plane(output);
  if (filter < 0 || filter > 2 || origin_x < 0 || origin_y < 0)
    throw std::invalid_argument("invalid reduction filter or origin");
  const auto read_width = 2LL * output.width(), read_height = 2LL * output.height();
  // Every interior six-tap footprint is also contained in this rectangle.
  if (origin_x + read_width > source.width() || origin_y + read_height > source.height())
    throw std::invalid_argument("reduction reads outside the source plane");
  if (active_rows_overlap(source, output))
    throw std::invalid_argument("reduction source and output overlap");
  if (filter != 0) {
    validate_plane(scratch);
    if (scratch.width() != read_width || scratch.height() != output.height())
      throw std::invalid_argument("incorrect reduction scratch dimensions");
    if (active_rows_overlap(source, scratch) || active_rows_overlap(output, scratch))
      throw std::invalid_argument("reduction scratch overlaps input or output");
  }
  const auto read = [&](int x, int y) -> T {
    return source.row(origin_y + y)[origin_x + x];
  };
  // Check finite source values before modifying any output/scratch. Check only
  // the addressed rectangle, never allocation gaps or unrelated padding.
  if constexpr (std::is_same_v<T, float>) {
    for (int y = 0; y < read_height; ++y)
      for (int x = 0; x < read_width; ++x)
        if (!std::isfinite(read(x, y)))
          throw std::invalid_argument("non-finite pyramid reduction sample");
  }
  if (filter == 0) {
    using A = reduction_detail::Accumulator<T>;
    for (int y = 0; y < output.height(); ++y)
      for (int x = 0; x < output.width(); ++x)
        output.row(y)[x] = reduction_detail::rounded<T>(
            ((A(read(2 * x, 2 * y)) + A(read(2 * x + 1, 2 * y))) + A(read(2 * x + 1, 2 * y + 1))) +
                A(read(2 * x, 2 * y + 1)),
            2);
    return;
  }
  for (int y = 0; y < output.height(); ++y)
    for (int x = 0; x < scratch.width(); ++x)
      scratch.row(y)[x] =
          reduction_detail::filter_axis<T>([&](int sy) { return read(x, sy); }, y, output.height(), filter);
  for (int y = 0; y < output.height(); ++y)
    for (int x = 0; x < output.width(); ++x)
      output.row(y)[x] =
          reduction_detail::filter_axis<T>([&](int sx) { return scratch.row(y)[sx]; }, x, output.width(), filter);
}

} // namespace neo_mv
