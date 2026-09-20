#pragma once

#include "core/base/plane.hpp"

#include <algorithm>
#include <cmath>

namespace neo_mv {

// Specs: phase-1/super/kernel-border-extension.md. No allocation or host state.
template <class T>
void extend_border(span2d::Plane<const T> source, span2d::Plane<T> output, std::int32_t working_width,
                   std::int32_t working_height, std::int32_t horizontal_pad, std::int32_t vertical_pad) {
  validate_plane(source);
  validate_plane(output);
  if (working_width < source.width() || working_height < source.height() || horizontal_pad < 0 || vertical_pad < 0)
    throw std::invalid_argument("invalid border geometry");
  const auto width = static_cast<std::int64_t>(working_width) + 2LL * horizontal_pad;
  const auto height = static_cast<std::int64_t>(working_height) + 2LL * vertical_pad;
  if (width > INT32_MAX || height > INT32_MAX)
    throw std::overflow_error("padded dimensions exceed view range");
  if (output.width() != width || output.height() != height)
    throw std::invalid_argument("border output dimensions do not match geometry");
  if (active_rows_overlap(source, output))
    throw std::invalid_argument("border source and output overlap");
  if constexpr (std::is_same_v<T, float>) {
    for (std::int32_t y = 0; y < source.height(); ++y)
      for (std::int32_t x = 0; x < source.width(); ++x)
        if (!std::isfinite(source.row(y)[x]))
          throw std::invalid_argument("non-finite source sample");
  }
  for (std::int32_t y = 0; y < output.height(); ++y) {
    const auto input = source.row(std::clamp(y - vertical_pad, 0, source.height() - 1));
    auto row = output.row(y);
    for (std::int32_t x = 0; x < output.width(); ++x)
      row[x] = input[std::clamp(x - horizontal_pad, 0, source.width() - 1)];
  }
}

} // namespace neo_mv
