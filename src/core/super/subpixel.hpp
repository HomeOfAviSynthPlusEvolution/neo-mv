#pragma once

#include "core/base/plane.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace neo_mv {

// Borrowed logical phase views: index = ay * pel + ax. A built-in quarter
// phase may be narrower/shorter than its allocation; consumers see only valid samples.
template <class T>
struct SubpixelPhases {
  int pel;
  std::array<span2d::Plane<const T>, 16> planes{};
};

namespace subpixel_detail {
template <class T>
using Acc = std::conditional_t<std::is_same_v<T, float>, float, std::int64_t>;

template <class T>
std::int64_t sample_max(int bits) {
  if constexpr (std::is_same_v<T, float>) {
    if (bits != 32)
      throw std::invalid_argument("float phases require 32-bit precision");
    return 0;
  } else {
    if ((sizeof(T) == 1 && bits != 8) || (sizeof(T) == 2 && (bits < 9 || bits > 16)))
      throw std::invalid_argument("phase precision does not match sample storage");
    return (std::int64_t{1} << bits) - 1;
  }
}

template <class T>
void valid_sample(T value, std::int64_t maximum) {
  if constexpr (std::is_same_v<T, float>) {
    if (!std::isfinite(value))
      throw std::invalid_argument("non-finite phase sample");
  } else if (value > maximum) {
    throw std::invalid_argument("phase sample exceeds bit depth");
  }
}

template <class T>
T round_clip(Acc<T> value, int shift, std::int64_t maximum) {
  if constexpr (std::is_same_v<T, float>) {
    if (!std::isfinite(value))
      throw std::invalid_argument("non-finite phase intermediate");
    return value / static_cast<float>(1U << shift);
  } else {
    const auto biased = value + (std::int64_t{1} << (shift - 1));
    const auto divisor = std::int64_t{1} << shift;
    const auto rounded = biased / divisor - (biased % divisor < 0 ? 1 : 0);
    return static_cast<T>(std::clamp(rounded, std::int64_t{0}, maximum));
  }
}

template <class T>
T average(T a, T b, std::int64_t maximum) {
  return round_clip<T>(Acc<T>(a) + Acc<T>(b), 1, maximum);
}

template <class T, class Read>
T half(Read read, int i, int length, int sharp, bool vertical, std::int64_t maximum) {
  using A = Acc<T>;
  const auto at = [&](int offset) -> A {
    return read(i + offset);
  };
  if (i == length - 1)
    return read(i);
  if (sharp == 1 && i >= 1 && i < length - 3) {
    if constexpr (std::is_same_v<T, float>) {
      const A ends = vertical ? ((-at(-1)) - at(2)) : -(at(-1) + at(2));
      return round_clip<T>(ends + A(9) * (at(0) + at(1)), 4, maximum);
    } else {
      return round_clip<T>((A(9) * (at(0) + at(1)) - at(-1)) - at(2), 4, maximum);
    }
  }
  if (sharp == 2 && i >= 2 && i < length - 4) {
    if constexpr (std::is_same_v<T, float>)
      return round_clip<T>(at(-2) + (at(3) + A(5) * (A(4) * (at(0) + at(1)) - (at(-1) + at(2)))), 5, maximum);
    else
      return round_clip<T>(((at(-2) + at(3)) + A(20) * (at(0) + at(1))) - A(5) * (at(-1) + at(2)), 5, maximum);
  }
  return average<T>(read(i), read(i + 1), maximum);
}

template <class T>
void validate_storage(span2d::Plane<const T> base, int pel, const std::array<span2d::Plane<T>, 16>& storage) {
  validate_plane(base);
  if (pel != 1 && pel != 2 && pel != 4)
    throw std::invalid_argument("invalid subpixel factor");
  for (int i = 1; i < pel * pel; ++i) {
    validate_plane(storage[i]);
    if (storage[i].width() != base.width() || storage[i].height() != base.height())
      throw std::invalid_argument("incorrect phase storage dimensions");
    if (active_rows_overlap(base, storage[i]))
      throw std::invalid_argument("phase overwrites base");
    for (int j = 1; j < i; ++j)
      if (active_rows_overlap(storage[j], storage[i]))
        throw std::invalid_argument("phase outputs overlap");
  }
}
} // namespace subpixel_detail

// No allocations: phase zero borrows base; callers supply distinct full-size
// output planes for nonzero phases. All returned views share those lifetimes.
template <class T>
SubpixelPhases<T> interpolate_subpixels(span2d::Plane<const T> base, int pel, int sharp, int bits,
                                        const std::array<span2d::Plane<T>, 16>& storage) {
  using namespace subpixel_detail;
  validate_storage(base, pel, storage);
  const auto maximum = sample_max<T>(bits);
  if (sharp < 0 || sharp > 2)
    throw std::invalid_argument("invalid interpolation sharpness");
  const int width = base.width(), height = base.height();
  if (pel > 1 && (width < 2 * (sharp + 1) || height < 2 * (sharp + 1)))
    throw std::invalid_argument("padded plane too small for interpolation");
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      valid_sample(base.row(y)[x], maximum);
  SubpixelPhases<T> result{pel, {}};
  result.planes[0] = base;
  if (pel == 1)
    return result;
  const int half_phase = pel / 2;
  auto horizontal = storage[half_phase];
  auto vertical = storage[half_phase * pel];
  auto diagonal = storage[half_phase * pel + half_phase];
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      horizontal.row(y)[x] = half<T>([&](int xx) { return base.row(y)[xx]; }, x, width, sharp, false, maximum);
      vertical.row(y)[x] = half<T>([&](int yy) { return base.row(yy)[x]; }, y, height, sharp, true, maximum);
    }
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      if (sharp != 0) {
        diagonal.row(y)[x] = half<T>([&](int xx) { return vertical.row(y)[xx]; }, x, width, sharp, false, maximum);
      } else if (x == width - 1 && y == height - 1) {
        diagonal.row(y)[x] = base.row(y)[x];
      } else if (x == width - 1) {
        diagonal.row(y)[x] = average<T>(base.row(y)[x], base.row(y + 1)[x], maximum);
      } else if (y == height - 1) {
        diagonal.row(y)[x] = average<T>(base.row(y)[x], base.row(y)[x + 1], maximum);
      } else {
        using A = Acc<T>;
        diagonal.row(y)[x] = round_clip<T>(((A(base.row(y)[x]) + A(base.row(y)[x + 1])) + A(base.row(y + 1)[x])) +
                                               A(base.row(y + 1)[x + 1]),
                                           2, maximum);
      }
    }
  if (pel == 4) {
    const auto phase = [&](int ax, int ay) -> span2d::Plane<const T> {
      return ax == 0 && ay == 0 ? base : span2d::Plane<const T>(storage[ay * pel + ax]);
    };
    const auto blend = [&](int ax, int ay, int lx, int ly, int rx, int ry, int dx = 0, int dy = 0) {
      const auto left = phase(lx, ly), right = phase(rx, ry);
      auto dst = storage[ay * pel + ax];
      const int w = width - (ax == 3), h = height - (ay == 3);
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
          dst.row(y)[x] = average<T>(left.row(y + dy)[x + dx], right.row(y)[x], maximum);
    };
    blend(1, 0, 0, 0, 2, 0);
    blend(1, 2, 0, 2, 2, 2);
    blend(0, 1, 0, 0, 0, 2);
    blend(2, 1, 2, 0, 2, 2);
    blend(1, 1, 0, 1, 2, 1);
    blend(3, 0, 0, 0, 2, 0, 1);
    blend(3, 2, 0, 2, 2, 2, 1);
    blend(3, 1, 0, 1, 2, 1, 1);
    blend(0, 3, 0, 0, 0, 2, 0, 1);
    blend(2, 3, 2, 0, 2, 2, 0, 1);
    blend(1, 3, 0, 3, 2, 3);
    blend(3, 3, 0, 3, 2, 3, 1);
  }
  for (int ay = 0; ay < pel; ++ay)
    for (int ax = 0; ax < pel; ++ax) {
      if (ax == 0 && ay == 0)
        continue;
      result.planes[ay * pel + ax] =
          storage[ay * pel + ax].subplane(0, 0, width - (pel == 4 && ax == 3), height - (pel == 4 && ay == 3));
    }
  return result;
}

// External fractional phases use full domains. Integer phase always borrows base.
template <class T>
SubpixelPhases<T> extract_external_subpixels(span2d::Plane<const T> base, span2d::Plane<const T> external,
                                             int actual_width, int actual_height, int pad_x, int pad_y, int pel,
                                             int bits, const std::array<span2d::Plane<T>, 16>& storage) {
  using namespace subpixel_detail;
  validate_storage(base, pel, storage);
  const auto maximum = sample_max<T>(bits);
  if (actual_width <= 0 || actual_height <= 0 || pad_x < 0 || pad_y < 0 ||
      static_cast<std::int64_t>(actual_width) + 2LL * pad_x > base.width() ||
      static_cast<std::int64_t>(actual_height) + 2LL * pad_y > base.height())
    throw std::invalid_argument("invalid external phase geometry");
  SubpixelPhases<T> result{pel, {}};
  result.planes[0] = base;
  for (int y = 0; y < base.height(); ++y)
    for (int x = 0; x < base.width(); ++x)
      valid_sample(base.row(y)[x], maximum);
  if (pel == 1)
    return result; // The external pixels are unused for pel=1.
  validate_plane(external);
  if (external.width() != static_cast<std::int64_t>(pel) * actual_width ||
      external.height() != static_cast<std::int64_t>(pel) * actual_height)
    throw std::invalid_argument("incorrect external pel dimensions");
  for (int i = 1; i < pel * pel; ++i)
    if (active_rows_overlap(external, storage[i]))
      throw std::invalid_argument("phase output overwrites external input");
  for (int ay = 0; ay < pel; ++ay)
    for (int ax = 0; ax < pel; ++ax) {
      if (ax == 0 && ay == 0)
        continue;
      auto dst = storage[ay * pel + ax];
      for (int y = 0; y < dst.height(); ++y)
        for (int x = 0; x < dst.width(); ++x) {
          const auto sx = pel * std::clamp(x - pad_x, 0, actual_width - 1) + ax;
          const auto sy = pel * std::clamp(y - pad_y, 0, actual_height - 1) + ay;
          const auto value = external.row(sy)[sx];
          valid_sample(value, maximum);
          dst.row(y)[x] = value;
        }
      result.planes[ay * pel + ax] = dst;
    }
  return result;
}

} // namespace neo_mv
