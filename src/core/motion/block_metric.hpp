#pragma once

#include "core/base/plane.hpp"

#include <array>
#include <cmath>
#include <string_view>

namespace neo_mv {

enum class BlockMetric { sad, satd };

inline BlockMetric parse_block_metric(std::string_view value) {
  if (value == "sad") return BlockMetric::sad;
  if (value == "satd") return BlockMetric::satd;
  throw std::invalid_argument("metric must be sad or satd");
}

namespace metric_detail {
template <class T>
T finite(T value) {
  if constexpr (std::is_same_v<T, float>)
    if (!std::isfinite(value))
      throw std::invalid_argument("nonfinite block metric intermediate or sample");
  return value;
}

// Integer transform intermediates are bounded by 16 * 65535. Only the
// nonnegative sum over arbitrarily many pixels/cells needs an overflow check.
template <class T>
T add(T a, T b) {
  return finite(T(a + b));
}
template <class T>
T subtract(T a, T b) {
  return finite(T(a - b));
}
template <class T>
T magnitude(T value) {
  return value < T(0) ? -value : value;
}
template <class T>
T accumulate(T total, T value) {
  if constexpr (std::is_same_v<T, std::int64_t>)
    if (value > INT64_MAX - total)
      throw std::overflow_error("block metric exceeds int64 range");
  return add(total, value);
}

template <class T>
std::array<T, 4> hadamard(const std::array<T, 4>& z) {
  const T a = add(z[0], z[1]), b = add(z[2], z[3]);
  const T c = subtract(z[0], z[1]), d = subtract(z[2], z[3]);
  return {add(a, b), add(c, d), subtract(a, b), subtract(c, d)};
}
} // namespace metric_detail

// Encode a complete float plane metric, never an individual SATD cell.
inline std::int64_t encode_float_error(float error) {
  const float z = metric_detail::finite(metric_detail::finite(error) * 65535.0f);
  if (z <= 0.0f)
    return 0;
  if (z >= 4294967040.0f)
    return 4294967295LL;
  return static_cast<std::int64_t>(z + 0.5f);
}

// The views are already sampled, identically sized rectangles. Sampling,
// whole-candidate-domain admission and luma/chroma assembly are separate steps.
// No nominal-range clipping or bit-depth normalization is applied here.
template <class T, bool Validated = false>
std::int64_t block_metric(span2d::Plane<const T> source, span2d::Plane<const T> reference, BlockMetric metric) {
  static_assert(supported_sample<T>, "unsupported sample storage");
  if constexpr (!Validated) {
    validate_plane(source);
    validate_plane(reference);
    if (source.width() != reference.width() || source.height() != reference.height())
      throw std::invalid_argument("block metric rectangle size mismatch");
    if (metric != BlockMetric::sad && metric != BlockMetric::satd)
      throw std::invalid_argument("unknown block metric");
    if (metric == BlockMetric::satd && (source.width() % 4 != 0 || source.height() % 4 != 0))
      throw std::invalid_argument("SATD requires complete 4x4 cells");
  }
  using A = std::conditional_t<std::is_same_v<T, float>, float, std::int64_t>;
  using namespace metric_detail;
  const auto difference = [&](int x, int y) -> A {
    const A s = finite(A(source.row(y)[x]));
    const A r = finite(A(reference.row(y)[x]));
    return subtract(s, r);
  };
  A total = A(0);
  if (metric == BlockMetric::sad) {
    for (int y = 0; y < source.height(); ++y)
      for (int x = 0; x < source.width(); ++x)
        total = accumulate(total, magnitude(difference(x, y)));
  } else {
    for (int y = 0; y < source.height(); y += 4)
      for (int x = 0; x < source.width(); x += 4) {
        std::array<std::array<A, 4>, 4> rows{};
        for (int j = 0; j < 4; ++j)
          rows[j] = hadamard<A>(
              {difference(x, y + j), difference(x + 1, y + j), difference(x + 2, y + j), difference(x + 3, y + j)});
        A cell = A(0);
        for (int i = 0; i < 4; ++i) {
          const auto column = hadamard<A>({rows[0][i], rows[1][i], rows[2][i], rows[3][i]});
          const A sum =
              add(add(add(magnitude(column[0]), magnitude(column[1])), magnitude(column[2])), magnitude(column[3]));
          if constexpr (std::is_same_v<T, float>)
            total = accumulate(total, sum); // one global total, in column order
          else
            cell += sum;
        }
        if constexpr (!std::is_same_v<T, float>)
          total = accumulate(total, cell / 2);
      }
    if constexpr (std::is_same_v<T, float>)
      total = finite(total * 0.5f);
  }
  if constexpr (std::is_same_v<T, float>)
    return encode_float_error(total);
  else
    return total;
}

} // namespace neo_mv
