#pragma once
#include "core/base/plane.hpp"
#include <cstdlib>

namespace neo_mv {
struct SadReferenceSum { std::int64_t sad = 0, reference_sum = 0; };
template <class T>
std::int64_t block_luma_sum(span2d::Plane<const T> plane) {
  static_assert(std::is_integral_v<T>);
  std::int64_t result = 0;
  for (int y = 0; y < plane.height(); ++y)
    for (int x = 0; x < plane.width(); ++x) result += plane.row(y)[x];
  return result;
}
template <class T>
SadReferenceSum sad_and_reference_sum(span2d::Plane<const T> source, span2d::Plane<const T> reference) {
  static_assert(std::is_integral_v<T>);
  SadReferenceSum result;
  for (int y = 0; y < source.height(); ++y)
    for (int x = 0; x < source.width(); ++x) {
      const auto r = reference.row(y)[x];
      result.sad += std::abs(int(source.row(y)[x]) - int(r));
      result.reference_sum += r;
    }
  return result;
}
} // namespace neo_mv
