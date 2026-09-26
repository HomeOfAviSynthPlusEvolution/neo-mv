#pragma once
#include "core/motion/dct_constants.hpp"
#include <algorithm>
#include <cmath>

namespace neo_mv::dct_detail {
inline int padded_stride(int n) {
  return (n + 7) & ~7;
}
// All float buffers have padded_stride(width)*padded_stride(height) elements.
// Input is centered integer pixels, stored at padded_stride(width). Output is
// normalized AC (4*sum(cos*cos*pixels)/(sqrt(2)*area)) at that same stride.
// Rows is scratch. Buffers must not overlap; padding may be overwritten.
void transform_block(int width, int height, const float* input, float* rows, float* output, bool simd);
// Converts normalized coefficients to packed integers. Leaves output[0] alone.
void quantize_ac(int width, int height, const float* transformed, int* output, int maximum, bool simd);
inline int round_even(float value) {
  const float low = std::floor(value);
  const int base = int(low);
  const float fraction = value - low;
  return base + int(fraction > 0.5f || (fraction == 0.5f && base % 2 != 0));
}
} // namespace neo_mv::dct_detail
