#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>
namespace neo_mv::dct_detail {
// Validated unsigned16 integer pixels and an AC frequency. The estimate only
// seeds the candidate; integer enclosures determine the returned rounding.
int refine_ac(const std::uint16_t* samples, int width, int height, int u, int v, double estimate);
// Internal entry points also exercised directly by the numerical tests.
bool fixed_round(const std::uint16_t* samples, int width, int height, int u, int v, int candidate);
int interval_round(const std::uint16_t* samples, int width, int height, int u, int v);
void transform_block(int width, int height, const double* input, double* rows, double* output, bool simd);
// AC values are bounded by 2*sqrt(2)*maximum plus the FFT enclosure, so
// truncation fits int. Correcting negative fractions implements floor in
// every FP rounding mode without a baseline-instruction-set libm call.
inline int quantize_one(const std::uint16_t* samples, int width, int height, int index, double value, double error,
                        int maximum) {
  const int truncated = int(value);
  const int lower = truncated - (value < double(truncated));
  const double low = double(lower), fraction = value - low;
  int q = lower + (fraction > .5 || (fraction == .5 && lower % 2 != 0));
  // The factor two also covers rounding of the boundary-distance subtraction.
  if (std::abs(value - (low + .5)) <= 2 * error)
    q = refine_ac(samples, width, height, index % width, index / width, value);
  return std::clamp(q + (maximum + 1) / 2, 0, maximum);
}
void quantize_ac(const std::uint16_t* samples, int width, int height, const double* transformed, int* output,
                 int maximum, double error, bool simd);
} // namespace neo_mv::dct_detail
