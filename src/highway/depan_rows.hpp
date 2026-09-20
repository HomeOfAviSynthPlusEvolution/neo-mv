#pragma once
#include "core/depan/transform.hpp"
#include <cstddef>
#include <cstdint>

namespace neo_mv::simd::depan_rows {
// Only native-FMA targets fuse floating residual and fit-update operations.
// Other targets retain separate rounding. Inputs and products remain finite.
bool native_fma();
void adjust(const float* values, const float* scales, const float* gradients, std::size_t count, float* output);
// Planar row arrays: every tap occupies count contiguous elements.
// Weights/sample products and all intermediate sums fit signed int64.
void weighted(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
              bool round, std::int64_t maximum, std::int64_t* out);
void residuals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count,
               depan::Transform map, float* ex, float* ey);
} // namespace neo_mv::simd::depan_rows
