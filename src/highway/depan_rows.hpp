#pragma once
#include "core/depan/analysis.hpp"
#include <cstddef>
#include <cstdint>

namespace neo_mv::simd::depan_rows {
// Only native-FMA targets fuse floating residual, accumulation and fit-update operations.
// Other targets retain separate rounding. Inputs and products remain finite.
bool native_fma();
// Accumulate in observation order; arrays contain one residual per observation.
// Inputs have been validated by fit_update before dispatch.
depan::FitSums accumulate(const depan::Observations& observations, const std::vector<float>& weights, const float* ex,
                          const float* ey, bool zoom, bool rotation, const depan::FitGeometry* geometry = nullptr);
void adjust(const float* values, const float* scales, const float* gradients, std::size_t count, float* output);
// Planar row arrays: each tap has count active elements and tap_stride spacing.
// A zero tap_stride selects tightly packed arrays (spacing count); otherwise
// tap_stride must be at least count, with all active elements readable.
// Weights/sample products and all intermediate sums fit signed int64.
void weighted(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
              bool round, std::int64_t maximum, std::int64_t* out, std::size_t tap_stride = 0);
void residuals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count,
               depan::Transform map, float* ex, float* ey);
} // namespace neo_mv::simd::depan_rows
