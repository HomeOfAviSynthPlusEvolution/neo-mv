#pragma once
#include <cstddef>
#include <cstdint>
namespace neo_mv::simd::mask_rows {
// One Q14 pass; null indices select contiguous vertical interpolation.
#define NEO_MASK_PASS(T)                                                                                               \
  void resize_pass(const std::int32_t* top, const std::int32_t* bottom, const std::int32_t* left,                      \
                   const std::int32_t* right, const std::int32_t* weights, int width, std::int32_t vertical,           \
                   T* output);
NEO_MASK_PASS(std::int32_t)
NEO_MASK_PASS(std::int16_t)
NEO_MASK_PASS(std::uint16_t)
NEO_MASK_PASS(std::uint8_t)
#undef NEO_MASK_PASS
#define NEO_MASK_VERTICAL(T) \
  void resize_vertical(const std::uint16_t* top, const std::uint16_t* bottom, int width, std::int32_t weight, T* output);
NEO_MASK_VERTICAL(std::int16_t)
NEO_MASK_VERTICAL(std::uint16_t)
NEO_MASK_VERTICAL(std::uint8_t)
#undef NEO_MASK_VERTICAL
void resize_float_vertical(const double* top, const double* bottom, int width, double coefficient, float* output);
void magnitude(const double* x, const double* y, std::size_t count, int pel, float f2, float exponent, float maximum,
               double* scores);
void sad(const float* samples, std::size_t count, float scale, float exponent, float maximum, float* scores);
#define NEO_MASK_MAX(T) void max_span(T* samples, std::size_t count, T value);
NEO_MASK_MAX(std::uint8_t)
NEO_MASK_MAX(std::uint16_t)
NEO_MASK_MAX(float)
#undef NEO_MASK_MAX
} // namespace neo_mv::simd::mask_rows
