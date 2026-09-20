#pragma once
#include <cstddef>
#include <cstdint>
namespace neo_mv::simd::mask_rows {
// For integer rows, dx*dy <= 2^32 and input doubles are exact biased integers.
#define NEO_MASK_RESIZE(T)                                                                                             \
  void resize(const double* top, const double* bottom, const std::int64_t* left, const std::int64_t* right,            \
              const double* rx, int width, double dx, double dy, double ry, T* output);
NEO_MASK_RESIZE(std::uint8_t)
NEO_MASK_RESIZE(std::uint16_t)
NEO_MASK_RESIZE(std::int16_t)
NEO_MASK_RESIZE(float)
#undef NEO_MASK_RESIZE
void magnitude(const double* x, const double* y, std::size_t count, int pel, float f2, float exponent, float maximum,
               double* scores);
void sad(const float* samples, std::size_t count, float scale, float exponent, float maximum, float* scores);
#define NEO_MASK_MAX(T) void max_span(T* samples, std::size_t count, T value);
NEO_MASK_MAX(std::uint8_t)
NEO_MASK_MAX(std::uint16_t)
NEO_MASK_MAX(float)
#undef NEO_MASK_MAX
} // namespace neo_mv::simd::mask_rows
