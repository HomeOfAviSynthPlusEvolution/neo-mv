#pragma once
#include <cstddef>
#include <cstdint>
namespace neo_mv::simd::mask_rows {
// Integer source rows are biased to [0,65535]. Coefficients lie in [0,16384].
#define NEO_MASK_RESIZE(T)                                                                                             \
  void resize(const std::int32_t* top, const std::int32_t* bottom, const std::int32_t* left,                           \
              const std::int32_t* right, const std::int32_t* coefficients, int width, std::int32_t vertical,           \
              bool horizontal_first, T* output);
NEO_MASK_RESIZE(std::uint8_t)
NEO_MASK_RESIZE(std::uint16_t)
NEO_MASK_RESIZE(std::int16_t)
#undef NEO_MASK_RESIZE
void resize(const double* top, const double* bottom, const std::int64_t* left, const std::int64_t* right,
            const double* rx, int width, double dx, double dy, double ry, float* output);
void magnitude(const double* x, const double* y, std::size_t count, int pel, float f2, float exponent, float maximum,
               double* scores);
void sad(const float* samples, std::size_t count, float scale, float exponent, float maximum, float* scores);
#define NEO_MASK_MAX(T) void max_span(T* samples, std::size_t count, T value);
NEO_MASK_MAX(std::uint8_t)
NEO_MASK_MAX(std::uint16_t)
NEO_MASK_MAX(float)
#undef NEO_MASK_MAX
} // namespace neo_mv::simd::mask_rows
