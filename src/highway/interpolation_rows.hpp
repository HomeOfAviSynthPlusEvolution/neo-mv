#pragma once
#include <cstddef>
#include <cstdint>

namespace neo_mv::simd::interpolation_rows {
// Inputs are validated render samples; integer samples are widened to uint32.
// X/Y denote A0/C0 in basic mode and E/K in extra mode. Masks are in [0,255].
#define NEO_TEMPORAL_ROWS(T)                                                                                           \
  void compose(const T* a, const T* c, const T* x, const T* y, const T* mf, const T* mb, int width, bool extra,        \
               int time, T* output);                                                                                   \
  void blend(const T* a, const T* b, int width, int time, T* output);
NEO_TEMPORAL_ROWS(std::uint32_t)
NEO_TEMPORAL_ROWS(float)
#undef NEO_TEMPORAL_ROWS
std::uint32_t sum(const std::uint8_t* samples, std::size_t count);
std::uint32_t sum(const std::uint16_t* samples, std::size_t count);
} // namespace neo_mv::simd::interpolation_rows
