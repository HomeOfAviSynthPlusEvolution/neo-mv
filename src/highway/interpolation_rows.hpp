#pragma once
#include "core/interpolation/sampling.hpp"
#include <cstddef>
#include <cstdint>

namespace neo_mv::simd::interpolation_rows {
// Internal plane render: fields/phase storage and all coordinates were admitted
// by the frame caller, masks have width*height entries, output is disjoint.
template <class T>
struct SampledPlane {
  RenderPhaseGeometry left, right;
  SubpixelPhases<T> images[2];
  const DenseFlowField* fields[4]; // F, B, FF, BB
  const std::uint8_t* masks[2];    // mF, mB
  int time, bits;
};
#define NEO_SAMPLED_TEMPORAL(T) void render(const SampledPlane<T>&, span2d::Plane<T>);
NEO_SAMPLED_TEMPORAL(std::uint8_t)
NEO_SAMPLED_TEMPORAL(std::uint16_t)
NEO_SAMPLED_TEMPORAL(float)
#undef NEO_SAMPLED_TEMPORAL

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
