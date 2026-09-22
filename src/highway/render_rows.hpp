#pragma once
#include "core/render/overlap.hpp"
#include <cstdint>
namespace neo_mv::simd::detail {
// Internal borrowed blocks: full rectangles have admitted geometry, output is
// disjoint, and coefficient pointers refer to immutable plan windows.
template <class T>
struct SampledRenderBlock {
  const T* data;
  std::ptrdiff_t stride;
  const std::uint16_t* coefficients;
};
#define NEO_SAMPLED(T)                                                                                                 \
  void compose_sampled(const BlockCompositionGeometry& geometry, const SampledRenderBlock<T>* blocks,                  \
                       span2d::Plane<T> output, std::int64_t maximum);
NEO_SAMPLED(std::uint8_t)
NEO_SAMPLED(std::uint16_t)
NEO_SAMPLED(float)
#undef NEO_SAMPLED

#define NEO_RENDER_ROWS(T, A)                                                                                          \
  void weighted(const T* centre, const T* const* refs, const int* weights, int centre_weight, int references, T* out,  \
                int count);                                                                                            \
  void overlap_add(const T* src, const std::uint16_t* coefficients, A* sum, int count);                                \
  void overlap_finish(const A* sum, T* out, int count, std::int64_t maximum);                                          \
  void change_limit(const T* q, const T* centre, T* out, int count, bool active, std::int64_t maximum,                 \
                    std::int64_t integer_limit, float float_limit);
NEO_RENDER_ROWS(std::uint8_t, std::int32_t)
NEO_RENDER_ROWS(std::uint16_t, std::int32_t)
NEO_RENDER_ROWS(float, float)
#undef NEO_RENDER_ROWS
} // namespace neo_mv::simd::detail
