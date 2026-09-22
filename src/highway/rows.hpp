#pragma once
#include <cstddef>
#include <cstdint>
namespace neo_mv::simd::detail {
// Uses the same dispatch table selection as the row kernels.
const char *target_name();
enum class Formula { average, four_average, reduce4, reduce6, sharp4h, sharp4v, sharp6 };
template <class T> struct MetricRequest {
  const T *source;
  std::ptrdiff_t source_stride;
  const T *reference;
  std::ptrdiff_t reference_stride;
  int width, height;
  bool satd;
};
template <class T>
using MetricBatchFunction = void (*)(const MetricRequest<T> *, int, std::int64_t *);
template <class T>
using BoundedMetricBatchFunction = bool (*)(const MetricRequest<T> *, std::int64_t, std::int64_t *);
// step=2 requires 2*count accessible samples at each tap for deinterleaving.
#define NEO_DECLARE(T)                                                                                                 \
  void extract(const T *, T *, int, int, int);                                                                         \
  void scan(const T *, int, std::int64_t);                                                                             \
  void copy(const T *, T *, int);                                                                                      \
  void fill(T, T *, int);                                                                                              \
  void formula(const T *const *, int step, T *, int count, Formula, std::int64_t maximum);                             \
  std::int64_t metric(const T *, std::ptrdiff_t, const T *, std::ptrdiff_t, int, int, bool);                            \
  void metric_batch(const MetricRequest<T> *, int, std::int64_t *);                                                     \
  MetricBatchFunction<T> metric_batch_function(T *);                                                                    \
  MetricBatchFunction<T> metric_batch_420_function(T *);                                                                \
  MetricBatchFunction<T> metric_batch_420_small_function(T *);                                                          \
  BoundedMetricBatchFunction<T> metric_batch_420_bounded_function(T *);                                                  \
  BoundedMetricBatchFunction<T> metric_batch_420_small_bounded_function(T *);
NEO_DECLARE(std::uint8_t)
NEO_DECLARE(std::uint16_t)
NEO_DECLARE(float)
#undef NEO_DECLARE
} // namespace neo_mv::simd::detail
