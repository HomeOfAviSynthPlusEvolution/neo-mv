#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
namespace neo_mv {
struct MotionTriple;
struct SpatialPredictors;
struct MotionVector;
struct CandidateDomain;
struct AnalyseControls;
struct SearchResult;
} // namespace neo_mv
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
// Layer-wide phase tables and block-local source/coordinates. The caller has
// admitted the complete candidate domain before entering these kernels.
template <class T> struct MetricReferenceFrames {
  std::array<std::array<const T*, 16>, 3> references{};
  std::array<std::array<std::ptrdiff_t, 16>, 3> strides{};
};
template <class T> struct MotionMetricRequest {
  std::array<MetricRequest<T>, 3> planes{};
  std::array<int, 3> x{}, y{};
  int pel = 1;
};
template <class T>
using BoundedMotionMetricFunction = bool (*)(const MotionMetricRequest<T>&, const MetricReferenceFrames<T>&,
                                            int, int, std::int64_t, std::int64_t*);

template <class T>
using AnalyseBlockFunction = SearchResult (*)(const MotionMetricRequest<T>&, const MetricReferenceFrames<T>&,
    MotionTriple, const SpatialPredictors&, MotionVector, CandidateDomain, int, std::int64_t, std::int64_t,
    AnalyseControls);
AnalyseBlockFunction<std::uint8_t> analyse_block_420_function(std::uint8_t*);
AnalyseBlockFunction<std::uint16_t> analyse_block_420_function(std::uint16_t*);

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
  BoundedMetricBatchFunction<T> metric_batch_420_small_bounded_function(T *);                                         \
  BoundedMotionMetricFunction<T> motion_metric_420_bounded_function(T *, int width);
NEO_DECLARE(std::uint8_t)
NEO_DECLARE(std::uint16_t)
NEO_DECLARE(float)
#undef NEO_DECLARE
} // namespace neo_mv::simd::detail
