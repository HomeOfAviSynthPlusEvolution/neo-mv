#pragma once
#include "core/motion/metric_context.hpp"
#include "core/motion/dct.hpp"
#include <memory>

namespace neo_mv {
// One synchronous layer/request owns this workspace. The evaluator resets
// source readiness for each block; no coefficients survive into another block.
struct MetricScratch { std::unique_ptr<DctWorkspace> dct; };
template <class T, class Kernels>
class PreparedMetricEvaluator {
  static_assert(std::is_integral_v<T>);
  decltype(Kernels::prepare_statistics()) statistics_ = Kernels::prepare_statistics();
  MetricLayerPlan plan_;
  const SamplingGeometry& geometry_;
  BlockRegion block_;
  const SamplingFrames<T>& frames_;
  MetricScratch& scratch_;
  int bits_;
  bool source_ready_ = false;
  span2d::Plane<const T> source_;
  std::int64_t source_sum_ = 0;

  std::int64_t transformed(span2d::Plane<const T> reference) {
    if (plan_.descriptor.transform == MetricTransform::satd)
      return Kernels::pixel_metric(source_, reference, BlockMetric::satd);
    if (!scratch_.dct)
      scratch_.dct = std::make_unique<DctWorkspace>(block_.width, block_.height, bits_,
                                                    !std::is_same_v<Kernels, ScalarKernels<T>>);
    if (!source_ready_) {
      scratch_.dct->set_source(source_);
      source_ready_ = true;
    }
    return scratch_.dct->scale_distance(scratch_.dct->compare_parts(reference), plan_.descriptor.dc_weight);
  }
public:
  PreparedMetricEvaluator(MetricLayerPlan plan, const SamplingGeometry& geometry, BlockRegion block,
                           const SamplingFrames<T>& frames, MetricScratch& scratch, int bits)
      : plan_(plan), geometry_(geometry), block_(block), frames_(frames), scratch_(scratch), bits_(bits),
        source_(source_block(geometry, block, frames, 0)) {
    if (plan.descriptor.mix == MetricMix::local) source_sum_ = Kernels::block_luma_sum(source_);
  }
  // Deliberately no bounded/analyse methods: SAD is not a lower bound on a
  // mixed error, so the SAD-specific early rejection would change vectors.
  BlockError operator()(MotionVector vector) {
    const auto reference = reference_block(geometry_, block_, frames_, 0, vector);
    std::int64_t luma;
    if (plan_.descriptor.mix == MetricMix::none) {
      luma = transformed(reference);
    } else if (plan_.descriptor.mix == MetricMix::local) {
      const auto measured = statistics_(source_, reference);
      luma = measured.sad;
      if (local_metric_active(source_sum_, measured.reference_sum, plan_.config.threshold))
        luma = mix_metric(luma, transformed(reference), plan_.weight, plan_.unit);
    } else {
      const auto sad = Kernels::pixel_metric(source_, reference, BlockMetric::sad);
      luma = mix_metric(sad, transformed(reference), plan_.weight, plan_.unit);
    }
    std::int64_t chroma = 0;
    for (int k = 1; k < (geometry_.chroma ? 3 : 1); ++k)
      chroma = metric_detail::accumulate(chroma, Kernels::pixel_metric(source_block(geometry_, block_, frames_, k),
          reference_block(geometry_, block_, frames_, k, vector), BlockMetric::sad));
    return {luma, chroma, metric_detail::accumulate(luma, chroma)};
  }
};
template <class T, class Kernels, class PreparedFrames, class Callback>
auto with_motion_evaluator(MetricLayerPlan plan, const SamplingGeometry& geometry, BlockRegion block,
                            const SamplingFrames<T>& frames, const PreparedFrames& prepared,
                            MetricScratch& scratch, int bits, Callback&& callback) {
  if (plan.descriptor.mix == MetricMix::none && plan.descriptor.transform != MetricTransform::dct) {
    auto evaluate = Kernels::prepare_block_error(geometry, block, prepared,
        plan.descriptor.transform == MetricTransform::satd ? BlockMetric::satd : BlockMetric::sad);
    return callback(evaluate);
  }
  if constexpr (std::is_integral_v<T>) {
    PreparedMetricEvaluator<T, Kernels> evaluate(plan, geometry, block, frames, scratch, bits);
    return callback(evaluate);
  } else {
    throw std::invalid_argument("DCT and mixed metrics require integer samples");
  }
}
} // namespace neo_mv
