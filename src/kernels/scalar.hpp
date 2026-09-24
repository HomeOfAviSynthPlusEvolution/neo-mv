#pragma once

#include "core/motion/block_sampling.hpp"
#include "core/motion/prediction.hpp"
#include "core/super/border_extension.hpp"
#include "core/super/pyramid_reduction.hpp"
#include "core/super/subpixel.hpp"

namespace neo_mv {
// Compile-time pixel operations; control flow and search stay in the core.
template <class T>
struct ScalarKernels {
  static constexpr auto interpolate_predictions = &neo_mv::interpolate_predictions;
  static const SamplingFrames<T>& prepare_frames(const SamplingGeometry&, const SamplingFrames<T>& frames) {
    return frames;
  }
  static auto prepare_block_error(const SamplingGeometry& geometry, BlockRegion block, const SamplingFrames<T>& frames,
                                  BlockMetric metric) {
    return [&geometry, block, &frames, metric](MotionVector vector) {
      return neo_mv::block_error<T, true>(geometry, block, frames, vector, metric);
    };
  }
  static void validate_samples(const T* samples, int count, std::int64_t maximum) {
    for (int i = 0; i < count; ++i)
      subpixel_detail::valid_sample(samples[i], maximum);
  }
  static constexpr auto extend_border = &neo_mv::extend_border<T>;
  static constexpr auto reduce_pyramid = &neo_mv::reduce_pyramid<T>;
  static constexpr auto interpolate_subpixels = &neo_mv::interpolate_subpixels<T>;
  // SuperPlan admits geometry; SuperPyramid owns disjoint output buffers and
  // admits source samples once. Each producer still checks new arithmetic.
  static constexpr auto extend_border_validated = &neo_mv::extend_border<T, true>;
  static constexpr auto reduce_pyramid_validated = &neo_mv::reduce_pyramid<T, true>;
  static constexpr auto interpolate_subpixels_validated = &neo_mv::interpolate_subpixels<T, true>;
  static constexpr auto extract_external_subpixels = &neo_mv::extract_external_subpixels<T>;
  static BlockError block_error(const SamplingGeometry& geometry, BlockRegion block, const SamplingFrames<T>& frames,
                                MotionVector vector, BlockMetric metric) {
    return neo_mv::block_error(geometry, block, frames, vector, metric);
  }
  // Geometry/domain and frame storage have already been admitted by the plan.
  static BlockError block_error_validated(const SamplingGeometry& geometry, BlockRegion block,
                                          const SamplingFrames<T>& frames, MotionVector vector, BlockMetric metric) {
    return neo_mv::block_error<T, true>(geometry, block, frames, vector, metric);
  }
};
} // namespace neo_mv
