#pragma once

#include "core/motion/block_sampling.hpp"
#include "core/super/border_extension.hpp"
#include "core/super/pyramid_reduction.hpp"
#include "core/super/subpixel.hpp"

namespace neo_mv {
// Compile-time pixel operations; control flow and search stay in the core.
template <class T> struct ScalarKernels {
  static constexpr auto extend_border = &neo_mv::extend_border<T>;
  static constexpr auto reduce_pyramid = &neo_mv::reduce_pyramid<T>;
  static constexpr auto interpolate_subpixels = &neo_mv::interpolate_subpixels<T>;
  static constexpr auto extract_external_subpixels = &neo_mv::extract_external_subpixels<T>;
  static BlockError block_error(const SamplingGeometry& geometry, BlockRegion block, const SamplingFrames<T>& frames,
                               MotionVector vector, BlockMetric metric) {
    return neo_mv::block_error(geometry, block, frames, vector, metric);
  }
};
} // namespace neo_mv
