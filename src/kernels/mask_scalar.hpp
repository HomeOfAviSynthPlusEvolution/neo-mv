#pragma once

#include "core/mask/grid_resampling.hpp"
#include "core/mask/scores.hpp"
#include "core/motion/scene_classification.hpp"

namespace neo_mv {
template <class T>
struct ScalarMaskKernels {
  static constexpr auto scene_count = &scalar_scene_count;
  static constexpr auto scene_count_validated = &scalar_scene_count_validated;
  using GridResampling = GridResamplingPlan;
  using VectorLength = VectorLengthMaskPlan<T>;
  using SAD = SADMaskPlan<T>;
  using Occlusion = OcclusionMaskPlan<T>;
};
} // namespace neo_mv
