#pragma once

#include "core/mask/grid_resampling.hpp"
#include "core/mask/scores.hpp"

namespace neo_mv {
template <class T>
struct ScalarMaskKernels {
  using GridResampling = GridResamplingPlan;
  using VectorLength = VectorLengthMaskPlan<T>;
  using SAD = SADMaskPlan<T>;
  using Occlusion = OcclusionMaskPlan<T>;
};
} // namespace neo_mv
