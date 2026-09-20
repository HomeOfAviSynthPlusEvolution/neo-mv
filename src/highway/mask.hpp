#pragma once

#include "highway/grid_resampling.hpp"
#include "highway/mask_scores.hpp"

namespace neo_mv {
template <class T>
struct HighwayMaskKernels {
  using GridResampling = simd::GridResamplingPlan;
  using VectorLength = simd::VectorLengthMaskPlan<T>;
  using SAD = simd::SADMaskPlan<T>;
  using Occlusion = simd::OcclusionMaskPlan<T>;
};
} // namespace neo_mv
