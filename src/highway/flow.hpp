#pragma once

#include "highway/grid_resampling.hpp"
#include "kernels/flow_scalar.hpp"

namespace neo_mv {
// Dense signed-component resampling uses the Highway row kernels. Reference
// sampling remains a scalar representation-preserving copy in this backend.
template <class T>
struct HighwayFlowKernels : ScalarFlowKernels<T> {
  using DenseFlow = DenseFlowPlan<simd::GridResamplingPlan>;
};
} // namespace neo_mv
