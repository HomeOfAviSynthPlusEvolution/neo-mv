#pragma once

#include "highway/grid_resampling.hpp"
#include "highway/scene.hpp"
#include "highway/flow_sampling.hpp"
#include "kernels/flow_scalar.hpp"

namespace neo_mv {
// Dense resampling and sampling coordinates use Highway; pixel reads preserve
// the exact representation of each selected sample.
template <class T>
struct HighwayFlowKernels : ScalarFlowKernels<T> {
  static constexpr auto scene_count = &simd::scene_count;
  using Sampling = simd::FlowSamplingPlan;
  static void sample(const Sampling& plan, const DenseFlowField& field, const SubpixelPhases<T>& source,
                     span2d::Plane<T> output) {
    plan.sample(field, source, output);
  }
  static void sample_preflighted(const Sampling& plan, const DenseFlowField& field, const SubpixelPhases<T>& source,
                                 span2d::Plane<T> output) {
    plan.template sample<T, true>(field, source, output);
  }
  using DenseFlow = DenseFlowPlan<simd::GridResamplingPlan>;
};
} // namespace neo_mv
