#pragma once

#include "core/flow/sampling.hpp"
#include "core/motion/scene_classification.hpp"

namespace neo_mv {
template <class T>
struct ScalarFlowKernels {
  static constexpr auto scene_count = &scalar_scene_count;
  static constexpr auto scene_count_validated = &scalar_scene_count_validated;
  using Sampling = FlowSamplingPlan;
  using DenseFlow = DenseFlowPlan<>;
  static void sample(const FlowSamplingPlan& plan, const DenseFlowField& field, const SubpixelPhases<T>& source,
                     span2d::Plane<T> output) {
    plan.sample(field, source, output);
  }
  static void sample_preflighted(const FlowSamplingPlan& plan, const DenseFlowField& field,
                                 const SubpixelPhases<T>& source, span2d::Plane<T> output) {
    plan.template sample<T, true, true>(field, source, output);
  }
};
} // namespace neo_mv
