#include "highway/flow_sampling.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/flow_sampling.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
#include "highway/flow_sampling-inl.hpp"
void FlowSample8(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                 PhaseRounding rounding) {
  FlowSample<1>(plan, field, storage, rounding);
}
void FlowSample16(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                  PhaseRounding rounding) {
  FlowSample<2>(plan, field, storage, rounding);
}
void FlowSample32(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                  PhaseRounding rounding) {
  FlowSample<4>(plan, field, storage, rounding);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd {
HWY_EXPORT(FlowSample8);
HWY_EXPORT(FlowSample16);
HWY_EXPORT(FlowSample32);
void flow_sample(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                 PhaseRounding rounding) {
  if (!storage || storage->sample_bytes == 1)
    HWY_DYNAMIC_DISPATCH(FlowSample8)(plan, field, storage, rounding);
  else if (storage->sample_bytes == 2)
    HWY_DYNAMIC_DISPATCH(FlowSample16)(plan, field, storage, rounding);
  else
    HWY_DYNAMIC_DISPATCH(FlowSample32)(plan, field, storage, rounding);
}
} // namespace neo_mv::simd
#endif
