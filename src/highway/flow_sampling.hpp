#pragma once
#include "core/flow/sampling.hpp"

namespace neo_mv::simd {
// Byte views preserve all sample representations and permit narrow samples
// without wider gathers reading outside their logical storage.
enum class PhaseRounding { nearest, floor };
struct FlowSampleStorage {
  std::array<const std::byte*, 16> planes{};
  std::array<std::ptrdiff_t, 16> strides{};
  std::byte* output;
  std::ptrdiff_t output_stride;
  std::size_t sample_bytes, output_pixel_stride;
  bool coordinates_validated = false; // Set only after a full preflight pass.
};
void flow_sample(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                 PhaseRounding rounding = PhaseRounding::nearest);

class FlowSamplingPlan : public neo_mv::FlowSamplingPlan {
public:
  using neo_mv::FlowSamplingPlan::FlowSamplingPlan;
  void preflight(const DenseFlowField& field) const {
    validate_dense(field);
    flow_sample(*this, field, nullptr);
  }
  template <class T, bool Preflighted = false>
  void sample(const DenseFlowField& field, const SubpixelPhases<T>& source, span2d::Plane<T> output) const {
    if constexpr (!Preflighted)
      preflight(field);
    validate_storage(field, source, output);
    FlowSampleStorage storage{};
    storage.coordinates_validated = true;
    for (int a = 0; a < geometry().pel * geometry().pel; ++a) {
      storage.planes[a] = reinterpret_cast<const std::byte*>(source.planes[a].row(0).data());
      storage.strides[a] = source.planes[a].stride_bytes();
    }
    storage.output = reinterpret_cast<std::byte*>(output.row(0).data());
    storage.output_stride = output.stride_bytes();
    storage.sample_bytes = storage.output_pixel_stride = sizeof(T);
    flow_sample(*this, field, &storage);
  }
};
} // namespace neo_mv::simd
