#pragma once
#include "core/interpolation/sampling.hpp"
#include "highway/flow_sampling.hpp"
#include "highway/interpolation_rows.hpp"

namespace neo_mv::simd {
class InterpolationSamplingPlan : public neo_mv::InterpolationSamplingPlan {
  void admit(const neo_mv::FlowSamplingPlan& image, const DenseFlowField& field, int time) const {
    const neo_mv::FlowSamplingPlan plan(image.geometry(), width(), height(), time);
    flow_sample(plan, field, nullptr, PhaseRounding::floor);
  }
  template <class T>
  void channel(const SubpixelPhases<T>& image, const neo_mv::FlowSamplingPlan& geometry, const DenseFlowField& field,
               int time, std::byte* output, std::size_t offset) const {
    const neo_mv::FlowSamplingPlan plan(geometry.geometry(), width(), height(), time);
    FlowSampleStorage storage{};
    storage.coordinates_validated = true;
    for (int a = 0; a < image.pel * image.pel; ++a) {
      storage.planes[a] = reinterpret_cast<const std::byte*>(image.planes[a].row(0).data());
      storage.strides[a] = image.planes[a].stride_bytes();
    }
    storage.output = output + offset;
    storage.output_pixel_stride = sizeof(InterpolationSamples<T>);
    storage.output_stride = std::ptrdiff_t(width()) * sizeof(InterpolationSamples<T>);
    storage.sample_bytes = sizeof(T);
    flow_sample(plan, field, &storage, PhaseRounding::floor);
  }

public:
  using neo_mv::InterpolationSamplingPlan::InterpolationSamplingPlan;
  void preflight(const DenseFlowField& B, const DenseFlowField& F, const DenseFlowField* BB = nullptr,
                 const DenseFlowField* FF = nullptr) const {
    validate_fields(B, F, BB, FF);
    admit(left_, F, time_);
    admit(right_, B, 256 - time_);
    if (BB) {
      admit(left_, *FF, time_);
      admit(right_, *BB, 256 - time_);
    }
    // Zero-displacement basic samples were admitted by construction.
  }
  // Frame-only entry: all planes passed preflight; output is independent.
  template <class T, class MaskAllocator>
  void render_preflighted(const SubpixelPhases<T>& left, const SubpixelPhases<T>& right, const DenseFlowField& B,
                          const DenseFlowField& F, const DenseFlowField* BB, const DenseFlowField* FF,
                          const std::vector<std::uint8_t, MaskAllocator>& mF,
                          const std::vector<std::uint8_t, MaskAllocator>& mB, span2d::Plane<T> out) const {
    interpolation_rows::render(
        interpolation_rows::SampledPlane<T>{
            left_.geometry(), right_.geometry(), {left, right}, {&F, &B, FF, BB}, {mF.data(), mB.data()}, time_, bits_},
        out);
  }
  template <class T, bool Preflighted = false>
  std::vector<InterpolationSamples<T>>
  sample(const SubpixelPhases<T>& left, const SubpixelPhases<T>& right, const DenseFlowField& B,
         const DenseFlowField& F, const DenseFlowField* BB = nullptr, const DenseFlowField* FF = nullptr) const {
    mask_detail::validate_storage<T>(bits_);
    validate_image(left, left_.geometry());
    validate_image(right, right_.geometry());
    if constexpr (!Preflighted)
      preflight(B, F, BB, FF);
    using Samples = InterpolationSamples<T>;
    const auto count = std::uint64_t(width()) * height();
    if (count > std::vector<Samples>().max_size() || count > std::uint64_t(PTRDIFF_MAX) / sizeof(Samples))
      throw std::overflow_error("interpolation sample storage size is unrepresentable");
    std::vector<Samples> out(static_cast<std::size_t>(count));
    auto* bytes = reinterpret_cast<std::byte*>(out.data());
    channel(left, left_, F, time_, bytes, offsetof(Samples, A));
    channel(right, right_, B, 256 - time_, bytes, offsetof(Samples, C));
    if (BB) {
      channel(left, left_, *FF, time_, bytes, offsetof(Samples, E));
      channel(right, right_, *BB, 256 - time_, bytes, offsetof(Samples, K));
    } else {
      channel(left, left_, F, 0, bytes, offsetof(Samples, A0));
      channel(right, right_, B, 0, bytes, offsetof(Samples, C0));
    }
    // Storage is private until every consumed sample passes numeric admission.
    for (const auto& s : out)
      for (const T value : {s.A, s.C, BB ? s.E : s.A0, BB ? s.K : s.C0})
        interpolation_detail::sample(value, bits_);
    return out;
  }
};
} // namespace neo_mv::simd
