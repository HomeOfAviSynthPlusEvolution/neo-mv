#pragma once
#include "core/interpolation/blur.hpp"
#include "highway/flow_sampling.hpp"

namespace neo_mv::simd {
// A single direction excludes the center and emits count samples in order.
// count and steps come from the scalar, integer-only direction contract.
void blur_samples(const RenderPhaseGeometry& geometry, int x, int y, int count, std::int64_t step_x,
                  std::int64_t step_y, const FlowSampleStorage* storage);
class BlurSamplingPlan : public neo_mv::BlurSamplingPlan {
public:
  using neo_mv::BlurSamplingPlan::BlurSamplingPlan;
  void preflight(const DenseFlowField& forward, const DenseFlowField& backward) const {
    validate_field(forward);
    validate_field(backward);
    for (int y = 0; y < height(); ++y)
      for (int x = 0; x < width(); ++x) {
        const auto i = std::size_t(y) * width() + x;
        for (const auto* field : {&forward, &backward}) {
          const auto d = direction(field->x[i], field->y[i]);
          if (d.count)
            blur_samples(geometry(), x, y, d.count, d.x, d.y, nullptr);
        }
      }
  }
  template <class T, class Average = ScalarBlurAverage>
  void sample(const DenseFlowField& forward, const DenseFlowField& backward, const SubpixelPhases<T>& source,
              span2d::Plane<T> output, int bits, Average average = {}) const {
    mask_detail::validate_storage<T>(bits);
    preflight(forward, backward);
    validate_storage(forward, backward, source, output);
    FlowSampleStorage storage{};
    for (int a = 0; a < geometry().pel * geometry().pel; ++a) {
      storage.planes[a] = reinterpret_cast<const std::byte*>(source.planes[a].row(0).data());
      storage.strides[a] = source.planes[a].stride_bytes();
    }
    storage.sample_bytes = sizeof(T);
    std::vector<T> samples;
    for (int y = 0; y < height(); ++y)
      for (int x = 0; x < width(); ++x) {
        const auto i = std::size_t(y) * width() + x;
        const auto f = direction(forward.x[i], forward.y[i]), b = direction(backward.x[i], backward.y[i]);
        samples.resize(std::size_t(1 + f.count + b.count));
        // The constructor admits all zero-displacement center coordinates.
        std::memcpy(samples.data(), source.planes[0].row(y + geometry().pad_y).data() + x + geometry().pad_x, sizeof(T));
        std::size_t offset = 1;
        for (const auto d : {f, b}) {
          if (d.count) {
            storage.output = reinterpret_cast<std::byte*>(samples.data() + offset);
            blur_samples(geometry(), x, y, d.count, d.x, d.y, &storage);
            offset += d.count;
          }
        }
        average(samples.data(), samples.size(), bits, output.row(y).data() + x);
      }
  }
};
} // namespace neo_mv::simd
