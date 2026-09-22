#pragma once
#include "core/interpolation/blur.hpp"
#include "highway/flow_sampling.hpp"

namespace neo_mv {
struct HighwayBlurAverage;
}
namespace neo_mv::simd {
class BlurSamplingPlan;
void blur_preflight(const BlurSamplingPlan&, const DenseFlowField&, const DenseFlowField&);
#define NEO_BLUR_PLANE(T)                                                                                              \
  void blur_plane(const BlurSamplingPlan&, const DenseFlowField&, const DenseFlowField&, const SubpixelPhases<T>&,     \
                  span2d::Plane<T>, int);
NEO_BLUR_PLANE(std::uint8_t)
NEO_BLUR_PLANE(std::uint16_t)
NEO_BLUR_PLANE(float)
#undef NEO_BLUR_PLANE
// A single direction excludes the center and emits count samples in order.
// count and steps come from the scalar, integer-only direction contract.
void blur_samples(const RenderPhaseGeometry& geometry, int x, int y, int count, std::int64_t step_x,
                  std::int64_t step_y, const FlowSampleStorage* storage);
class BlurSamplingPlan : public neo_mv::BlurSamplingPlan {
public:
  using neo_mv::BlurSamplingPlan::BlurSamplingPlan;
  using neo_mv::BlurSamplingPlan::direction;
  void preflight(const DenseFlowField& forward, const DenseFlowField& backward) const {
    validate_field(forward);
    validate_field(backward);
    blur_preflight(*this, forward, backward);
  }
  void preflight_generated(const DenseFlowField& forward, const DenseFlowField& backward) const {
    validate_field(forward);
    validate_field(backward);
    const flow_coordinates::CommonDomain domain(geometry());
    const auto covered = [&](const DenseFlowField& field) {
      return field.generated_bounds &&
             domain.covers_bounds(width(), height(), *field.generated_bounds, time_coefficient(), 0) &&
             domain.contains_scaled(0, 0, 0, 0) && domain.contains_scaled(width() - 1, height() - 1, 0, 0);
    };
    if (covered(forward) && covered(backward))
      return;
    preflight(forward, backward);
  }
  template <class T, class Average = ScalarBlurAverage, bool Preflighted = false, bool StorageValidated = false>
  void sample(const DenseFlowField& forward, const DenseFlowField& backward, const SubpixelPhases<T>& source,
              span2d::Plane<T> output, int bits, Average average = {}) const {
    mask_detail::validate_storage<T>(bits);
    if constexpr (!Preflighted)
      preflight(forward, backward);
    if constexpr (!StorageValidated)
      validate_storage(forward, backward, source, output);
    if constexpr (std::is_same_v<Average, ScalarBlurAverage> || std::is_same_v<Average, HighwayBlurAverage>) {
      blur_plane(*this, forward, backward, source, output, bits);
      return;
    }

    FlowSampleStorage storage{};
    storage.coordinates_validated = true;
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
        std::memcpy(samples.data(), source.planes[0].row(y + geometry().pad_y).data() + x + geometry().pad_x,
                    sizeof(T));
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
