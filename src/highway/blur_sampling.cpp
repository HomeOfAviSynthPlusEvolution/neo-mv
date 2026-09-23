#include "highway/blur_sampling.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/blur_sampling.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <std::size_t Bytes, class Consume>
void BlurVisit(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t step_x, std::int64_t step_y,
               const FlowSampleStorage* storage, Consume&& consume) {
#if HWY_TARGET == HWY_AVX2
  if (count == 1 && storage && storage->coordinates_validated) {
    // A single admitted sample does not need vector setup and spill arrays.
    const auto dx = flow_coordinates::floor_shift(step_x, 8);
    const auto dy = flow_coordinates::floor_shift(step_y, 8);
    const int shift = flow_coordinates::shift(g.pel);
    const auto qx = flow_coordinates::floor_shift(dx, shift);
    const auto qy = flow_coordinates::floor_shift(dy, shift);
    const auto phase = std::size_t((dy - qy * g.pel) * g.pel + dx - qx * g.pel);
    const auto offset = (std::int64_t(g.pad_y) + y + qy) * storage->strides[phase] +
                        (std::int64_t(g.pad_x) + x + qx) * Bytes;
    consume(0, storage->planes[phase] + offset);
    return;
  }
#endif
  const hn::ScalableTag<std::int64_t> d;
  const int lanes = static_cast<int>(hn::Lanes(d));
  const hn::Rebind<std::int32_t, decltype(d)> d32;
  HWY_ALIGN std::int64_t widths[16]{}, heights[16]{};
  HWY_ALIGN std::int64_t columns[hn::MaxLanes(d)], rows[hn::MaxLanes(d)], phases[hn::MaxLanes(d)];
  for (int a = 0; a < g.pel * g.pel; ++a) {
    widths[a] = g.phases[a].width;
    heights[a] = g.phases[a].height;
  }
  const int shift = g.pel == 4 ? 2 : g.pel == 2 ? 1 : 0;
  const auto fraction = hn::Set(d, g.pel - 1), zero = hn::Zero(d);
  for (int first = 1; first <= count;) {
    const int used = std::min(lanes, count - first + 1);
    // direction() bounds count * abs(step) by 32768 * 256.
    // Clamp inactive lanes as well, so every int32 product stays in range.
    const auto sample = hn::Min(hn::Iota(d32, first), hn::Set(d32, count));
    const auto dx =
        hn::PromoteTo(d, hn::ShiftRight<8>(hn::Mul(sample, hn::Set(d32, static_cast<std::int32_t>(step_x)))));
    const auto dy =
        hn::PromoteTo(d, hn::ShiftRight<8>(hn::Mul(sample, hn::Set(d32, static_cast<std::int32_t>(step_y)))));
    const auto phase = hn::Add(hn::And(dx, fraction), hn::ShiftLeftSame(hn::And(dy, fraction), shift));
    const auto sx = hn::Add(hn::Set(d, std::int64_t(g.pad_x) + x), hn::ShiftRightSame(dx, shift));
    const auto sy = hn::Add(hn::Set(d, std::int64_t(g.pad_y) + y), hn::ShiftRightSame(dy, shift));
    if (!storage || !storage->coordinates_validated) {
      const auto valid_x = hn::And(hn::Ge(sx, zero), hn::Lt(sx, hn::GatherIndex(d, widths, phase)));
      const auto valid_y = hn::And(hn::Ge(sy, zero), hn::Lt(sy, hn::GatherIndex(d, heights, phase)));
      if (!hn::AllTrue(d, hn::Or(hn::Not(hn::FirstN(d, used)), hn::And(valid_x, valid_y))))
        throw std::invalid_argument("blur trajectory exceeds logical phase domain");
    }
    if (storage) {
      hn::Store(sx, d, columns);
      hn::Store(sy, d, rows);
      hn::Store(phase, d, phases);
      for (int i = 0; i < used; ++i) {
        const auto a = static_cast<std::size_t>(phases[i]);
        const auto* input = storage->planes[a] + rows[i] * storage->strides[a] + columns[i] * Bytes;
        consume(first - 1 + i, input);
      }
    }
    first += used;
  }
}
template <std::size_t Bytes>
void BlurSamples(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t sx, std::int64_t sy,
                 const FlowSampleStorage* storage) {
  BlurVisit<Bytes>(g, x, y, count, sx, sy, storage, [&](int i, const std::byte* input) {
    std::memcpy(storage->output + std::size_t(i) * Bytes, input, Bytes);
  });
}
void BlurPreflight(const BlurSamplingPlan& plan, const DenseFlowField& f, const DenseFlowField& b) {
  const flow_coordinates::CommonDomain domain(plan.geometry());
  for (int y = 0; y < plan.height(); ++y)
    for (int x = 0; x < plan.width(); ++x) {
      const auto i = std::size_t(y) * plan.width() + x;
      for (const auto* field : {&f, &b}) {
        if (domain.contains_scaled(x, y, 0, 0) &&
            domain.contains_scaled(x, y, std::int64_t(field->x[i]) * plan.time_coefficient(),
                                   std::int64_t(field->y[i]) * plan.time_coefficient()))
          continue;
        const auto d = plan.direction(field->x[i], field->y[i]);
        if (d.count &&
            !(domain.contains_scaled(x, y, 0, 0) && domain.contains_scaled(x, y, d.count * d.x, d.count * d.y)))
          BlurSamples<1>(plan.geometry(), x, y, d.count, d.x, d.y, nullptr);
      }
    }
}
template <class T>
void BlurPlane(const BlurSamplingPlan& plan, const DenseFlowField& forward, const DenseFlowField& backward,
               const SubpixelPhases<T>& source, span2d::Plane<T> output, int bits) {
  const auto& g = plan.geometry();
  FlowSampleStorage storage{};
  storage.coordinates_validated = true;
  for (int a = 0; a < g.pel * g.pel; ++a) {
    storage.planes[a] = reinterpret_cast<const std::byte*>(source.planes[a].row(0).data());
    storage.strides[a] = source.planes[a].stride_bytes();
  }
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  // Dense integer fields often repeat between adjacent pixels. Only the
  // trajectory parameters depend on the vector; sample positions remain local.
  struct DirectionCache {
    int x = INT32_MAX, y = INT32_MAX;
    decltype(plan.direction(0, 0)) direction{};
  };
  DirectionCache forward_cache, backward_cache;
  const auto direction = [&](std::int16_t x, std::int16_t y, DirectionCache& cache) {
    if (x != cache.x || y != cache.y) {
      cache.direction = plan.direction(x, y);
      cache.x = x;
      cache.y = y;
    }
    return cache.direction;
  };
  for (int y = 0; y < plan.height(); ++y)
    for (int x = 0; x < plan.width(); ++x) {
      const auto i = std::size_t(y) * plan.width() + x;
      const auto f = direction(forward.x[i], forward.y[i], forward_cache);
      const auto b = direction(backward.x[i], backward.y[i], backward_cache);
      const int count = 1 + f.count + b.count;
      const T* centre = source.planes[0].row(y + g.pad_y).data() + x + g.pad_x;
      T* out = output.row(y).data() + x;
      if (count == 1) {
        std::memcpy(out, centre, sizeof(T)); // Preserve all representations for copy-only trajectories.
        continue;
      }
      subpixel_detail::valid_sample(*centre, maximum);
      using Sum = std::conditional_t<std::is_same_v<T, float>, double, std::uint32_t>;
      Sum sum = *centre;
      for (const auto d : {f, b}) {
        if (!d.count)
          continue;
        BlurVisit<sizeof(T)>(g, x, y, d.count, d.x, d.y, &storage, [&](int, const std::byte* input) {
          T sample;
          std::memcpy(&sample, input, sizeof(T));
          subpixel_detail::valid_sample(sample, maximum);
          sum = sum + Sum(sample); // Centre, forward, backward; binary64 order is contractual.
          if constexpr (std::is_same_v<T, float>)
            if (!std::isfinite(sum))
              throw std::overflow_error("non-finite blur accumulation");
        });
      }
      if constexpr (std::is_same_v<T, float>)
        *out = mask_detail::binary32(sum / double(count));
      else
        *out = static_cast<T>(sum / std::uint32_t(count));
    }
}
#define NEO_BLUR_PLANE_IMPL(T, S)                                                                                      \
  void BlurPlane##S(const BlurSamplingPlan& p, const DenseFlowField& f, const DenseFlowField& b,                       \
                    const SubpixelPhases<T>& s, span2d::Plane<T> o, int bits) {                                        \
    BlurPlane(p, f, b, s, o, bits);                                                                                    \
  }
NEO_BLUR_PLANE_IMPL(std::uint8_t, U8)
NEO_BLUR_PLANE_IMPL(std::uint16_t, U16)
NEO_BLUR_PLANE_IMPL(float, F32)
#undef NEO_BLUR_PLANE_IMPL
#define NEO_BLUR_VARIANT(SUFFIX, BYTES)                                                                                \
  void BlurSamples##SUFFIX(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t sx, std::int64_t sy,    \
                           const FlowSampleStorage* storage) {                                                         \
    BlurSamples<BYTES>(g, x, y, count, sx, sy, storage);                                                               \
  }
NEO_BLUR_VARIANT(8, 1)
NEO_BLUR_VARIANT(16, 2)
NEO_BLUR_VARIANT(32, 4)
#undef NEO_BLUR_VARIANT
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd {
HWY_EXPORT(BlurPreflight);
void blur_preflight(const BlurSamplingPlan& p, const DenseFlowField& f, const DenseFlowField& b) {
  HWY_DYNAMIC_DISPATCH(BlurPreflight)(p, f, b);
}
#define NEO_BLUR_PLANE_EXPORT(T, S)                                                                                    \
  HWY_EXPORT(BlurPlane##S);                                                                                            \
  void blur_plane(const BlurSamplingPlan& p, const DenseFlowField& f, const DenseFlowField& b,                         \
                  const SubpixelPhases<T>& s, span2d::Plane<T> o, int bits) {                                          \
    HWY_DYNAMIC_DISPATCH(BlurPlane##S)(p, f, b, s, o, bits);                                                           \
  }
NEO_BLUR_PLANE_EXPORT(std::uint8_t, U8)
NEO_BLUR_PLANE_EXPORT(std::uint16_t, U16)
NEO_BLUR_PLANE_EXPORT(float, F32)
#undef NEO_BLUR_PLANE_EXPORT
HWY_EXPORT(BlurSamples8);
HWY_EXPORT(BlurSamples16);
HWY_EXPORT(BlurSamples32);
void blur_samples(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t sx, std::int64_t sy,
                  const FlowSampleStorage* storage) {
  if (!storage || storage->sample_bytes == 1)
    HWY_DYNAMIC_DISPATCH(BlurSamples8)(g, x, y, count, sx, sy, storage);
  else if (storage->sample_bytes == 2)
    HWY_DYNAMIC_DISPATCH(BlurSamples16)(g, x, y, count, sx, sy, storage);
  else
    HWY_DYNAMIC_DISPATCH(BlurSamples32)(g, x, y, count, sx, sy, storage);
}
} // namespace neo_mv::simd
#endif
