#include "highway/blur_sampling.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/blur_sampling.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <std::size_t Bytes>
void BlurSamples(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t step_x,
                 std::int64_t step_y, const FlowSampleStorage* storage) {
  const hn::ScalableTag<std::int64_t> d;
  const int lanes = static_cast<int>(hn::Lanes(d));
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
    const auto sample = hn::Iota(d, first);
    const auto dx = hn::ShiftRight<8>(hn::Mul(sample, hn::Set(d, step_x)));
    const auto dy = hn::ShiftRight<8>(hn::Mul(sample, hn::Set(d, step_y)));
    const auto phase = hn::Add(hn::And(dx, fraction), hn::ShiftLeftSame(hn::And(dy, fraction), shift));
    const auto sx = hn::Add(hn::Set(d, std::int64_t(g.pad_x) + x), hn::ShiftRightSame(dx, shift));
    const auto sy = hn::Add(hn::Set(d, std::int64_t(g.pad_y) + y), hn::ShiftRightSame(dy, shift));
    const auto valid_x = hn::And(hn::Ge(sx, zero), hn::Lt(sx, hn::GatherIndex(d, widths, phase)));
    const auto valid_y = hn::And(hn::Ge(sy, zero), hn::Lt(sy, hn::GatherIndex(d, heights, phase)));
    if (!hn::AllTrue(d, hn::Or(hn::Not(hn::FirstN(d, used)), hn::And(valid_x, valid_y))))
      throw std::invalid_argument("blur trajectory exceeds logical phase domain");
    if (storage) {
      hn::Store(sx, d, columns);
      hn::Store(sy, d, rows);
      hn::Store(phase, d, phases);
      for (int i = 0; i < used; ++i) {
        const auto a = static_cast<std::size_t>(phases[i]);
        const auto* input = storage->planes[a] + rows[i] * storage->strides[a] + columns[i] * Bytes;
        std::memcpy(storage->output + std::size_t(first - 1 + i) * Bytes, input, Bytes);
      }
    }
    first += used;
  }
}
#define NEO_BLUR_VARIANT(SUFFIX, BYTES) \
  void BlurSamples##SUFFIX(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t sx, \
                           std::int64_t sy, const FlowSampleStorage* storage) { \
    BlurSamples<BYTES>(g, x, y, count, sx, sy, storage); \
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
HWY_EXPORT(BlurSamples8);
HWY_EXPORT(BlurSamples16);
HWY_EXPORT(BlurSamples32);
void blur_samples(const RenderPhaseGeometry& g, int x, int y, int count, std::int64_t sx,
                  std::int64_t sy, const FlowSampleStorage* storage) {
  if (!storage || storage->sample_bytes == 1)
    HWY_DYNAMIC_DISPATCH(BlurSamples8)(g, x, y, count, sx, sy, storage);
  else if (storage->sample_bytes == 2)
    HWY_DYNAMIC_DISPATCH(BlurSamples16)(g, x, y, count, sx, sy, storage);
  else
    HWY_DYNAMIC_DISPATCH(BlurSamples32)(g, x, y, count, sx, sy, storage);
}
} // namespace neo_mv::simd
#endif
