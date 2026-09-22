#include "highway/flow_sampling.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/flow_sampling.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <std::size_t Bytes>
void FlowSample(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage) {
  const auto& g = plan.geometry();
  const hn::ScalableTag<std::int64_t> d;
  const int lanes = static_cast<int>(hn::Lanes(d));
  HWY_ALIGN std::int64_t vx[hn::MaxLanes(d)]{}, vy[hn::MaxLanes(d)]{};
  HWY_ALIGN std::int64_t columns[hn::MaxLanes(d)], rows[hn::MaxLanes(d)], phases[hn::MaxLanes(d)];
  HWY_ALIGN std::int64_t widths[16]{}, heights[16]{};
  for (int a = 0; a < g.pel * g.pel; ++a) {
    widths[a] = g.phases[a].width;
    heights[a] = g.phases[a].height;
  }
  const int shift = g.pel == 4 ? 2 : g.pel == 2 ? 1 : 0;
  const auto time = hn::Set(d, plan.time_coefficient()), half = hn::Set(d, 128);
  const auto fraction = hn::Set(d, g.pel - 1), zero = hn::Zero(d);
  for (int y = 0; y < plan.height(); ++y) {
    for (int x = 0; x < plan.width();) {
      const int used = std::min(lanes, plan.width() - x);
      const auto index = std::size_t(y) * plan.width() + x;
      for (int i = 0; i < used; ++i) {
        vx[i] = field.x[index + i];
        vy[i] = field.y[index + i];
      }
      // Arithmetic right shifts implement floor division, including exact
      // negative multiples. All arithmetic stays in signed 64-bit lanes.
      const auto dx = hn::ShiftRight<8>(hn::Add(hn::Mul(hn::Load(d, vx), time), half));
      const auto dy = hn::ShiftRight<8>(hn::Add(hn::Mul(hn::Load(d, vy), time), half));
      const auto phase = hn::Add(hn::And(dx, fraction), hn::ShiftLeftSame(hn::And(dy, fraction), shift));
      const auto sx = hn::Add(hn::Set(d, g.pad_x), hn::Add(hn::Iota(d, x), hn::ShiftRightSame(dx, shift)));
      const auto sy = hn::Add(hn::Set(d, std::int64_t(g.pad_y) + y), hn::ShiftRightSame(dy, shift));
      const auto valid_x = hn::And(hn::Ge(sx, zero), hn::Lt(sx, hn::GatherIndex(d, widths, phase)));
      const auto valid_y = hn::And(hn::Ge(sy, zero), hn::Lt(sy, hn::GatherIndex(d, heights, phase)));
      if (!hn::AllTrue(d, hn::Or(hn::Not(hn::FirstN(d, used)), hn::And(valid_x, valid_y))))
        throw std::invalid_argument("Flow sample exceeds its logical phase domain");
      if (storage) {
        hn::Store(sx, d, columns);
        hn::Store(sy, d, rows);
        hn::Store(phase, d, phases);
        for (int i = 0; i < used; ++i) {
          const auto a = static_cast<std::size_t>(phases[i]);
          const auto* source = storage->planes[a] + rows[i] * storage->strides[a] + columns[i] * Bytes;
          auto* output = storage->output + std::ptrdiff_t(y) * storage->output_stride +
                         std::size_t(x + i) * Bytes;
          std::memcpy(output, source, Bytes);
        }
      }
      x += used;
    }
  }
}
void FlowSample8(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage) {
  FlowSample<1>(plan, field, storage);
}
void FlowSample16(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage) {
  FlowSample<2>(plan, field, storage);
}
void FlowSample32(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage) {
  FlowSample<4>(plan, field, storage);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd {
HWY_EXPORT(FlowSample8);
HWY_EXPORT(FlowSample16);
HWY_EXPORT(FlowSample32);
void flow_sample(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage) {
  if (!storage || storage->sample_bytes == 1)
    HWY_DYNAMIC_DISPATCH(FlowSample8)(plan, field, storage);
  else if (storage->sample_bytes == 2)
    HWY_DYNAMIC_DISPATCH(FlowSample16)(plan, field, storage);
  else
    HWY_DYNAMIC_DISPATCH(FlowSample32)(plan, field, storage);
}
} // namespace neo_mv::simd
#endif
