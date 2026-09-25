#pragma once
#include "core/motion/metric_policy.hpp"
#include "core/motion/composition.hpp"

namespace neo_mv {
struct MetricFrameContext { int base_weight = 0; };
struct MetricLayerPlan {
  MetricConfig config;
  MetricDescriptor descriptor;
  std::int64_t weight, unit;
};
inline MetricLayerPlan metric_layer_plan(MetricConfig config, MetricFrameContext context) {
  auto d = metric_descriptor(config.mode);
  const auto weight = d.mix == MetricMix::global ? context.base_weight / d.global_divisor : config.weight;
  const auto unit = d.mix == MetricMix::global ? 16 : metric_unit;
  if (d.mix != MetricMix::none) {
    if (weight == 0 || (d.mix == MetricMix::local && config.threshold == metric_unit)) {
      d.transform = MetricTransform::sad;
      d.mix = MetricMix::none;
    } else if (d.mix == MetricMix::global && weight == unit) {
      d.mix = MetricMix::none;
    }
  }
  return {config, d, weight, unit};
}
template <class T, class Kernels>
MetricFrameContext metric_frame_context(const AnalysisMetadata& m, const SamplingGeometry& geometry,
                                        const SamplingFrames<T>& frames) {
  static_assert(std::is_integral_v<T>);
  std::int64_t delta = 0;
  for (int y = 0; y < m.blocks_y; ++y)
    for (int x = 0; x < m.blocks_x; ++x) {
      const auto block = analysis_block(m, x, y);
      const auto source = source_block(geometry, block, frames, 0);
      const auto reference = reference_block(geometry, block, frames, 0, {0, 0});
      delta = prediction_detail::add(delta, Kernels::block_luma_sum(reference) - Kernels::block_luma_sum(source));
    }
  const auto mean = delta / (std::int64_t(m.blocks_x) * m.blocks_y);
  const auto normalized = std::abs(mean) >> (m.bits - 8);
  return {int(std::min<std::int64_t>(16, normalized / (std::int64_t(m.block_width) * m.block_height)))};
}
} // namespace neo_mv
