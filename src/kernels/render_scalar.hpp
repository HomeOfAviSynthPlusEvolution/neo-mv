#pragma once

#include "core/render/fused.hpp"
#include "core/render/compensation.hpp"
#include "core/render/overlap.hpp"
#include "core/render/weighted_samples.hpp"

namespace neo_mv {
// Admission and reference-selection rules stay in core; counting and pixel
// kernels can be selected independently of their shared orchestration.
template <class T>
struct ScalarRenderKernels : RenderPreparation<T> {
  using CompensationBlocks = typename RenderPreparation<T>::CompensationBlocks;
  using DegrainPlane = neo_mv::DegrainPlane<T>;
  static void compose_compensated(const OverlapCompositionPlan& plan, const CompensationBlocks& blocks,
                                  span2d::Plane<T> out, int bits) {
    compose_streamed(
        plan, out, subpixel_detail::sample_max<T>(bits),
        [&](std::size_t i, int x, int y) { return blocks[i].data[y * blocks[i].stride + x]; },
        [&](std::size_t i) { return blocks[i].coefficients; }, [](int) {});
  }
  static void compose_degrain(const OverlapCompositionPlan& plan, const DegrainPlane& p, span2d::Plane<const T> centre,
                              span2d::Plane<T> out, const ChangeLimit<T>& limit) {
    const auto read = [&](std::size_t i, int x, int y) {
      const auto base = i * (p.references + 1);
      const auto* sources = p.sources.data() + base;
      const auto* weights = p.weights.data() + base;
      const T c = sources[0].data[y * sources[0].stride + x];
      subpixel_detail::valid_sample(c, limit.maximum());
      using A = subpixel_detail::Acc<T>;
      A sum = A(c) * A(weights[0]);
      if constexpr (std::is_same_v<T, float>) {
        if (!std::isfinite(sum))
          throw std::overflow_error("non-finite weighted centre sample");
      } else
        sum += 128;
      for (int r = 1; r <= p.references; ++r) {
        const T q = sources[r].data ? sources[r].data[y * sources[r].stride + x] : c;
        subpixel_detail::valid_sample(q, limit.maximum());
        const A product = A(q) * A(weights[r]);
        sum = sum + product;
        if constexpr (std::is_same_v<T, float>)
          if (!std::isfinite(product) || !std::isfinite(sum))
            throw std::overflow_error("non-finite weighted reference intermediate");
      }
      return T(sum / A(256));
    };
    compose_streamed(
        plan, out, limit.maximum(), read, [&](std::size_t i) { return p.sources[i * (p.references + 1)].coefficients; },
        [&](int y) {
          if (limit.active())
            for (int x = 0; x < out.width(); ++x)
              out.row(y)[x] = limit(out.row(y)[x], centre.row(y)[x]);
        });
  }
  // Internal validated calls require plan-admitted geometry and input views,
  // disjoint owned outputs, and (for weighting/composition) admitted samples.
  // Sample producers continue to check values read from borrowed images.

  static constexpr auto scene_count = &scalar_scene_count;
  static constexpr auto scene_count_validated = &scalar_scene_count_validated;
  static constexpr auto sample_render_block = &neo_mv::sample_render_block<T>;
  static constexpr auto sample_render_block_validated = &neo_mv::sample_render_block<T, true>;
  static constexpr auto sample_compensated_block = &neo_mv::sample_compensated_block<T>;
  static constexpr auto sample_compensated_block_validated = &neo_mv::sample_compensated_block<T, true>;
  static constexpr auto weighted_render_block = &neo_mv::weighted_render_block<T>;
  static constexpr auto weighted_render_block_validated = &neo_mv::weighted_render_block<T, true>;
  static constexpr auto compose_render_blocks = &neo_mv::compose_render_blocks<T>;
  static constexpr auto compose_render_blocks_validated = &neo_mv::compose_render_blocks<T, true>;
  static constexpr auto limit_render_plane = &neo_mv::limit_render_plane<T>;
};
} // namespace neo_mv
