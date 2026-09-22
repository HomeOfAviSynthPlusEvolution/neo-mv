#pragma once
#include "highway/render.hpp"
#include "highway/render_rows.hpp"
#include "highway/scene.hpp"
#include "core/render/weighted_samples.hpp"
#include "core/render/change_limit.hpp"
#include "core/render/overlap.hpp"
#include "core/render/compensation.hpp"

namespace neo_mv::simd {
template <class T, bool Validated = false>
void weighted_render_block(span2d::Plane<const T> centre, const std::vector<WeightedReferenceBlock<T>>& references,
                           const DegrainWeights& weights, span2d::Plane<T> output, int bits) {
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  if constexpr (!Validated) {
    if (references.size() < 2 || references.size() > 50 || references.size() % 2 != 0 ||
        references.size() != weights.reference.size() || weights.centre < 0 || weights.centre > 256)
      throw std::invalid_argument("invalid weighted block reference count or centre weight");
    int sum = weights.centre;
    for (std::size_t i = 0; i < references.size(); ++i) {
      const int w = weights.reference[i];
      if (w < 0 || w > 256 || (!references[i].available && w != 0))
        throw std::invalid_argument("invalid weighted block reference weight");
      sum += w;
    }
    if (sum != 256)
      throw std::invalid_argument("weighted block weights must sum to 256");
    validate_plane(output);
    const auto validate_input = [&](span2d::Plane<const T> input) {
      validate_plane(input);
      if (input.width() != output.width() || input.height() != output.height() || active_rows_overlap(input, output))
        throw std::invalid_argument("weighted block geometry mismatch or output alias");
      for (int y = 0; y < input.height(); ++y)
        detail::scan(input.row(y).data(), input.width(), maximum);
    };
    validate_input(centre);
    for (const auto& r : references)
      if (r.available)
        validate_input(r.samples);
  }
  std::array<const T*, 50> rows{};
  for (int y = 0; y < output.height(); ++y) {
    for (std::size_t i = 0; i < references.size(); ++i)
      rows[i] = references[i].available ? references[i].samples.row(y).data() : nullptr;
    detail::weighted(centre.row(y).data(), rows.data(), weights.reference.data(), weights.centre,
                     int(references.size()), output.row(y).data(), output.width());
  }
}
template <class T>
void limit_render_plane(const ChangeLimit<T>& limit, span2d::Plane<const T> composed, span2d::Plane<const T> centre,
                        span2d::Plane<T> output) {
  validate_plane(composed);
  validate_plane(centre);
  validate_plane(output);
  if (composed.width() != output.width() || composed.height() != output.height() || centre.width() != output.width() ||
      centre.height() != output.height() || active_rows_overlap(composed, output) ||
      active_rows_overlap(centre, output))
    throw std::invalid_argument("change limit plane geometry mismatch or output alias");
  if constexpr (std::is_same_v<T, float>) {
    for (int y = 0; y < output.height(); ++y) {
      detail::scan(composed.row(y).data(), output.width(), limit.maximum());
      detail::scan(centre.row(y).data(), output.width(), limit.maximum());
    }
  } else if (limit.maximum() != std::numeric_limits<T>::max()) {
    for (int y = 0; y < output.height(); ++y) {
      detail::scan(composed.row(y).data(), output.width(), limit.maximum());
      detail::scan(centre.row(y).data(), output.width(), limit.maximum());
    }
  }
  for (int y = 0; y < output.height(); ++y)
    detail::change_limit(composed.row(y).data(), centre.row(y).data(), output.row(y).data(), output.width(),
                         limit.active(), limit.maximum(), limit.integer_limit(), limit.float_limit());
}
template <class T, bool Validated = false>
void compose_render_blocks(const OverlapCompositionPlan& plan, const std::vector<span2d::Plane<const T>>& blocks,
                           span2d::Plane<T> output, int bits) {
  const auto& g = plan.geometry();
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  if constexpr (!Validated) {
    validate_plane(output);
    if (blocks.size() != std::uint64_t(g.blocks_x) * g.blocks_y || output.width() != g.visible_width ||
        output.height() != g.visible_height)
      throw std::invalid_argument("block composition storage geometry mismatch");
    for (auto block : blocks) {
      validate_plane(block);
      if (block.width() != g.block_width || block.height() != g.block_height || active_rows_overlap(block, output))
        throw std::invalid_argument("invalid block storage or output aliases input");
      // Cropping never excuses an invalid generated block rectangle.
      for (int y = 0; y < block.height(); ++y)
        detail::scan(block.row(y).data(), block.width(), maximum);
    }
  }
  const int sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
  using A = std::conditional_t<std::is_same_v<T, float>, float, std::int32_t>;
  // One row of local scratch; per-coordinate contribution order stays row-major.
  std::vector<A> sums(plan.has_overlap() ? super_detail::sample_count<A>(g.visible_width, 1) : 0);
  for (int y = 0; y < g.visible_height; ++y) {
    if (!plan.has_overlap()) {
      for (int x = 0; x < g.visible_width;) {
        const auto block = blocks[std::size_t(y / sy) * g.blocks_x + x / sx];
        const int count = std::min(sx, g.visible_width - x);
        detail::copy(block.row(y % sy).data(), output.row(y).data() + x, count);
        x += count;
      }
      continue;
    }
    std::fill(sums.begin(), sums.end(), A(0));
    const int first_y = y < g.block_height ? 0 : (y - g.block_height) / sy + 1;
    const int last_y = std::min(y / sy, g.blocks_y - 1);
    for (int by = first_y; by <= last_y; ++by) {
      const int ly = y - by * sy;
      for (int bx = 0; bx < g.blocks_x; ++bx) {
        const int x = bx * sx;
        if (x >= g.visible_width)
          break;
        const int count = std::min(g.block_width, g.visible_width - x);
        const auto block = blocks[std::size_t(by) * g.blocks_x + bx];
        detail::overlap_add(block.row(ly).data(), plan.coefficient_row(bx, by, ly), sums.data() + x, count);
      }
    }
    detail::overlap_finish(sums.data(), output.row(y).data(), g.visible_width, maximum);
  }
}
template <class T, bool Validated = false>
void sample_compensated_block(const CompensationRule& rule, const RenderPhaseGeometry& g, BlockRegion b,
                              MotionTriple vector, int shift, const SubpixelPhases<T>& current,
                              const SubpixelPhases<T>& reference, span2d::Plane<T> output, int bits) {
  if constexpr (!Validated)
    validate_compensation_footprint(rule, g, b, vector.vector, shift);
  const auto selected = rule.select(vector.vector, vector.error, shift);
  if constexpr (!Validated) {
    validate_plane(output);
    // Both images are required inputs even when this block selects only one.
    // Reject aliasing against the unselected image as well as the selected one.
    for (const auto* image : {&current, &reference}) {
      if (image->pel != g.pel)
        throw std::invalid_argument("compensation image phase count mismatch");
      for (int a = 0; a < g.pel * g.pel; ++a) {
        const auto plane = image->planes[a];
        validate_plane(plane);
        if (plane.width() != g.phases[a].width || plane.height() != g.phases[a].height ||
            active_rows_overlap(plane, output))
          throw std::invalid_argument("compensation image geometry mismatch or output alias");
      }
    }
  }
  neo_mv::simd::sample_render_block<T, Validated>(g, b, selected.displacement, selected.reference ? reference : current,
                                                  output, bits);
}

} // namespace neo_mv::simd

namespace neo_mv {
template <class T>
struct HighwayRenderKernels {
  static constexpr bool fused_compensation = true;
  using CompensationBlocks = std::vector<simd::detail::SampledRenderBlock<T>>;
  static CompensationBlocks prepare_compensated(const OverlapCompositionPlan& plan, const CompensationRule& rule,
                                                const RenderPhaseGeometry& geometry, const MotionGrid& field, int shift,
                                                const SubpixelPhases<T>& current, const SubpixelPhases<T>& reference) {
    const auto& g = plan.geometry();
    CompensationBlocks blocks;
    blocks.reserve(field.values.size());
    for (int by = 0; by < g.blocks_y; ++by)
      for (int bx = 0; bx < g.blocks_x; ++bx) {
        const BlockRegion block{bx * (g.block_width - g.overlap_x) * geometry.ratio_x,
                                by * (g.block_height - g.overlap_y) * geometry.ratio_y,
                                g.block_width * geometry.ratio_x, g.block_height * geometry.ratio_y};
        const auto v = field.values[std::size_t(by) * g.blocks_x + bx];
        const auto selected = rule.select(v.vector, v.error, shift);
        // Preflight the actual reference even when SAD selects current pixels.
        const auto reference_footprint =
            render_footprint(geometry, block, rule.reference_displacement(v.vector, shift));
        const auto footprint =
            selected.reference ? reference_footprint : render_footprint(geometry, block, selected.displacement);
        const auto plane = (selected.reference ? reference : current).planes[footprint.phase];
        blocks.push_back({plane.row(footprint.y).data() + footprint.x, plane.stride(),
                          plan.has_overlap() ? plan.coefficient_row(bx, by, 0) : nullptr});
      }
    return blocks;
  }
  static void compose_compensated(const OverlapCompositionPlan& plan, const CompensationBlocks& blocks,
                                  span2d::Plane<T> output, int bits) {
    simd::detail::compose_sampled(plan.geometry(), blocks.data(), output, subpixel_detail::sample_max<T>(bits));
  }

  // Internal validated calls require plan-admitted geometry and input views,
  // disjoint owned outputs, and (for weighting/composition) admitted samples.
  // Sample producers continue to check values read from borrowed images.

  static constexpr auto scene_count = &simd::scene_count;
  static constexpr auto sample_render_block = &simd::sample_render_block<T>;
  static constexpr auto sample_render_block_validated = &simd::sample_render_block<T, true>;
  static constexpr auto sample_compensated_block = &simd::sample_compensated_block<T>;
  static constexpr auto sample_compensated_block_validated = &simd::sample_compensated_block<T, true>;
  static constexpr auto weighted_render_block = &simd::weighted_render_block<T>;
  static constexpr auto weighted_render_block_validated = &simd::weighted_render_block<T, true>;
  static constexpr auto compose_render_blocks = &simd::compose_render_blocks<T>;
  static constexpr auto compose_render_blocks_validated = &simd::compose_render_blocks<T, true>;
  static constexpr auto limit_render_plane = &simd::limit_render_plane<T>;
};
} // namespace neo_mv
