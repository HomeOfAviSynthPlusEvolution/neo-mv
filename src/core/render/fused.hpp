#pragma once
#include "core/render/compensation.hpp"
#include "core/render/change_limit.hpp"
#include "core/render/degrain_weights.hpp"
#include "core/render/overlap.hpp"
namespace neo_mv {
// Internal borrowed blocks: full rectangles have admitted geometry, output is
// disjoint, and coefficient pointers refer to immutable plan windows.
template <class T>
struct SampledRenderBlock {
  const T* data;
  std::ptrdiff_t stride;
  const std::uint16_t* coefficients;
};
// Block-major borrowed sources, centre first followed by every reference.
// Missing references have null data and weight zero; available zero-weight
// references are still sampled and checked. Weights are normalized to 256.
template <class T>
struct DegrainPlane {
  int references = 0;
  std::vector<SampledRenderBlock<T>> sources;
  std::vector<int> weights;
};

// All prepared views borrow admitted image storage and immutable plan windows.
// Callers prepare every processed plane before reading samples into owned output.
template <class T>
struct RenderPreparation {
  using CompensationBlocks = std::vector<SampledRenderBlock<T>>;
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
            render_footprint_admitted(geometry, block, rule.reference_displacement(v.vector, shift));
        const auto footprint = selected.reference ? reference_footprint
                                                  : render_footprint_admitted(geometry, block, selected.displacement);
        const auto plane = (selected.reference ? reference : current).planes[footprint.phase];
        blocks.push_back({plane.row(footprint.y).data() + footprint.x, plane.stride(),
                          plan.has_overlap() ? plan.coefficient_row(bx, by, 0) : nullptr});
      }
    return blocks;
  }
  using DegrainPlane = neo_mv::DegrainPlane<T>;
  template <class Images>
  static DegrainPlane prepare_degrain(const OverlapCompositionPlan& plan, const RenderPhaseGeometry& geometry,
                                      const std::vector<AnalysisField>& fields,
                                      const std::vector<std::optional<std::int64_t>>& selected,
                                      const SubpixelPhases<T>& current, const Images& images,
                                      const DegrainWeightPlan& weight_plan, int plane_index) {
    const auto& g = plan.geometry();
    DegrainPlane result;
    result.references = int(selected.size());
    if (selected.size() > 50 || std::size_t(g.blocks_x) > std::size_t(-1) / std::size_t(g.blocks_y))
      throw std::overflow_error("Degrain block storage size is unrepresentable");
    const auto blocks = std::size_t(g.blocks_x) * g.blocks_y;
    const auto stride = selected.size() + 1;
    if (blocks > result.sources.max_size() / stride || blocks > result.weights.max_size() / stride)
      throw std::overflow_error("Degrain block storage size is unrepresentable");
    const auto count = blocks * stride;
    result.sources.reserve(count);
    result.weights.resize(count);
    std::array<ReferenceReliability, 50> reliability{};
    for (int by = 0; by < g.blocks_y; ++by)
      for (int bx = 0; bx < g.blocks_x; ++bx) {
        const auto index = std::size_t(by) * g.blocks_x + bx;
        const BlockRegion block{bx * (g.block_width - g.overlap_x) * geometry.ratio_x,
                                by * (g.block_height - g.overlap_y) * geometry.ratio_y,
                                g.block_width * geometry.ratio_x, g.block_height * geometry.ratio_y};
        const auto* window = plan.has_overlap() ? plan.coefficient_row(bx, by, 0) : nullptr;
        const auto append = [&](const SubpixelPhases<T>& image, RenderDisplacement displacement) {
          const auto f = render_footprint_admitted(geometry, block, displacement);
          const auto view = image.planes[f.phase];
          result.sources.push_back({view.row(f.y).data() + f.x, view.stride(), window});
        };
        append(current, {0, 0});
        for (std::size_t r = 0; r < selected.size(); ++r) {
          reliability[r] = {bool(selected[r]), selected[r] ? fields[r].grid.values[index].error : 0};
          if (selected[r]) {
            const auto v = fields[r].grid.values[index].vector;
            append(images[r].planes[plane_index], {v.x, v.y});
          } else
            result.sources.push_back({nullptr, 0, nullptr});
        }
        weight_plan.compute(reliability.data(), selected.size(), plane_index,
                            result.weights.data() + index * (selected.size() + 1));
      }
    return result;
  }
};

template <class T, class Read, class Window, class Finish>
void compose_streamed(const OverlapCompositionPlan& plan, span2d::Plane<T> out, std::int64_t maximum, Read&& read,
                      Window&& window, Finish&& finish) {
  const auto& g = plan.geometry();
  const int sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
  const int height = (g.blocks_y - 1) * sy + g.block_height;
  using A = subpixel_detail::Acc<T>;
  std::vector<A> sums(plan.has_overlap() ? g.visible_width : 0);
  for (int y = 0; y < height; ++y) {
    std::fill(sums.begin(), sums.end(), A(0));
    const int first = y < g.block_height ? 0 : (y - g.block_height) / sy + 1;
    const int last = std::min(y / sy, g.blocks_y - 1);
    for (int by = first; by <= last; ++by)
      for (int bx = 0; bx < g.blocks_x; ++bx) {
        const auto i = std::size_t(by) * g.blocks_x + bx;
        const int ly = y - by * sy, ox = bx * sx;
        const auto* coeff = window(i);
        for (int x = 0; x < g.block_width; ++x) {
          const T q = read(i, x, ly);
          // Full block numeric admission survives visible output cropping.
          subpixel_detail::valid_sample(q, maximum);
          if (y >= g.visible_height || ox + x >= g.visible_width)
            continue;
          if (!plan.has_overlap()) {
            out.row(y)[ox + x] = q;
            continue;
          }
          const auto w = coeff[std::size_t(ly) * g.block_width + x];
          auto& sum = sums[ox + x];
          if constexpr (std::is_same_v<T, float>) {
            const float product = q * float(w);
            sum = sum + product / 64.0f;
            if (!std::isfinite(product) || !std::isfinite(sum))
              throw std::overflow_error("non-finite overlap intermediate");
          } else
            sum += (std::int64_t(q) * w) / 64;
        }
      }
    if (y < g.visible_height) {
      if (plan.has_overlap())
        for (int x = 0; x < g.visible_width; ++x) {
          if constexpr (std::is_same_v<T, float>)
            out.row(y)[x] = sums[x] / 32.0f;
          else
            out.row(y)[x] = T(std::clamp((sums[x] + 16) / 32, std::int64_t(0), maximum));
        }
      finish(y);
    }
  }
}
} // namespace neo_mv
