#pragma once
#include "core/render/fused.hpp"
#include <type_traits>
#include <vector>

namespace neo_mv {
// Request-owned blocks keep the existing composition kernels and Super layout.
// Reconstruct half-phase positions linearly; this is not a new Wiener phase.
template <class T>
void refine_chroma_block(SampledRenderBlock<T>& output, std::vector<std::vector<T>>& storage,
                         const RenderPhaseGeometry& g, BlockRegion block, RenderDisplacement d,
                         const SubpixelPhases<T>& image) {
  using namespace render_sampling_detail;
  const int fx = remainder(d.x, g.ratio_x), fy = remainder(d.y, g.ratio_y);
  if (!fx && !fy)
    return;
  const auto base = render_footprint_admitted(g, block, d);
  const auto ax = coordinate(block.x, g.pel, d.x, g.ratio_x);
  const auto ay = coordinate(block.y, g.pel, d.y, g.ratio_y);
  storage.emplace_back(std::size_t(base.width) * base.height);
  auto& pixels = storage.back();
  const int denominator = g.ratio_x * g.ratio_y;
  for (int y = 0; y < base.height; ++y)
    for (int x = 0; x < base.width; ++x) {
      using Sum = std::conditional_t<std::is_floating_point_v<T>, double, std::int64_t>;
      Sum sum = 0;
      for (int j = 0; j <= (fy != 0); ++j)
        for (int i = 0; i <= (fx != 0); ++i) {
          const int weight = (i ? fx : g.ratio_x - fx) * (j ? fy : g.ratio_y - fy);
          const auto plane = image.planes[remainder(ay + j, g.pel) * g.pel + remainder(ax + i, g.pel)];
          // Replicate the logical phase edge, including trimmed pel=4 phases.
          const auto sx = std::clamp<std::int64_t>(g.pad_x + sampling_detail::floor_div(ax + i, g.pel) + x,
                                                 0, plane.width() - 1);
          const auto sy = std::clamp<std::int64_t>(g.pad_y + sampling_detail::floor_div(ay + j, g.pel) + y,
                                                 0, plane.height() - 1);
          sum += Sum(plane.row(int(sy))[int(sx)]) * weight;
        }
      if constexpr (std::is_floating_point_v<T>)
        pixels[std::size_t(y) * base.width + x] = T(sum / denominator);
      else
        pixels[std::size_t(y) * base.width + x] = T((sum + denominator / 2) / denominator);
    }
  output.data = pixels.data();
  output.stride = base.width;
}
} // namespace neo_mv
