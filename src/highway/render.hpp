#pragma once
#include "core/render/block_sampling.hpp"
#include "highway/rows.hpp"

namespace neo_mv::simd {
template <class T, bool Validated = false>
void sample_render_block(const RenderPhaseGeometry& g, BlockRegion b, RenderDisplacement d,
                         const SubpixelPhases<T>& source, span2d::Plane<T> output, int bits) {
  const auto footprint = render_footprint(g, b, d);
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  if constexpr (!Validated) {
    validate_plane(output);
    if (source.pel != g.pel || output.width() != footprint.width || output.height() != footprint.height)
      throw std::invalid_argument("render block storage geometry mismatch");
    for (int a = 0; a < g.pel * g.pel; ++a) {
      const auto view = source.planes[a];
      validate_plane(view);
      if (view.width() != g.phases[a].width || view.height() != g.phases[a].height || active_rows_overlap(view, output))
        throw std::invalid_argument("render phase storage mismatch or output aliases input");
    }
  }
  const auto input =
      source.planes[footprint.phase].subplane(footprint.x, footprint.y, footprint.width, footprint.height);
  // Reject invalid samples before writing even a partial block. Row gaps and
  // undefined quarter-phase edges are never read. Copies preserve float bits.
  for (int y = 0; y < input.height(); ++y)
    detail::scan(input.row(y).data(), input.width(), maximum);
  for (int y = 0; y < input.height(); ++y)
    detail::copy(input.row(y).data(), output.row(y).data(), input.width());
}
} // namespace neo_mv::simd
