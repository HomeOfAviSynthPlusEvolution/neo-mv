#pragma once

#include "core/motion/prediction.hpp"
#include "core/super/subpixel.hpp"

namespace neo_mv {

struct RenderDisplacement {
  std::int64_t x, y;
};
struct RenderPhaseGeometry {
  int pel, ratio_x, ratio_y, pad_x, pad_y;
  std::array<PhaseExtent, 16> phases{};
};
struct RenderFootprint {
  int phase, x, y, width, height;
};

inline RenderPhaseGeometry render_phase_geometry(const SamplingGeometry& g, int plane) {
  if (plane < 0 || plane >= (g.chroma ? 3 : 1))
    throw std::invalid_argument("render plane does not exist");
  const auto& p = g.planes[plane];
  return {g.pel, plane == 0 ? 1 : g.ratio_x, plane == 0 ? 1 : g.ratio_y, p.pad_x, p.pad_y, p.reference};
}

namespace render_sampling_detail {
inline void validate(const RenderPhaseGeometry& g, BlockRegion b) {
  if ((g.pel != 1 && g.pel != 2 && g.pel != 4) || (g.ratio_x != 1 && g.ratio_x != 2) ||
      (g.ratio_y != 1 && g.ratio_y != 2) || g.pad_x < 0 || g.pad_y < 0 || b.x < 0 || b.y < 0 || b.width <= 0 ||
      b.height <= 0 || b.x % g.ratio_x || b.y % g.ratio_y || b.width % g.ratio_x || b.height % g.ratio_y)
    throw std::invalid_argument("invalid render sampling geometry");
  for (int a = 0; a < g.pel * g.pel; ++a)
    if (g.phases[a].width <= 0 || g.phases[a].height <= 0)
      throw std::invalid_argument("invalid render phase extent");
}
inline std::int64_t coordinate(int origin, int pel, std::int64_t displacement, int ratio) {
  return sampling_detail::floor_div(prediction_detail::add(std::int64_t(origin) * pel, displacement), ratio);
}
inline int remainder(std::int64_t value, int pel) {
  const int r = static_cast<int>(value % pel);
  return r < 0 ? r + pel : r;
}
inline bool fits(std::int64_t start, int size, int extent) {
  return start >= 0 && start <= std::int64_t(extent) - size;
}
inline sampling_detail::OffsetRange phase_range(std::int64_t first, std::int64_t last, int pel, int phase) {
  return {sampling_detail::floor_div(first, pel) + (remainder(first, pel) > phase),
          sampling_detail::floor_div(last, pel) - (remainder(last, pel) < phase)};
}
} // namespace render_sampling_detail

// Unlike analysis-error sampling, floor the total coordinate, including the
// negative chroma displacement. The returned rectangle includes cropped edges.
template <bool GeometryAdmitted = false, bool DomainProven = false>
inline RenderFootprint render_footprint_impl(const RenderPhaseGeometry& g, BlockRegion b, RenderDisplacement d) {
  using namespace render_sampling_detail;
  if constexpr (!GeometryAdmitted)
    validate(g, b);
  // In frame preparation, creation already admitted the geometry and the
  // decoded vector is int32 (plus at most one field shift). The sums fit i64.
  const auto ax = GeometryAdmitted ? sampling_detail::floor_div(std::int64_t(b.x) * g.pel + d.x, g.ratio_x)
                                   : coordinate(b.x, g.pel, d.x, g.ratio_x);
  const auto ay = GeometryAdmitted ? sampling_detail::floor_div(std::int64_t(b.y) * g.pel + d.y, g.ratio_y)
                                   : coordinate(b.y, g.pel, d.y, g.ratio_y);
  const int phase = remainder(ay, g.pel) * g.pel + remainder(ax, g.pel);
  const auto x = GeometryAdmitted ? std::int64_t(g.pad_x) + sampling_detail::floor_div(ax, g.pel)
                                  : prediction_detail::add(g.pad_x, sampling_detail::floor_div(ax, g.pel));
  const auto y = GeometryAdmitted ? std::int64_t(g.pad_y) + sampling_detail::floor_div(ay, g.pel)
                                  : prediction_detail::add(g.pad_y, sampling_detail::floor_div(ay, g.pel));
  const int width = b.width / g.ratio_x, height = b.height / g.ratio_y;
  if constexpr (!DomainProven)
    if (!fits(x, width, g.phases[phase].width) || !fits(y, height, g.phases[phase].height))
      throw std::invalid_argument("render block exceeds its logical phase domain");
  return {phase, static_cast<int>(x), static_cast<int>(y), width, height};
}
inline RenderFootprint render_footprint(const RenderPhaseGeometry& g, BlockRegion b, RenderDisplacement d) {
  return render_footprint_impl(g, b, d);
}
inline RenderFootprint render_footprint_admitted(const RenderPhaseGeometry& g, BlockRegion b, RenderDisplacement d) {
  return render_footprint_impl<true>(g, b, d);
}
// Frame plans may use this only after admitting the complete vector domain
// for this block and plane. A field shift invalidates that proof.
inline RenderFootprint render_footprint_domain_proven(const RenderPhaseGeometry& g, BlockRegion b,
                                                      RenderDisplacement d) {
  return render_footprint_impl<true, true>(g, b, d);
}

// Geometry proof for every integer vector in the unchanged public rectangle.
// t=256 is Degrain; Compensate supplies its quantized time in [0,256]. Both
// trunc(v*t/256) and subsequent floor division are monotone with steps <=1,
// so their endpoint interval has no holes. Each phase needs only its extrema.
// Field shifts on actual Compensate vectors are checked separately at runtime.
inline void validate_render_domain(const RenderPhaseGeometry& g, BlockRegion b, CandidateDomain domain, int t = 256) {
  using namespace render_sampling_detail;
  validate(g, b);
  if (t < 0 || t > 256 || domain.left >= domain.right || domain.top >= domain.bottom)
    throw std::invalid_argument("invalid render vector domain or time coefficient");
  const auto mapped = [&](std::int64_t v, int origin, int ratio) {
    return coordinate(origin, g.pel, prediction_detail::weight(v, t) / 256, ratio);
  };
  const auto left = mapped(domain.left, b.x, g.ratio_x), right = mapped(domain.right - 1, b.x, g.ratio_x);
  const auto top = mapped(domain.top, b.y, g.ratio_y), bottom = mapped(domain.bottom - 1, b.y, g.ratio_y);
  for (int ay = 0; ay < g.pel; ++ay) {
    const auto dy = phase_range(top, bottom, g.pel, ay);
    if (dy.first > dy.last)
      continue;
    for (int ax = 0; ax < g.pel; ++ax) {
      const auto dx = phase_range(left, right, g.pel, ax);
      if (dx.first > dx.last)
        continue;
      const auto extent = g.phases[ay * g.pel + ax];
      if (prediction_detail::add(g.pad_x, dx.first) < 0 || prediction_detail::add(g.pad_y, dy.first) < 0 ||
          !fits(prediction_detail::add(g.pad_x, dx.last), b.width / g.ratio_x, extent.width) ||
          !fits(prediction_detail::add(g.pad_y, dy.last), b.height / g.ratio_y, extent.height))
        throw std::invalid_argument("public vector domain exceeds render phase domains");
    }
  }
}

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
    for (int x = 0; x < input.width(); ++x)
      subpixel_detail::valid_sample(input.row(y)[x], maximum);
  for (int y = 0; y < input.height(); ++y)
    for (int x = 0; x < input.width(); ++x)
      output.row(y)[x] = input.row(y)[x];
}

} // namespace neo_mv
