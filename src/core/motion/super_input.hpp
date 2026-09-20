#pragma once

#include "core/motion/analysis_field.hpp"
#include "core/super/pyramid.hpp"

namespace neo_mv {

// Derive the complete read domains at creation, before any frame is available.
// Chroma ratios describe the source format even when the metric uses luma only.
template <class T>
std::vector<SamplingGeometry> super_sampling_geometry(const SuperPlan<T>& plan, bool chroma = true) {
  const auto& g = plan.geometry();
  const auto& p = plan.params();
  std::vector<SamplingGeometry> result(g.planes[0].levels.size());
  for (std::size_t l = 0; l < result.size(); ++l) {
    auto& sampling = result[l];
    sampling.pel = l == 0 ? p.pel : 1;
    sampling.ratio_x = p.ratio_x;
    sampling.ratio_y = p.ratio_y;
    sampling.chroma = chroma && p.chroma;
    for (int k = 0; k < (sampling.chroma ? 3 : 1); ++k) {
      const auto& plane = g.planes[k];
      const auto& size = plane.levels[l];
      auto& out = sampling.planes[k];
      out.pad_x = plane.pad_x;
      out.pad_y = plane.pad_y;
      out.current = {size.padded_width, size.padded_height};
      const bool quarter = sampling.pel == 4 && !plan.external();
      for (int ay = 0; ay < sampling.pel; ++ay)
        for (int ax = 0; ax < sampling.pel; ++ax)
          out.reference[ay * sampling.pel + ax] = {size.padded_width - (quarter && ax == 3),
                                                   size.padded_height - (quarter && ay == 3)};
    }
  }
  return result;
}

// Initial descriptor with Super's block/overlap hints. A caller resolving target
// overrides must run Analyse/Recalculate's geometry admission afterward.
template <class T>
AnalysisMetadata super_analysis_metadata(const SuperPlan<T>& plan, std::int32_t delta, bool chroma = true) {
  const auto& p = plan.params();
  const auto& g = plan.geometry();
  AnalysisMetadata m;
  m.width = g.planes[0].levels[0].width;
  m.height = g.planes[0].levels[0].height;
  m.real_width = p.width;
  m.real_height = p.height;
  m.pad_x = p.pad_x;
  m.pad_y = p.pad_y;
  m.pel = p.pel;
  m.levels = static_cast<std::int32_t>(g.planes[0].levels.size());
  m.chroma = chroma && p.chroma;
  m.ratio_x = p.ratio_x;
  m.ratio_y = p.ratio_y;
  m.block_width = p.block_width;
  m.block_height = p.block_height;
  m.overlap_x = p.overlap_x;
  m.overlap_y = p.overlap_y;
  m.blocks_x = g.blocks_x;
  m.blocks_y = g.blocks_y;
  m.delta = delta;
  m.bits = plan.bits();
  return m;
}

// Match logical geometry and sample format, including disabled chroma planes.
// Kernel choices may differ when they leave those logical domains unchanged.
template <class T>
void validate_super_pair(const SuperPlan<T>& current, const SuperPlan<T>& reference) {
  const auto& a = current.params();
  const auto& b = reference.params();
  if (current.bits() != reference.bits() || a.width != b.width || a.height != b.height || a.chroma != b.chroma ||
      a.ratio_x != b.ratio_x || a.ratio_y != b.ratio_y)
    throw std::invalid_argument("Super pair sample formats or actual dimensions differ");
  const auto ag = super_sampling_geometry(current), bg = super_sampling_geometry(reference);
  if (ag.size() != bg.size())
    throw std::invalid_argument("Super pair level counts differ");
  const auto same_extent = [](PhaseExtent x, PhaseExtent y) {
    return x.width == y.width && x.height == y.height;
  };
  for (std::size_t l = 0; l < ag.size(); ++l) {
    if (ag[l].pel != bg[l].pel)
      throw std::invalid_argument("Super pair pel differs");
    for (int k = 0; k < (a.chroma ? 3 : 1); ++k) {
      const auto& x = ag[l].planes[k];
      const auto& y = bg[l].planes[k];
      if (x.pad_x != y.pad_x || x.pad_y != y.pad_y || !same_extent(x.current, y.current))
        throw std::invalid_argument("Super pair integer geometry differs");
      for (int phase = 0; phase < ag[l].pel * ag[l].pel; ++phase)
        if (!same_extent(x.reference[phase], y.reference[phase]))
          throw std::invalid_argument("Super pair phase domains differ");
    }
  }
}

// Borrowed views only. Both immutable payload owners must remain alive and must
// not be assigned/moved for the entire use of these frames by motion kernels.
template <class T>
std::vector<SamplingFrames<T>> borrow_super_frames(const SuperPyramid<T>& current, const SuperPyramid<T>& reference,
                                                   bool chroma = true) {
  validate_super_pair(current.plan(), reference.plan());
  const auto geometries = super_sampling_geometry(current.plan(), chroma);
  std::vector<SamplingFrames<T>> result(geometries.size());
  for (std::size_t l = 0; l < result.size(); ++l) {
    const auto& g = geometries[l];
    for (int k = 0; k < (g.chroma ? 3 : 1); ++k) {
      result[l].current[k] = current.phase(k, static_cast<int>(l));
      for (int ay = 0; ay < g.pel; ++ay)
        for (int ax = 0; ax < g.pel; ++ax)
          result[l].reference[k][ay * g.pel + ax] = reference.phase(k, static_cast<int>(l), ax, ay);
    }
  }
  return result;
}
} // namespace neo_mv
