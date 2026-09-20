#pragma once

#include "core/motion/block_metric.hpp"

#include <initializer_list>

namespace neo_mv {

struct MotionVector {
  std::int32_t x, y;
};
struct BlockRegion {
  std::int32_t x, y, width, height;
};
// Exclusive upper bounds can represent INT32_MAX + 1.
struct CandidateDomain {
  std::int64_t left, top, right, bottom;
};
struct PhaseExtent {
  std::int32_t width, height;
};
struct SamplingPlaneGeometry {
  std::int32_t pad_x, pad_y;
  PhaseExtent current;
  std::array<PhaseExtent, 16> reference{};
};
// Built from the logical Super level before any frame is requested. Domains
// must preserve the actual defined phase extents, including quarter edges.
struct SamplingGeometry {
  std::int32_t pel = 1, ratio_x = 1, ratio_y = 1;
  bool chroma = false;
  std::array<SamplingPlaneGeometry, 3> planes{};
};
template <class T>
struct SamplingFrames {
  std::array<span2d::Plane<const T>, 3> current{};
  std::array<std::array<span2d::Plane<const T>, 16>, 3> reference{};
};
struct BlockError {
  std::int64_t luma, chroma, raw;
};

namespace sampling_detail {
inline std::int64_t floor_div(std::int64_t value, int divisor) {
  return value / divisor - (value % divisor < 0);
}
inline void unsupported() {
  throw std::invalid_argument("unsupported sampling geometry");
}
inline bool fits(std::int64_t start, std::int64_t size, std::int32_t extent) {
  return start >= 0 && start + size <= extent;
}
inline void validate(const SamplingGeometry& g, BlockRegion b) {
  if ((g.pel != 1 && g.pel != 2 && g.pel != 4) || (g.ratio_x != 1 && g.ratio_x != 2) ||
      (g.ratio_y != 1 && g.ratio_y != 2) || b.x < 0 || b.y < 0 || b.width <= 0 || b.height <= 0 || b.x % g.ratio_x ||
      b.width % g.ratio_x || b.y % g.ratio_y || b.height % g.ratio_y)
    unsupported();
  for (int k = 0; k < (g.chroma ? 3 : 1); ++k) {
    const auto& plane = g.planes[k];
    const int rx = k == 0 ? 1 : g.ratio_x, ry = k == 0 ? 1 : g.ratio_y;
    if (plane.pad_x < 0 || plane.pad_y < 0 || plane.current.width <= 0 || plane.current.height <= 0 ||
        !fits(std::int64_t(plane.pad_x) + b.x / rx, b.width / rx, plane.current.width) ||
        !fits(std::int64_t(plane.pad_y) + b.y / ry, b.height / ry, plane.current.height))
      unsupported();
    for (int a = 0; a < g.pel * g.pel; ++a)
      if (plane.reference[a].width <= 0 || plane.reference[a].height <= 0)
        unsupported();
  }
}

struct OffsetRange {
  std::int64_t first, last;
};
inline OffsetRange offsets(std::int64_t low, std::int64_t high, int ratio, int pel, int phase) {
  // Truncation by a positive ratio is monotone and hits every integer between
  // its endpoints. For this phase, retain exactly t = pel*q + phase.
  const auto first_t = low / ratio, last_t = (high - 1) / ratio;
  return {-floor_div(-(first_t - phase), pel), floor_div(last_t - phase, pel)};
}

inline void domain(const SamplingGeometry& g, BlockRegion b, CandidateDomain omega) {
  if (omega.left < INT32_MIN || omega.top < INT32_MIN || omega.right > std::int64_t(INT32_MAX) + 1 ||
      omega.bottom > std::int64_t(INT32_MAX) + 1 || omega.left >= omega.right || omega.top >= omega.bottom)
    unsupported();
  for (int k = 0; k < (g.chroma ? 3 : 1); ++k) {
    const auto& plane = g.planes[k];
    const int rx = k == 0 ? 1 : g.ratio_x, ry = k == 0 ? 1 : g.ratio_y;
    const auto x = std::int64_t(plane.pad_x) + b.x / rx;
    const auto y = std::int64_t(plane.pad_y) + b.y / ry;
    for (int ay = 0; ay < g.pel; ++ay) {
      const auto dy = offsets(omega.top, omega.bottom, ry, g.pel, ay);
      if (dy.first > dy.last)
        continue;
      for (int ax = 0; ax < g.pel; ++ax) {
        const auto dx = offsets(omega.left, omega.right, rx, g.pel, ax);
        if (dx.first > dx.last)
          continue;
        const auto extent = plane.reference[ay * g.pel + ax];
        if (x + dx.first < 0 || y + dy.first < 0 || !fits(x + dx.last, b.width / rx, extent.width) ||
            !fits(y + dy.last, b.height / ry, extent.height))
          unsupported();
      }
    }
  }
}
inline CandidateDomain singleton(MotionVector v) {
  return {v.x, v.y, std::int64_t(v.x) + 1, std::int64_t(v.y) + 1};
}
} // namespace sampling_detail

// Call for every block/layer at creation, including every separately admitted
// seed (e.g. Analyse's three possible field-zero seeds). No pixel access and
// no candidate enumeration: at most 16 phase-bound checks per enabled plane.
inline void validate_sampling_domain(const SamplingGeometry& geometry, BlockRegion block, CandidateDomain omega,
                                     std::initializer_list<MotionVector> extra_seeds = {}) {
  sampling_detail::validate(geometry, block);
  sampling_detail::domain(geometry, block, omega);
  for (auto seed : extra_seeds)
    sampling_detail::domain(geometry, block, sampling_detail::singleton(seed));
}

template <class T>
void validate_sampling_frames(const SamplingGeometry& g, const SamplingFrames<T>& frames) {
  if (g.pel != 1 && g.pel != 2 && g.pel != 4)
    throw std::invalid_argument("invalid frame phase count");
  const int count = g.chroma ? 3 : 1;
  // All enabled frame views must conform before any metric reads a pixel.
  const auto conform = [](auto view, PhaseExtent extent) {
    validate_plane(view);
    if (view.width() != extent.width || view.height() != extent.height)
      throw std::invalid_argument("frame does not conform to sampling geometry");
  };
  for (int k = 0; k < count; ++k) {
    conform(frames.current[k], g.planes[k].current);
    for (int a = 0; a < g.pel * g.pel; ++a)
      conform(frames.reference[k][a], g.planes[k].reference[a]);
  }
}

// Direct evaluation rejects an unsafe vector before any sample read. Callers
// must additionally admit the entire Omega at creation.
template <class T>
BlockError block_error(const SamplingGeometry& g, BlockRegion b, const SamplingFrames<T>& frames, MotionVector vector,
                       BlockMetric metric) {
  validate_sampling_domain(g, b, sampling_detail::singleton(vector));
  if ((metric != BlockMetric::sad && metric != BlockMetric::satd) ||
      (metric == BlockMetric::satd && (b.width % 4 || b.height % 4)))
    throw std::invalid_argument("invalid block error metric or SATD dimensions");
  validate_sampling_frames(g, frames);
  const int count = g.chroma ? 3 : 1;
  std::array<std::int64_t, 3> errors{};
  for (int k = 0; k < count; ++k) {
    const int rx = k == 0 ? 1 : g.ratio_x, ry = k == 0 ? 1 : g.ratio_y;
    const auto tx = std::int64_t(vector.x) / rx, ty = std::int64_t(vector.y) / ry;
    const auto qx = sampling_detail::floor_div(tx, g.pel), qy = sampling_detail::floor_div(ty, g.pel);
    const auto ax = tx - g.pel * qx, ay = ty - g.pel * qy;
    const int x = static_cast<int>(std::int64_t(g.planes[k].pad_x) + b.x / rx);
    const int y = static_cast<int>(std::int64_t(g.planes[k].pad_y) + b.y / ry);
    const auto source = frames.current[k].subplane(x, y, b.width / rx, b.height / ry);
    const auto reference = frames.reference[k][static_cast<std::size_t>(ay * g.pel + ax)].subplane(
        static_cast<int>(x + qx), static_cast<int>(y + qy), b.width / rx, b.height / ry);
    errors[k] = block_metric(source, reference, k == 0 ? metric : BlockMetric::sad);
  }
  const auto chroma = metric_detail::accumulate(errors[1], errors[2]);
  return {errors[0], chroma, metric_detail::accumulate(errors[0], chroma)};
}

} // namespace neo_mv
