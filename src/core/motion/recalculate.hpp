#pragma once

#include "core/motion/composition.hpp"

namespace neo_mv {

struct RecalculateControls {
  std::int32_t thsad = 200, mvlambda = 1000, search = 2, searchparam = 2, pnew = 25;
  bool smooth = true, satd = false, meander = true;
};

namespace recalculate_detail {
// Exact trunc(value*factor/divisor) without a 128-bit temporary. The mapping
// remainders obey 0 <= factor < divisor <= INT32_MAX.
inline std::int64_t fractional_product(std::int64_t value, std::int64_t factor, std::int64_t divisor) {
  return prediction_detail::add(prediction_detail::weight(value / divisor, factor),
                                prediction_detail::weight(value % divisor, factor) / divisor);
}
inline MotionVector map(const AnalysisField& old, const AnalysisMetadata& target, int bx, int by, CandidateDomain omega,
                        bool smooth) {
  if (old.state == FieldState::metadata_only)
    return prediction_detail::clamp({0, 0}, omega);
  const auto& m = old.metadata;
  const auto cx = target.block_width / 2 + std::int64_t(target.block_width - target.overlap_x) * bx;
  const auto cy = target.block_height / 2 + std::int64_t(target.block_height - target.overlap_y) * by;
  const auto sx = std::int64_t(m.block_width - m.overlap_x), sy = std::int64_t(m.block_height - m.overlap_y);
  const auto jx = (cx - m.block_width / 2) / sx, jy = (cy - m.block_height / 2) / sy;
  const auto dx = std::max(std::int64_t{0}, cx - (m.block_width / 2 + sx * jx));
  const auto dy = std::max(std::int64_t{0}, cy - (m.block_height / 2 + sy * jy));
  const auto at = [&](std::int64_t x, std::int64_t y) {
    const auto ix = static_cast<std::size_t>(std::clamp(x, std::int64_t{0}, std::int64_t(m.blocks_x) - 1));
    const auto iy = static_cast<std::size_t>(std::clamp(y, std::int64_t{0}, std::int64_t(m.blocks_y) - 1));
    return old.grid.values[iy * m.blocks_x + ix].vector;
  };
  const auto a = at(jx, jy), b = at(jx + 1, jy), c = at(jx, jy + 1), d = at(jx + 1, jy + 1);
  const auto component = [&](bool vertical) {
    const auto get = [&](MotionVector v) {
      return std::int64_t(vertical ? v.y : v.x);
    };
    std::int64_t z;
    if (!smooth)
      z = get(at(jx + (2 * dx >= sx), jy + (2 * dy >= sy)));
    else {
      const auto u = prediction_detail::add(get(a) * sx, dx * (get(b) - get(a)));
      const auto v = prediction_detail::add(get(c) * sx, dx * (get(d) - get(c)));
      z = prediction_detail::add(u, fractional_product(v - u, dy, sy)) / sx;
    }
    const auto scaled = sampling_detail::floor_div(z * target.pel, m.pel);
    return static_cast<std::int32_t>(
        std::clamp(scaled, vertical ? omega.top : omega.left, (vertical ? omega.bottom : omega.right) - 1));
  };
  return {component(false), component(true)};
}
} // namespace recalculate_detail

// target metadata describes the new Super, not the vector carrier. Call the
// geometry-only validate_motion_layer at plugin creation as well; this frame
// entry point rechecks it before any mapping or metric evaluation unless the
// caller retains the unchanged creation-time target and geometry.
template <class T, class Kernels = ScalarKernels<T>, bool GeometryValidated = false, bool InputValidated = false>
MotionGrid recalculate_vectors(const AnalysisField& old, const AnalysisMetadata& target,
                               const SamplingGeometry& geometry, const SamplingFrames<T>& frames,
                               RecalculateControls controls = {}) {
  if constexpr (!InputValidated)
    validate_owned_field(old);
  validate_analysis_precision<T>(target.bits);
  if (old.metadata.bits != target.bits || controls.mvlambda < 0 || controls.search < 0 || controls.search > 5 ||
      controls.pnew < 0 || controls.pnew > 256 ||
      (controls.satd && (target.block_width % 4 || target.block_height % 4)))
    throw std::invalid_argument("invalid Recalculate controls or input precision");
  if constexpr (!GeometryValidated)
    validate_motion_layer(target, geometry, true);
  validate_sampling_frames(geometry, frames);
  const auto lambda0 =
      scale_precision(scale_area(controls.mvlambda, target.block_width, target.block_height), target.bits);
  auto threshold = scale_area(scale_precision(controls.thsad, target.bits), target.block_width, target.block_height);
  if (target.chroma)
    threshold = prediction_detail::add(threshold, 2 * (threshold / (target.ratio_x * target.ratio_y)));
  MotionGrid output{target.blocks_x, target.blocks_y, {}};
  output.values.resize(static_cast<std::size_t>(field_detail::count(target)));
  for (int by = 0; by < target.blocks_y; ++by)
    for (int index = 0; index < target.blocks_x; ++index) {
      const int bx = controls.meander && by % 2 ? target.blocks_x - 1 - index : index;
      const auto block = analysis_block(target, bx, by);
      const auto omega = analysis_domain(target, block);
      const auto u = recalculate_detail::map(old, target, bx, by, omega, controls.smooth);
      const auto evaluate = [&](MotionVector vector) {
        return Kernels::block_error_validated(geometry, block, frames, vector,
                                              controls.satd ? BlockMetric::satd : BlockMetric::sad);
      };
      const auto error = evaluate(u);
      SearchResult result{u, error.raw, error.raw};
      if (error.raw > threshold) {
        const auto lambda = by == 0 ? 0 : lambda0 / (target.pel * target.pel);
        result = refine_motion(
            result, {u, lambda, controls.pnew, omega, controls.search, std::max(1, controls.searchparam), {}},
            evaluate);
      }
      output.values[std::size_t(by) * target.blocks_x + bx] = {result.vector, result.raw};
    }
  return output;
}

} // namespace neo_mv
