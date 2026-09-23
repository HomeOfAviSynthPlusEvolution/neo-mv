#pragma once

#include "core/motion/composition.hpp"

namespace neo_mv {

struct AnalyseControls {
  std::int32_t levels = 0, search = 2, searchparam = 2, pelsearch = 1;
  std::int32_t mvlambda = 1000, lsad = 400, plevel = 1;
  std::int32_t pnew = 25, pzero = 25, pglobal = 0, badsad = 10000, badrange = 24, trymany = 0;
  bool globalmv = true, meander = true, fields = false, satd = false;
};
struct AnalysisLayer {
  AnalysisMetadata metadata;
  SamplingGeometry sampling;
  std::int32_t bound_pad_x, bound_pad_y;
};

inline void validate_analyse_controls(AnalyseControls c, int block_width, int block_height) {
  if (c.search < 0 || c.search > 5 || c.pelsearch <= 0 || c.mvlambda < 0 || c.plevel < 0 || c.plevel > 2 ||
      c.pnew < 0 || c.pnew > 256 || c.pzero < 0 || c.pzero > 256 || c.pglobal < 0 || c.pglobal > 256 || c.trymany < 0 ||
      c.trymany > 2 || (c.satd && (block_width % 4 || block_height % 4)))
    throw std::invalid_argument("invalid Analyse controls");
}

// Supply logical Super geometries finest first, without reading frame samples.
// This planner is also the creation-time admission path for a future host.
inline std::vector<AnalysisLayer> plan_analysis(AnalysisMetadata finest, const std::vector<SamplingGeometry>& super,
                                                AnalyseControls controls = {}) {
  validate_analyse_controls(controls, finest.block_width, finest.block_height);
  if (!geometry_detail::block_pair(finest.block_width, finest.block_height) || finest.overlap_x < 0 ||
      finest.overlap_y < 0 || finest.overlap_x > finest.block_width / 2 || finest.overlap_y > finest.block_height / 2 ||
      super.empty() || super.size() > INT32_MAX || finest.delta == 0)
    throw std::invalid_argument("invalid analysis planning inputs");
  const int sx = finest.block_width - finest.overlap_x, sy = finest.block_height - finest.overlap_y;
  finest.blocks_x = geometry_detail::dimension((std::int64_t(finest.real_width) - finest.overlap_x + sx - 1) / sx);
  finest.blocks_y = geometry_detail::dimension((std::int64_t(finest.real_height) - finest.overlap_y + sy - 1) / sy);
  finest.levels = 1;
  if (!valid_analysis_metadata(finest))
    throw std::invalid_argument("invalid finest analysis geometry");
  int reductions = 0;
  auto w = finest.width, h = finest.height;
  for (;;) {
    w = geometry_detail::reduced(w, finest.ratio_x, finest.pad_x);
    h = geometry_detail::reduced(h, finest.ratio_y, finest.pad_y);
    if (w < finest.block_width || h < finest.block_height)
      break;
    ++reductions;
  }
  const auto requested =
      controls.levels > 0 ? std::int64_t(controls.levels) : std::int64_t(std::max(1, reductions)) + controls.levels;
  const auto levels = std::min<std::int64_t>(static_cast<std::int64_t>(super.size()), requested);
  if (levels < 1)
    throw std::invalid_argument("no usable analysis levels");
  finest.levels = static_cast<std::int32_t>(levels);
  std::vector<AnalysisLayer> result;
  auto covered_x = std::int64_t(finest.blocks_x) * sx + finest.overlap_x;
  auto covered_y = std::int64_t(finest.blocks_y) * sy + finest.overlap_y;
  int hp = finest.pad_x, vp = finest.pad_y;
  for (int l = 0; l < levels; ++l) {
    auto m = finest;
    const auto& g = super[l];
    if (g.planes[0].pad_x != finest.pad_x || g.planes[0].pad_y != finest.pad_y)
      throw std::invalid_argument("Super level padding changed");
    m.pel = l == 0 ? finest.pel : 1;
    m.width = geometry_detail::dimension(std::int64_t(g.planes[0].current.width) - 2LL * m.pad_x);
    m.height = geometry_detail::dimension(std::int64_t(g.planes[0].current.height) - 2LL * m.pad_y);
    if (l == 0 && (m.width != finest.width || m.height != finest.height))
      throw std::invalid_argument("finest Super working extent mismatch");
    if (l > 0) {
      m.real_width = m.width;
      m.real_height = m.height;
    }
    m.blocks_x = geometry_detail::dimension((covered_x - m.overlap_x) / sx);
    m.blocks_y = geometry_detail::dimension((covered_y - m.overlap_y) / sy);
    if (l == 0 && controls.fields && m.pel > 1 && m.delta % 2 != 0)
      validate_motion_layer(m, g, true, {{0, 0}, {0, -m.pel / 2}, {0, m.pel / 2}}, hp, vp);
    else
      validate_motion_layer(m, g, l == 0, {{0, 0}}, hp, vp);
    result.push_back({m, g, hp, vp});
    covered_x /= 2;
    covered_y /= 2;
    hp /= 2;
    vp /= 2;
  }
  return result;
}

inline std::int64_t adaptive_lambda(std::int64_t base, std::int64_t lsad, std::int64_t confidence) {
  if (base < 0 || confidence < 0)
    throw std::invalid_argument("invalid adaptive lambda inputs");
  const auto denominator = std::max<std::int64_t>(prediction_detail::add(lsad, confidence / 2), 1);
  const double t = double(lsad) / double(denominator);
  return prediction_detail::truncate((double(base) * t) * t);
}

namespace analyse_detail {
#define NEO_MV_MOTION_ATTR
#include "core/motion/analyse-block-inl.hpp"
#undef NEO_MV_MOTION_ATTR

template <class Evaluate>
SearchResult block(MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega,
                   int layer, int pel, std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls,
                   Evaluate&& evaluate) {
  search_detail::SearchEvaluator<std::remove_reference_t<Evaluate>> execution{evaluate};
  return block_impl(predictor, spatial, zero, omega, layer, pel, lambda, bad_threshold, controls, execution);
}

template <class Evaluate>
auto execute_block(MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega,
                   int layer, int pel, std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls,
                   Evaluate& evaluate, int)
    -> decltype(evaluate.analyse(predictor, spatial, zero, omega, layer, pel, lambda, bad_threshold, controls)) {
  return evaluate.analyse(predictor, spatial, zero, omega, layer, pel, lambda, bad_threshold, controls);
}
template <class Evaluate>
SearchResult execute_block(MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero,
                           CandidateDomain omega, int layer, int pel, std::int64_t lambda,
                           std::int64_t bad_threshold, AnalyseControls controls, Evaluate& evaluate, long) {
  return block(predictor, spatial, zero, omega, layer, pel, lambda, bad_threshold, controls, evaluate);
}
} // namespace analyse_detail

// Internal path: layers must be the unchanged result of plan_analysis for
// these controls. Borrowed frame storage and per-request field shifts remain
// validated below; callers may reuse the immutable creation-time geometry.
template <class T, class Kernels = ScalarKernels<T>>
MotionGrid analyse_vectors_planned(const std::vector<AnalysisLayer>& layers,
                                   const std::vector<SamplingFrames<T>>& frames, AnalyseControls controls,
                                   int field_shift = 0) {
  if (layers.empty())
    throw std::invalid_argument("missing analysis plan layers");
  const auto& finest = layers.front().metadata;
  validate_analysis_precision<T>(finest.bits);
  if (frames.size() < layers.size())
    throw std::invalid_argument("missing analysis frame levels");
  const bool field = controls.fields && finest.pel > 1 && finest.delta % 2 != 0;
  if (field_shift != 0 && (!field || (field_shift != -finest.pel / 2 && field_shift != finest.pel / 2)))
    throw std::invalid_argument("invalid finest field shift");
  for (std::size_t l = 0; l < layers.size(); ++l)
    validate_sampling_frames(layers[l].sampling, frames[l]);
  const auto lambda0 =
      scale_precision(scale_area(controls.mvlambda, finest.block_width, finest.block_height), finest.bits);
  const auto lsad = scale_area(scale_precision(controls.lsad, finest.bits), finest.block_width, finest.block_height);
  const auto badsad =
      scale_area(scale_precision(controls.badsad, finest.bits), finest.block_width, finest.block_height);
  MotionGrid parent{0, 0, {}};
  int parent_pel = 1;
  for (std::size_t index = layers.size(); index-- > 0;) {
    const auto& layer = layers[index];
    const auto& m = layer.metadata;
    const bool coarsest = index + 1 == layers.size();
    const int f = index == 0 ? field_shift : 0;
    MotionGrid current{m.blocks_x, m.blocks_y, {}};
    current.values.resize(static_cast<std::size_t>(field_detail::count(m)));
    if (!coarsest) {
      const PredictionInterpolationPlan interpolation(
          parent, {m.block_width, m.block_height, m.overlap_x, m.overlap_y, parent_pel, m.pel});
      for (int y = 0; y < m.blocks_y; ++y)
        for (int x = 0; x < m.blocks_x; ++x)
          current.values[std::size_t(y) * m.blocks_x + x] = interpolation(x, y);
    }
    const auto global =
        enter_global_level(coarsest ? MotionVector{0, 0} : global_predictor(parent, controls.globalmv), m.pel, f);
    decltype(auto) prepared_frames = Kernels::prepare_frames(layer.sampling, frames[index]);
    for (int y = 0; y < m.blocks_y; ++y) {
      const int direction = controls.meander && y % 2 ? -1 : 1;
      std::int64_t base = 0;
      if (y != 0) {
        base = lambda0 / (m.pel * m.pel);
        for (std::size_t exponent = 0; exponent < index * controls.plevel; ++exponent)
          base = prediction_detail::weight(base, 2);
      }
      for (int i = 0; i < m.blocks_x; ++i) {
        const int x = direction == 1 ? i : m.blocks_x - 1 - i;
        const auto block = analysis_block(m, x, y);
        const auto omega = analysis_domain(m, block, layer.bound_pad_x, layer.bound_pad_y);
        const auto spatial = spatial_predictors(current, x, y, direction, f, global, omega);
        auto u = current.values[std::size_t(y) * m.blocks_x + x];
        u.vector = prediction_detail::clamp(u.vector, omega);
        if (coarsest)
          u = spatial.p[0];
        const auto lambda = adaptive_lambda(base, lsad, u.error);
        auto evaluate = Kernels::prepare_block_error(layer.sampling, block, prepared_frames,
                                                     controls.satd ? BlockMetric::satd : BlockMetric::sad);
        const auto result = analyse_detail::execute_block(u, spatial, {0, f}, omega, static_cast<int>(index), m.pel, lambda,
                                                  badsad, controls, evaluate, 0);
        current.values[std::size_t(y) * m.blocks_x + x] = {result.vector, result.raw};
      }
    }
    parent = std::move(current);
    parent_pel = m.pel;
  }
  return parent;
}

template <class T, class Kernels = ScalarKernels<T>>
MotionGrid analyse_vectors(AnalysisMetadata finest, const std::vector<SamplingGeometry>& geometries,
                           const std::vector<SamplingFrames<T>>& frames, AnalyseControls controls = {},
                           int field_shift = 0) {
  return analyse_vectors_planned<T, Kernels>(plan_analysis(finest, geometries, controls), frames, controls,
                                             field_shift);
}

} // namespace neo_mv
