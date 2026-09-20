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
inline bool equal(MotionVector a, MotionVector b) {
  return a.x == b.x && a.y == b.y;
}
template <class Visit>
void ring(MotionVector center, std::int64_t r, int step, Visit&& visit) {
  for (auto i = -r + step; i < r; i += step) {
    visit(std::int64_t(center.x) + i, std::int64_t(center.y) - r);
    visit(std::int64_t(center.x) + i, std::int64_t(center.y) + r);
  }
  for (auto j = -r + step; j < r; j += step) {
    visit(std::int64_t(center.x) - r, std::int64_t(center.y) + j);
    visit(std::int64_t(center.x) + r, std::int64_t(center.y) + j);
  }
  for (auto x : {-r, r})
    for (auto y : {-r, r})
      visit(std::int64_t(center.x) + x, std::int64_t(center.y) + y);
}

template <class Evaluate>
SearchResult block(MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega,
                   int layer, int pel, std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls,
                   Evaluate&& evaluate) {
  std::array<SearchResult, 7> seeds{};
  std::size_t count = 0;
  const auto seed = [&](MotionVector v, int kind) {
    const auto e = evaluate(v);
    auto cost = candidate_cost(v, predictor.vector, 0, 0, e); // validates evaluator's raw error
    if (kind < 2) {
      const int penalty = kind == 0 || !controls.globalmv ? controls.pzero : controls.pglobal;
      cost = metric_detail::accumulate(cost, search_detail::penalty(e.raw, penalty));
    } else if (kind > 2)
      cost = candidate_cost(v, predictor.vector, lambda, 0, e);
    seeds[count++] = {v, cost, e.raw};
  };
  seed(zero, 0);
  seed(spatial.global, 1);
  seed(predictor.vector, 2);
  for (const auto& p : spatial.p) {
    bool duplicate = false;
    for (std::size_t i = 0; i < count; ++i)
      duplicate = duplicate || equal(p.vector, seeds[i].vector);
    if (!duplicate)
      seed(p.vector, 3);
  }
  const int type = layer == 0 || controls.search >= 4 ? controls.search : 1;
  const int range = layer == 0 ? controls.pelsearch : std::max(1, controls.searchparam);
  const SearchParams search{predictor.vector, lambda, controls.pnew, omega, type, range, {}};
  SearchResult best = seeds[0];
  if (controls.trymany == 2 || (controls.trymany == 1 && layer > 0)) {
    best = refine_motion(seeds[0], search, evaluate);
    for (std::size_t i = 1; i < count; ++i) {
      const auto candidate = refine_motion(seeds[i], search, evaluate);
      if (candidate.cost < best.cost)
        best = candidate;
    }
  } else {
    for (std::size_t i = 1; i < count; ++i)
      if (seeds[i].cost < best.cost)
        best = seeds[i];
    best = refine_motion(best, search, evaluate);
  }
  const auto ordinary_raw = best.raw;
  if (ordinary_raw <= bad_threshold)
    return best;
  const auto consider = [&](std::int64_t x, std::int64_t y) {
    if (x < omega.left || x >= omega.right || y < omega.top || y >= omega.bottom)
      return;
    const MotionVector v{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
    const auto e = evaluate(v);
    const auto cost = candidate_cost(v, predictor.vector, lambda, controls.pnew, e);
    if (cost < best.cost)
      best = {v, cost, e.raw};
  };
  if (controls.badrange > 0) {
    auto expansion = search;
    expansion.type = 3;
    expansion.range = prediction_detail::coordinate(std::int64_t(controls.badrange) * pel);
    expansion.cross_center = MotionVector{0, 0};
    best = refine_motion(best, expansion, evaluate);
  } else if (controls.badrange < 0) {
    const auto limit = -std::int64_t(controls.badrange) * pel;
    for (std::int64_t r = 1; r < limit; r += pel) {
      ring({0, 0}, r, pel, consider);
      if (best.raw < ordinary_raw / 4)
        break;
    }
  }
  const auto center = best.vector;
  for (int r = 1; r < pel; ++r)
    ring(center, r, 1, consider);
  return best;
}
} // namespace analyse_detail

template <class T>
MotionGrid analyse_vectors(AnalysisMetadata finest, const std::vector<SamplingGeometry>& geometries,
                           const std::vector<SamplingFrames<T>>& frames, AnalyseControls controls = {},
                           int field_shift = 0) {
  const auto layers = plan_analysis(finest, geometries, controls);
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
    if (!coarsest)
      for (int y = 0; y < m.blocks_y; ++y)
        for (int x = 0; x < m.blocks_x; ++x)
          current.values[std::size_t(y) * m.blocks_x + x] = interpolate_predictor(
              parent, x, y, {m.block_width, m.block_height, m.overlap_x, m.overlap_y, parent_pel, m.pel});
    const auto global =
        enter_global_level(coarsest ? MotionVector{0, 0} : global_predictor(parent, controls.globalmv), m.pel, f);
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
        const auto evaluate = [&](MotionVector v) {
          return block_error(layer.sampling, block, frames[index], v,
                             controls.satd ? BlockMetric::satd : BlockMetric::sad);
        };
        const auto result = analyse_detail::block(u, spatial, {0, f}, omega, static_cast<int>(index), m.pel, lambda,
                                                  badsad, controls, evaluate);
        current.values[std::size_t(y) * m.blocks_x + x] = {result.vector, result.raw};
      }
    }
    parent = std::move(current);
    parent_pel = m.pel;
  }
  return parent;
}

} // namespace neo_mv
