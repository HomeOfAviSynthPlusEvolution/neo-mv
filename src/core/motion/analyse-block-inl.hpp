// Intentionally included in both the core and selected SIMD namespaces.
// The evaluator owns full-error evaluation and sequential refinement.
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
SearchResult block_impl(MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega,
                   int layer, int pel, std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls,
                   Evaluate&& evaluate) {
  std::array<SearchResult, 7> seeds{};
  std::array<BlockError, 3> initial_errors{};
  std::size_t count = 0;
  const auto seed = [&](MotionVector v, int kind) NEO_MV_MOTION_ATTR {
    // Equal initial vectors share samples, but retain their separate penalties
    // and positions in the seed order.
    const auto e = kind > 0 && kind < 3 && equal(v, seeds[0].vector) ? initial_errors[0]
                   : kind == 2 && equal(v, seeds[1].vector) ? initial_errors[1]
                                                          : evaluate(v);
    if (kind < 3)
      initial_errors[kind] = e;
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
    best = evaluate.refine(seeds[0], search);
    for (std::size_t i = 1; i < count; ++i) {
      const auto candidate = evaluate.refine(seeds[i], search);
      if (candidate.cost < best.cost)
        best = candidate;
    }
  } else {
    for (std::size_t i = 1; i < count; ++i)
      if (seeds[i].cost < best.cost)
        best = seeds[i];
    best = evaluate.refine(best, search);
  }
  const auto ordinary_raw = best.raw;
  if (ordinary_raw <= bad_threshold)
    return best;
  const auto consider = [&](std::int64_t x, std::int64_t y) NEO_MV_MOTION_ATTR {
    if (x < omega.left || x >= omega.right || y < omega.top || y >= omega.bottom)
      return;
    const MotionVector v{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
    const auto e = evaluate(v);
    const auto cost = candidate_cost(v, predictor.vector, lambda, controls.pnew, e);
    if (cost < best.cost)
      best = {v, cost, e.raw};
  };
  // Wide rings often reject poor candidates early. The final nearby subpixel
  // ring keeps full SAD to avoid prefix reductions on similar candidates.
  const auto consider_bounded = [&](std::int64_t x, std::int64_t y) NEO_MV_MOTION_ATTR {
    if (x < omega.left || x >= omega.right || y < omega.top || y >= omega.bottom)
      return;
    const MotionVector v{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
    evaluate.improve_expansion(v, search, best);
  };
  if (controls.badrange > 0) {
    auto expansion = search;
    expansion.type = 3;
    expansion.range = prediction_detail::coordinate(std::int64_t(controls.badrange) * pel);
    expansion.cross_center = MotionVector{0, 0};
    best = evaluate.refine(best, expansion);
  } else if (controls.badrange < 0) {
    const auto limit = -std::int64_t(controls.badrange) * pel;
    for (std::int64_t r = 1; r < limit; r += pel) {
      ring({0, 0}, r, pel, consider_bounded);
      if (best.raw < ordinary_raw / 4)
        break;
    }
  }
  const auto center = best.vector;
  for (int r = 1; r < pel; ++r)
    ring(center, r, 1, consider);
  return best;
}
