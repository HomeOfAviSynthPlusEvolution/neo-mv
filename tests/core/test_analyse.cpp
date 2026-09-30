#include "core/motion/analyse.hpp"
#include "motion_fixture.hpp"

#include <iostream>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("analyse assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
using namespace neo_mv;

void planning() {
  MotionFixture<std::uint8_t> a(64, 64, 4, 4), b(32, 32, 4, 4), c(16, 16, 4, 4);
  AnalyseControls controls;
  const int requested[] = {0, -1, -2, 1}, expected[] = {3, 3, 2, 1};
  for (int i = 0; i < 4; ++i) {
    controls.levels = requested[i];
    CHECK(plan_analysis(a.metadata, {a.geometry, b.geometry, c.geometry}, controls).size() == std::size_t(expected[i]));
  }
  MotionFixture<std::uint8_t> overlap(20, 12, 8, 8);
  overlap.metadata.real_width = 18;
  overlap.metadata.real_height = 10;
  overlap.metadata.overlap_x = overlap.metadata.overlap_y = 4;
  controls.levels = 1;
  const auto layers = plan_analysis(overlap.metadata, {overlap.geometry}, controls);
  CHECK(layers[0].metadata.blocks_x == 4 && layers[0].metadata.blocks_y == 2);
  CHECK(adaptive_lambda(100, 100, 200) == 25);
  CHECK(adaptive_lambda(100, -1, 0) == 100);
  CHECK(adaptive_lambda(0, -1, 200) == 0);

  // Super reduces chroma independently, retaining its physical padding.
  MotionFixture<std::uint8_t> fine(20, 20, 4, 4, 3, 1, true), coarse(10, 10, 4, 4, 3, 1, true);
  for (int k = 1; k < 3; ++k) {
    coarse.geometry.planes[k].current = coarse.geometry.planes[k].reference[0] = {6, 6};
    coarse.frames.current[k] = coarse.frames.current[k].subplane(0, 0, 6, 6);
    coarse.frames.reference[k][0] = coarse.frames.reference[k][0].subplane(0, 0, 6, 6);
  }
  controls.levels = 2;
  const auto chroma = plan_analysis(fine.metadata, {fine.geometry, coarse.geometry}, controls);
  CHECK(chroma[1].bound_pad_x == 1 && chroma[1].sampling.planes[0].pad_x == 3);
  const auto grid = analyse_vectors<std::uint8_t>(fine.metadata, {fine.geometry, coarse.geometry},
                                                  {fine.frames, coarse.frames}, controls);
  for (auto v : grid.values)
    CHECK(v.vector.x == 0 && v.vector.y == 0 && v.error == 0);
  controls.levels = -100;
  rejects([&] { plan_analysis(a.metadata, {a.geometry}, controls); });
}

void pixels() {
  MotionFixture<std::uint8_t> f(4, 4, 4, 4);
  const int current[] = {10, 20, 30, 30}, reference[] = {0, 10, 20, 30};
  const auto extent = f.geometry.planes[0].current;
  for (int y = 0; y < extent.height; ++y)
    for (int x = 0; x < extent.width; ++x) {
      const auto i = std::size_t(y) * (extent.width + 1) + x;
      f.source[0][i] = static_cast<std::uint8_t>(current[std::clamp(x - 4, 0, 3)]);
      f.reference[0][0][i] = static_cast<std::uint8_t>(reference[std::clamp(x - 4, 0, 3)]);
    }
  AnalyseControls controls;
  controls.search = 4;
  controls.pnew = controls.pzero = 0;
  auto result = analyse_vectors<std::uint8_t>(f.metadata, {f.geometry}, {f.frames}, controls);
  CHECK(result.values[0].vector.x == 1 && result.values[0].vector.y == 0 && result.values[0].error == 0);
  MotionFixture<std::uint8_t> yuv(4, 4, 4, 4, 4, 1, true, 1);
  for (auto& plane : yuv.reference)
    std::fill(plane[0].begin(), plane[0].end(), 255);
  controls.trymany = 2;
  controls.badsad = INT32_MAX;
  result = analyse_vectors<std::uint8_t>(yuv.metadata, {yuv.geometry}, {yuv.frames}, controls);
  CHECK(result.values[0].vector.x == 0 && result.values[0].vector.y == 0 && result.values[0].error == 12240);

  MotionFixture<float> fine(16, 16, 4, 4, 4, 2), coarse(8, 8, 4, 4);
  controls.levels = 2;
  controls.fields = true;
  const auto field = analyse_vectors<float>(fine.metadata, {fine.geometry, coarse.geometry},
                                            {fine.frames, coarse.frames}, controls, 1);
  for (auto v : field.values)
    CHECK(v.vector.x == 0 && v.vector.y == 1 && v.error == 0);
  auto invalid = fine.frames;
  invalid.reference[0][3] = {};
  rejects([&] {
    analyse_vectors<float>(fine.metadata, {fine.geometry, coarse.geometry}, {invalid, coarse.frames}, controls);
  });
  rejects([&] {
    analyse_vectors<float>(fine.metadata, {fine.geometry, coarse.geometry}, {fine.frames, coarse.frames}, controls, 2);
  });
}

void duplicate_initial_seeds() {
  AnalyseControls c;
  c.search = 4;
  c.pzero = 256;
  c.pglobal = 128;
  // A singleton domain suppresses refinement while retaining all seed scores.
  SpatialPredictors spatial{};
  int calls = 0;
  const auto evaluate = [&](MotionVector) {
    ++calls;
    return BlockError{60, 40, 100};
  };
  const auto best = analyse_detail::block({}, spatial, {}, {0, 0, 1, 1}, 0, 1, 0, INT64_MAX, c, evaluate);
  CHECK(calls == 1);
  CHECK(best.cost == 100 && best.raw == 100 && best.vector.x == 0 && best.vector.y == 0);
  spatial.global = {1, 0};
  for (auto& p : spatial.p)
    p.vector = {1, 0};
  calls = 0;
  const auto distinct = analyse_detail::block({{1, 0}, 0}, spatial, {}, {1, 0, 2, 1}, 0, 1, 0,
                                             INT64_MAX, c, evaluate);
  CHECK(calls == 2);
  CHECK(distinct.cost == 100 && distinct.raw == 100 && distinct.vector.x == 1 && distinct.vector.y == 0);
}

void selection_and_expansion() {
  SpatialPredictors spatial{};
  spatial.global = {10, 0};
  const CandidateDomain omega{-40, -40, 41, 41};
  AnalyseControls c;
  c.search = 4;
  c.pnew = c.pzero = c.pglobal = 0;
  c.trymany = 2;
  const auto evaluate = [](MotionVector v) {
    const std::int64_t raw = v.y == 0 && (v.x == 11 || v.x == 21)               ? 5000
                             : v.y == 0 && (v.x == 0 || v.x == 10 || v.x == 20) ? 6000
                                                                                : 7000;
    return BlockError{raw, 0, raw};
  };
  auto best = analyse_detail::block({{20, 0}, 0}, spatial, {0, 0}, omega, 0, 1, 0, INT64_MAX, c, evaluate);
  CHECK(best.vector.x == 11 && best.raw == 5000);
  c.trymany = 0;
  best = analyse_detail::block({{20, 0}, 0}, spatial, {0, 0}, omega, 0, 1, 0, INT64_MAX, c, evaluate);
  CHECK(best.vector.x == 0 && best.raw == 6000);
  spatial = {};
  c.badrange = 1;
  const auto distant = [](MotionVector v) {
    const std::int64_t e = v.x == -4 && v.y == 2 ? 0 : 100;
    return BlockError{e, 0, e};
  };
  best = analyse_detail::block({}, spatial, {0, 0}, omega, 0, 1, 0, 100, c, distant);
  CHECK(best.raw == 100);
  best = analyse_detail::block({}, spatial, {0, 0}, omega, 0, 1, 0, 99, c, distant);
  CHECK(best.raw == 0 && best.vector.x == -4 && best.vector.y == 2);
  c.badrange = 0;
  c.search = 4; // Horizontal ordinary search cannot find either expansion target.
  const auto near = [](MotionVector v) {
    const std::int64_t e = v.x == 0 && v.y == -1 ? 1 : 100;
    return BlockError{e, 0, e};
  };
  best = analyse_detail::block({}, spatial, {0, 0}, omega, 0, 2, 0, 99, c, near);
  CHECK(best.raw == 1 && best.vector.y == -1);
  std::vector<std::pair<std::int64_t, std::int64_t>> visits;
  analyse_detail::ring({0, 0}, 1, 2, [&](auto x, auto y) { visits.emplace_back(x, y); });
  const std::vector<std::pair<std::int64_t, std::int64_t>> corners{{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  CHECK(visits == corners);
  c.badrange = -4;
  visits.clear();
  const auto stop = [&](MotionVector v) {
    visits.emplace_back(v.x, v.y);
    const std::int64_t e = v.x == -1 && v.y == -1 ? 20 : 100;
    return BlockError{e, 0, e};
  };
  best = analyse_detail::block({}, spatial, {0, 0}, omega, 0, 2, 0, 99, c, stop);
  CHECK(best.raw == 20);
  for (auto v : visits)
    CHECK(v.first > -3 && v.second > -3);
}

template <class T>
void parent_pixel_boundary() {
  for (int pel : {1, 2, 4}) {
    MotionFixture<T> fine(32, 16, 8, 8, 16, pel), coarse(16, 8, 8, 8, 16);
    const auto fill = [](auto& frame, int shift) {
      const auto e = frame.geometry.planes[0].current;
      const int stride = e.width + 1;
      const auto pattern = [](int x, int y) { return (x * 37 + y * 17 + x * y * 13) & 255; };
      for (int y = 0; y < e.height; ++y)
        for (int x = 0; x < e.width; ++x) {
          frame.source[0][std::size_t(y) * stride + x] = T(pattern(x + (x >= 24 ? shift : 0), y));
          for (int a = 0; a < frame.geometry.pel * frame.geometry.pel; ++a)
            frame.reference[0][a][std::size_t(y) * stride + x] = T(pattern(x, y));
        }
    };
    fill(coarse, 4);
    fill(fine, 8);
    AnalyseControls c;
    c.levels = 2;
    c.search = 4;
    c.searchparam = 4; // Coarse search reaches its exact four-pixel displacement.
    c.pelsearch = 1;   // Fine search cannot reach eight pixels from an averaged seed.
    c.mvlambda = c.pnew = c.pzero = c.pglobal = 0;
    c.badsad = INT32_MAX;
    const auto baseline = analyse_vectors<T>(fine.metadata, {fine.geometry, coarse.geometry},
                                            {fine.frames, coarse.frames}, c);
    c.parentpredict = true;
    const auto refined = analyse_vectors<T>(fine.metadata, {fine.geometry, coarse.geometry},
                                           {fine.frames, coarse.frames}, c);
    CHECK(baseline.values[1].error > 0);
    CHECK(refined.values[1].vector.x == 8 * pel && refined.values[1].vector.y == 0);
    CHECK(refined.values[1].error == 0);
    // A one-level analysis has no parent candidates, regardless of the option.
    c.levels = 1;
    const auto enabled = analyse_vectors<T>(fine.metadata, {fine.geometry}, {fine.frames}, c);
    c.parentpredict = false;
    const auto disabled = analyse_vectors<T>(fine.metadata, {fine.geometry}, {fine.frames}, c);
    for (std::size_t i = 0; i < enabled.values.size(); ++i) {
      CHECK(enabled.values[i].vector.x == disabled.values[i].vector.x);
      CHECK(enabled.values[i].vector.y == disabled.values[i].vector.y);
      CHECK(enabled.values[i].error == disabled.values[i].error);
    }
  }
}

void parent_seed_selection() {
  SpatialPredictors spatial{};
  spatial.has_parent = true;
  spatial.parent = {{{12, 0}, {12, 0}, {6, 0}, {0, 0}}};
  const CandidateDomain domain{-16, -4, 17, 5};
  AnalyseControls c;
  c.search = 4;
  c.pnew = c.pzero = c.pglobal = 0;
  // A narrow search around the interpolated midpoint cannot reach the
  // disjoint motion's true match. A parent seed makes that match reachable.
  const auto landscape = [](MotionVector v) {
    const std::int64_t raw = v.x == 12 && v.y == 0 ? 0 : v.x == 6 && v.y == 0 ? 50 : 100;
    return BlockError{raw, 0, raw};
  };
  auto best = analyse_detail::block({{6, 0}, 0}, spatial, {}, domain, 0, 1, 0, INT64_MAX, c, landscape);
  CHECK(best.vector.x == 6 && best.raw == 50);
  c.parentpredict = true;
  best = analyse_detail::block({{6, 0}, 0}, spatial, {}, domain, 0, 1, 0, INT64_MAX, c, landscape);
  CHECK(best.vector.x == 12 && best.raw == 0);
  // Parent candidates retain the original predictor's distance penalty.
  best = analyse_detail::block({{6, 0}, 0}, spatial, {}, domain, 0, 1, 512, INT64_MAX, c, landscape);
  CHECK(best.vector.x == 6 && best.raw == 50);
  const auto tied = [](MotionVector v) {
    const std::int64_t raw = (v.x == 6 || v.x == 12) && v.y == 0 ? 50 : 100;
    return BlockError{raw, 0, raw};
  };
  best = analyse_detail::block({{6, 0}, 0}, spatial, {}, domain, 0, 1, 0, INT64_MAX, c, tied);
  CHECK(best.vector.x == 6); // Original seed wins ties.

  struct Trace {
    int evaluations = 0, refinements = 0;
    BlockError operator()(MotionVector) { ++evaluations; return {100, 0, 100}; }
    SearchResult refine(SearchResult initial, const SearchParams&) { ++refinements; return initial; }
    bool improve_expansion(MotionVector, const SearchParams&, SearchResult&) { return false; }
  };
  for (int layer : {0, 1})
    for (int trymany : {0, 1, 2}) {
      c.trymany = trymany;
      Trace trace;
      analyse_detail::block_impl({{6, 0}, 0}, spatial, {}, domain, layer, 1, 0, INT64_MAX, c, trace);
      CHECK(trace.evaluations == 3); // Zero, midpoint, unique parent; no duplicate samples.
      CHECK(trace.refinements == ((trymany == 2 || (trymany == 1 && layer > 0)) ? 4 : 1));
    }
  spatial.has_parent = false; // The coarsest level has no parent stencil.
  Trace trace;
  analyse_detail::block_impl({{6, 0}, 0}, spatial, {}, domain, 0, 1, 0, INT64_MAX, c, trace);
  CHECK(trace.evaluations == 2 && trace.refinements == 3);
}

void bounded_expansion() {
  struct Evaluator {
    int full_corner = 0, bounded_corner = 0;
    static bool corner(MotionVector v) {
      return (v.x == -1 && v.y == -1) || (v.x == 1 && v.y == 1);
    }
    BlockError operator()(MotionVector v) {
      full_corner += corner(v);
      const std::int64_t raw = corner(v) ? 1 : 100;
      return {raw, 0, raw};
    }
    std::optional<BlockError> bounded(MotionVector v, std::int64_t limit, MotionVector, std::int64_t, int) {
      bounded_corner += corner(v);
      const std::int64_t raw = corner(v) ? 1 : 100;
      if (raw >= limit)
        return std::nullopt;
      return BlockError{raw, 0, raw};
    }
  } evaluate;
  AnalyseControls c;
  c.search = 4; // Ordinary horizontal search leaves these corners to expansion.
  c.pzero = c.pglobal = c.pnew = 0;
  c.badrange = -4;
  const auto best = analyse_detail::block({}, SpatialPredictors{}, {}, {-4, -4, 5, 5}, 0, 2, 0, 99, c, evaluate);
  CHECK(best.vector.x == -1 && best.vector.y == -1 && best.raw == 1 && best.cost == 1);
  CHECK(evaluate.full_corner == 0 && evaluate.bounded_corner > 0);
}
} // namespace
int main() {
  try {
    planning();
    pixels();
    duplicate_initial_seeds();
    selection_and_expansion();
    parent_pixel_boundary<std::uint8_t>();
    parent_pixel_boundary<std::uint16_t>();
    parent_pixel_boundary<float>();
    parent_seed_selection();
    bounded_expansion();
    std::cout << "Scalar Analyse checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
