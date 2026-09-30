#include "core/motion/analyse.hpp"
#include "motion_fixture.hpp"
#include "core/interpolation/dense.hpp"

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

template <class T>
void staggered_translation() {
  MotionFixture<T> f(48, 32, 8, 8, 8, 1, true);
  const auto noise = [](int x, int y) {
    return ((x * 31 + y * 17 + x * y * 7) & 127);
  };
  for (int k = 0; k < 3; ++k) {
    const auto e = f.geometry.planes[k].current;
    for (int y = 0; y < e.height; ++y)
      for (int x = 0; x < e.width; ++x) {
        const auto i = std::size_t(y) * (e.width + 1) + x;
        const double scale = std::is_same_v<T, float> ? 1.0 / 255 : 1.0;
        f.source[k][i] = T(noise(x, y) * scale);
        f.reference[k][0][i] = T(noise(x - (k ? 1 : 2), y) * scale);
      }
  }
  AnalyseControls c;
  c.levels = 1;
  c.search = 3;
  c.searchparam = 4;
  c.mvlambda = c.pnew = c.pzero = c.pglobal = 0;
  c.badsad = 0;
  c.badrange = 4;
  for (int layout : {1, 2})
    for (bool meander : {false, true}) {
      c.layout = layout;
      c.meander = meander;
      const auto layers = plan_analysis(f.metadata, {f.geometry}, c);
      const auto& m = layers[0].metadata;
      CHECK(analysis_block(m, 0, 1).x == 4 && analysis_block(m, 0, 1).y == 6);
      const auto result = analyse_vectors_planned<T>(layers, {f.frames}, c);
      for (const auto v : result.values)
        CHECK(v.vector.x == 2 && v.vector.y == 0 && v.error == 0);
      const auto encoded = encode_analysis_field({m, FieldState::complete, result});
      const auto decoded = read_analysis_field([&](const std::string& key) -> IntegerPropertyView {
        const auto it = encoded.find(key);
        if (it == encoded.end())
          return {};
        return {true, it->second.size(), it->second.data()};
      });
      CHECK(decoded.state == FieldState::complete && decoded.metadata.layout == layout);
      auto forward = m;
      forward.delta = -m.delta;
      const auto dense = DenseInterpolationPlan<>(m, forward, 1, 1).generate(result, result, 128);
      for (auto x : dense.B.x)
        CHECK(x == 2);
      for (auto mask : dense.mB)
        CHECK(mask == 0);
      const auto uv = DenseFlowPlan<>(m, 2, 2).generate(result, 0);
      for (auto x : uv.x)
        CHECK(x == 1);
    }
}
void staggered_geometry() {
  MotionFixture<std::uint8_t> f(64, 48, 8, 8, 8), coarse(32, 24, 8, 8, 8);
  AnalyseControls c;
  c.layout = 2;
  c.levels = 2;
  const auto layers = plan_analysis(f.metadata, {f.geometry, coarse.geometry}, c);
  CHECK(layers.size() == 2);
  const auto grid = analyse_vectors_planned<std::uint8_t>(layers, {f.frames, coarse.frames}, c);
  for (auto v : grid.values)
    CHECK(v.vector.x == 0 && v.vector.y == 0 && v.error == 0);
  const auto& m = layers[0].metadata;
  // A linear physical-coordinate field must survive all triangle orientations.
  for (double y = 5; y < 35; y += 0.5)
    for (double x = 13; x < 45; x += 0.5) {
      const auto t = staggered::triangle(m, x, y);
      const auto value = t.sample([&](std::size_t i) {
        const auto b = analysis_block(m, int(i % m.blocks_x), int(i / m.blocks_x));
        return 2.0 * (b.x + 4) + 3.0 * (b.y + 4);
      });
      CHECK(std::abs(value - (2 * x + 3 * y)) < 1e-10);
    }
  for (auto& plane : f.reference)
    for (auto& phase : plane)
      std::fill(phase.begin(), phase.end(), 10);
  const auto b = analysis_block(m, 0, 0);
  CHECK(staggered::error(f.geometry, b, f.frames, {0, 0}, true).raw == 640);
  // Corner differences are excluded by the hexagon but retained by rectangle.
  std::fill(f.reference[0][0].begin(), f.reference[0][0].end(), 0);
  f.reference[0][0][8 * (f.geometry.planes[0].current.width + 1) + 8] = 100;
  CHECK(staggered::error(f.geometry, b, f.frames, {0, 0}, true).raw == 0);
  CHECK(staggered::error(f.geometry, b, f.frames, {0, 0}, false).raw == 100);
  auto bad = c;
  bad.metric = MotionMetric::satd;
  rejects([&] { plan_analysis(f.metadata, {f.geometry}, bad); });
  bad = c;
  bad.fields = true;
  rejects([&] { plan_analysis(f.metadata, {f.geometry}, bad); });
  auto forward = m;
  forward.delta = -1;
  auto converging = grid;
  for (std::size_t i = 0; i < converging.values.size(); ++i) {
    const auto b = analysis_block(m, int(i % m.blocks_x), int(i / m.blocks_x));
    converging.values[i].vector = {-(b.x / 2), 0};
  }
  const auto d = DenseInterpolationPlan<>(m, forward, 1, 1).generate(converging, converging, 128);
  CHECK(d.mB[20 * 64 + 24] > 0);
  const auto endpoint = DenseInterpolationPlan<>(m, forward, 1, 1).generate(converging, converging, 0);
  for (auto v : endpoint.mF)
    CHECK(v == 0);
}

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
    staggered_translation<std::uint8_t>();
    staggered_translation<std::uint16_t>();
    staggered_translation<float>();
    staggered_geometry();
    planning();
    pixels();
    duplicate_initial_seeds();
    selection_and_expansion();
    bounded_expansion();
    std::cout << "Scalar Analyse checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
