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
} // namespace
int main() {
  try {
    planning();
    pixels();
    selection_and_expansion();
    std::cout << "Scalar Analyse checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
