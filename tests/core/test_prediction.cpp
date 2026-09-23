#include "core/motion/prediction.hpp"

#include <iostream>
#include <string>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("prediction assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E, class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const E&) {
    caught = true;
  }
  CHECK(caught);
}
using namespace neo_mv;
void triple(MotionTriple v, int x, int y, std::int64_t error) {
  if (v.vector.x != x || v.vector.y != y || v.error != error)
    throw std::runtime_error("triple (" + std::to_string(v.vector.x) + "," + std::to_string(v.vector.y) + "," +
                             std::to_string(v.error) + ") expected (" + std::to_string(x) + "," + std::to_string(y) +
                             "," + std::to_string(error) + ")");
}
void position(MotionVector v, int x, int y) {
  CHECK(v.x == x && v.y == y);
}

void parent_interpolation() {
  const MotionGrid parent{2, 2, {{{-1, 1}, 1}, {{-2, 2}, 2}, {{0, 3}, 3}, {{1, 4}, 4}}};
  PredictionGeometry g{8, 8, 0, 0, 1, 2};
  triple(interpolate_predictor(parent, 1, 1, g), -4, 7, 2);
  g.overlap_x = g.overlap_y = 2;
  triple(interpolate_predictor(parent, 1, 1, g), -4, 6, 1); // -544/36 truncates to -15, then floor /4
  triple(interpolate_predictor(parent, 0, 1, g), -4, 5, 1); // boundary selects A,A,below,below
  const MotionGrid line{2, 1, {{{1, -1}, 1}, {{5, -5}, 5}}};
  g = {8, 8, 0, 0, 1, 1};
  triple(interpolate_predictor(line, 0, 0, g), 2, -2, 1);
  triple(interpolate_predictor(line, 1, 0, g), 4, -4, 2);
  triple(interpolate_predictor(line, 3, 0, g), 10, -10, 5);
  triple(interpolate_predictor(line, INT32_MAX, INT32_MAX, g), 10, -10, 5);
  triple(interpolate_predictor(line, INT32_MIN, 0, g), 2, -2, 1);
  triple(interpolate_predictor(line, 1, -1, g), 4, -4, 2);
  triple(interpolate_predictor(line, -1, INT32_MIN, g), 2, -2, 1);
  const MotionGrid column{1, 2, line.values};
  triple(interpolate_predictor(column, 0, 1, g), 4, -4, 2);
  const MotionGrid single{1, 1, {{{-3, 3}, 7}}};
  g.parent_pel = 4;
  triple(interpolate_predictor(single, 0, 0, g), -2, 1, 7); // negative half rounds down
  g.child_pel = 4;
  triple(interpolate_predictor(single, 0, 0, g), -6, 6, 7);
  g.overlap_x = 4; // overlap on only one axis is still the overlap formula
  triple(interpolate_predictor(single, 0, 0, g), -6, 6, 7);
  triple(parent.values[0], -1, 1, 1); // parent remains immutable

  const MotionGrid extremes{2, 2, std::vector<MotionTriple>(4, {{INT32_MIN, INT32_MAX}, 7})};
  for (int ox : {0, 64})
    for (int oy : {0, 64}) {
      const PredictionInterpolationPlan plan(extremes, {128, 128, ox, oy, 4, 1});
      for (int y : {-1, 0, 1, 2, 3, INT32_MAX})
        for (int x : {-1, 0, 1, 2, 3, INT32_MAX})
          triple(plan(x, y), -1073741824, 1073741823, 7);
    }

  // Mixed interior weights checked against exact rational arithmetic for
  // small values: numerator truncation precedes floor coordinate division.
  for (int ox : {0, 2, 4})
    for (int oy : {0, 2, 4}) {
      if (ox == 0 && oy == 0)
        continue;
      const MotionGrid p{2, 2, {{{-13, 17}, 5}, {{23, -29}, 11}, {{-31, 37}, 19}, {{41, -43}, 23}}};
      const std::int64_t ax = 24 - 2 * ox, ay = 24 - 2 * oy, bx = 4 * (8 - ox) - ax, by = 4 * (8 - oy) - ay;
      const std::int64_t denominator = (8 - ox) * (8 - oy);
      const auto nx = (-13 * ax * ay + 23 * bx * ay - 31 * ax * by + 41 * bx * by) / denominator;
      const auto ny = (17 * ax * ay - 29 * bx * ay + 37 * ax * by - 43 * bx * by) / denominator;
      const auto ns = (5 * ax * ay + 11 * bx * ay + 19 * ax * by + 23 * bx * by) / denominator;
      const auto x = static_cast<int>(std::floor(double(nx) / 8));
      const auto y = static_cast<int>(std::floor(double(ny) / 8));
      triple(interpolate_predictor(p, 1, 1, {8, 8, ox, oy, 1, 1}), x, y, ns / 16);
    }
}

void interpolation_parity() {
  const MotionGrid p{2, 2, {{{-13, 17}, 5}, {{23, -29}, 11}, {{-31, 37}, 19}, {{41, -43}, 23}}};
  const PredictionInterpolationPlan plan(p, {8, 8, 2, 2, 1, 1});
  triple(plan(1, 1), -18, 23, 8);
  triple(plan(2, 1), 24, -29, 11);
  triple(plan(1, 2), -30, 37, 15);
  triple(plan(2, 2), 30, -31, 17);
}

void global_modes() {
  MotionGrid grid{2, 1, {{{2, 0}, 0}, {{0, 0}, 0}}};
  position(global_predictor(grid), 2, 0);
  grid.values[0].vector.x = 10;
  position(global_predictor(grid), 0, 0);
  std::reverse(grid.values.begin(), grid.values.end());
  position(global_predictor(grid), 0, 0);
  grid = {3, 1, {{{0, 0}, 0}, {{0, 0}, 0}, {{3, 0}, 0}}};
  position(global_predictor(grid), 2, 0);
  grid.values[2].vector.x = 6;
  position(global_predictor(grid), 0, 0); // strict distance <6
  grid.values[2].vector.x = 5;
  position(global_predictor(grid), 3, 0); // truncation after doubling
  grid = {4, 1, {{{10, 30}, 0}, {{10, 30}, 0}, {{30, 10}, 0}, {{30, 10}, 0}}};
  position(global_predictor(grid), 20, 20); // independent tied modes, empty joint set
  grid = {3, 1, {{{-3, -3}, 0}, {{-3, -3}, 0}, {{-2, -2}, 0}}};
  position(global_predictor(grid), -5, -5);
  position(global_predictor(grid, false), 0, 0);
  position(enter_global_level({2, -3}, 4, 2), 8, -10);
  position(enter_global_level({0, 0}, 2, -1), 0, -1);
}

void spatial_neighbours() {
  MotionGrid grid{3, 3, std::vector<MotionTriple>(9)};
  grid.values[3] = {{9, -2}, 10};  // completed left neighbour
  grid.values[1] = {{2, 8}, 20};   // completed above
  grid.values[8] = {{-3, 4}, 100}; // future lower-right, still initial
  grid.values[5] = {{-1, 7}, 30};
  grid.values[6] = {{6, 1}, 40};
  const CandidateDomain omega{-4, -4, 5, 10};
  auto p = spatial_predictors(grid, 1, 1, 1, 1, {10, 0}, omega);
  triple(p.p[1], 4, -2, 10);
  triple(p.p[2], 2, 8, 20);
  triple(p.p[3], -3, 4, 100);
  triple(p.p[0], 2, 4, 100);
  position(p.global, 4, 0);
  p = spatial_predictors(grid, 1, 1, -1, 1, {10, 0}, {-8, -8, 12, 12});
  triple(p.p[1], -1, 7, 30);
  triple(p.p[3], 6, 1, 40);
  triple(p.p[0], 2, 7, 40);
  position(p.global, 10, 0); // preceding clamp did not mutate incoming global
  p = spatial_predictors(grid, 1, 2, 1, 1, {0, 0}, {-8, -8, 12, 12});
  triple(p.p[3], -1, 7, 30); // bottom row falls back to forward upper neighbour
  p = spatial_predictors(grid, 0, 0, 1, -1, {0, 0}, omega);
  triple(p.p[1], 0, -1, 0);
  triple(p.p[2], 0, -1, 0);
  triple(p.p[0], 0, -1, 0);
  p = spatial_predictors(grid, 2, 1, 1, 1, {0, 0}, omega);
  triple(p.p[3], 0, 1, 0); // no forward column at either height
  triple(grid.values[3], 9, -2, 10);
}

void invalid_and_overflow() {
  const MotionGrid valid{1, 1, {{{1, 1}, 1}}};
  const PredictionGeometry g{8, 8, 0, 0, 1, 1};
  rejects<std::invalid_argument>([&] { interpolate_predictor({2, 1, valid.values}, 0, 0, g); });
  rejects<std::invalid_argument>([&] { interpolate_predictor(valid, 0, 0, {8, 8, 5, 0, 1, 1}); });
  rejects<std::invalid_argument>([&] { interpolate_predictor(valid, 0, 0, {8, 8, 0, 0, 3, 1}); });
  rejects<std::invalid_argument>([&] { interpolate_predictor({1, 1, {{{0, 0}, -1}}}, 0, 0, g); });
  rejects<std::overflow_error>([&] { interpolate_predictor({1, 1, {{{INT32_MAX, 0}, 0}}}, 0, 0, g); });
  rejects<std::overflow_error>([&] { interpolate_predictor({1, 1, {{{0, 0}, INT64_MAX}}}, 0, 0, g); });
  rejects<std::overflow_error>([] { global_predictor({1, 1, {{{INT32_MIN, 0}, 0}}}); });
  rejects<std::overflow_error>([] { enter_global_level({INT32_MAX, 0}, 4, 0); });
  rejects<std::invalid_argument>([&] { spatial_predictors(valid, 0, 0, 0, 0, {0, 0}, {0, 0, 1, 1}); });
  rejects<std::invalid_argument>([&] { spatial_predictors(valid, 0, 0, 1, 0, {0, 0}, {0, 0, 0, 1}); });
}
} // namespace

int main() {
  try {
    parent_interpolation();
    interpolation_parity();
    global_modes();
    spatial_neighbours();
    invalid_and_overflow();
    std::cout << "Scalar vector prediction checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
