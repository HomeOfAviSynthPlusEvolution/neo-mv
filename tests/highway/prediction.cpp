#include "highway/prediction.hpp"
#include "hwy/targets.h"
#include <iostream>
#include <random>
#include <string>

using namespace neo_mv;
namespace {
void compare(const MotionGrid& parent, PredictionGeometry g, int width, int height) {
  MotionGrid a{width, height, std::vector<MotionTriple>(std::size_t(width) * height)}, b = a;
  int scalar_error = 0, simd_error = 0;
  const auto invoke = [&](bool simd) {
    try {
      if (simd)
        neo_mv::simd::interpolate_predictions(parent, g, b);
      else
        neo_mv::interpolate_predictions(parent, g, a);
      return 0;
    } catch (const std::overflow_error&) {
      return 1;
    } catch (const std::invalid_argument&) {
      return 2;
    }
  };
  scalar_error = invoke(false);
  simd_error = invoke(true);
  if (scalar_error != simd_error)
    throw std::runtime_error("exception mismatch");
  if (scalar_error)
    return;
  for (std::size_t i = 0; i < a.values.size(); ++i)
    if (a.values[i].vector.x != b.values[i].vector.x || a.values[i].vector.y != b.values[i].vector.y ||
        a.values[i].error != b.values[i].error)
      throw std::runtime_error("interpolation mismatch at " + std::to_string(i));
}
void run() {
  std::mt19937 random(9346);
  for (int width : {1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 32, 33})
    for (int height : {1, 2, 5}) {
      MotionGrid parent{width, height, std::vector<MotionTriple>(std::size_t(width) * height)};
      for (auto& v : parent.values)
        v = {{int(random() % 20001) - 10000, int(random() % 20001) - 10000}, std::int64_t(random())};
      for (const auto size : {std::pair<int, int>{4, 4},
                              {6, 6},
                              {8, 4},
                              {12, 12},
                              {16, 2},
                              {16, 16},
                              {24, 24},
                              {32, 16},
                              {48, 48},
                              {64, 64},
                              {128, 128}})
        for (int ox : {0, 1, size.first / 2})
          for (int oy : {0, 1, size.second / 2})
            for (int pp : {1, 2, 4})
              for (int cp : {1, 2, 4}) {
                PredictionGeometry g{size.first, size.second, ox, oy, pp, cp};
                for (int dx : {-1, 0, 3})
                  compare(parent, g, 2 * width + dx, 2 * height + 1);
              }
    }
  // Overlap blksize/8 gives divisors 49, 196, 784, 3136 and 12544, whose
  // binary64 reciprocals satisfy n*fl(1/n) < 1. Uniform motion must divide
  // exactly on both paths: a reciprocal product would give 2v-1, not 2v.
  for (const auto size : {std::pair<int, int>{8, 8}, {16, 16}, {32, 32}, {64, 64}, {128, 128}})
    for (int v : {1, 3, 100, -5}) {
      // 33 interior pairs exercise full AVX-512 groups and a scalar tail.
      MotionGrid uniform{34, 3, std::vector<MotionTriple>(102, {{v, -v}, std::int64_t(v < 0 ? -v : v)})};
      MotionGrid mixed = uniform;
      for (auto& t : mixed.values)
        t = {{int(random() % 20001) - 10000, int(random() % 20001) - 10000}, std::int64_t(random())};
      for (int pp : {1, 2})
        for (int cp : {1, 2, 4}) {
          const PredictionGeometry g{size.first, size.second, size.first / 8, size.second / 8, pp, cp};
          compare(mixed, g, 68, 6);
          compare(mixed, g, 67, 7);
          compare(mixed, g, 69, 7);
          MotionGrid child{68, 6, std::vector<MotionTriple>(408)};
          neo_mv::simd::interpolate_predictions(uniform, g, child);
          const int r = 3 - (cp == 4 ? 2 : cp == 2 ? 1 : 0) + (pp == 2 ? 1 : 0);
          const int expected = r >= 0 ? (16 * v) >> r : (16 * v) << -r;
          for (const auto& t : child.values)
            if (t.vector.x != expected || t.vector.y != -expected || t.error != (v < 0 ? -v : v))
              throw std::runtime_error("uniform overlap prediction is not exact");
          compare(uniform, g, 68, 6);
        }
    }
  // Check the binary64 admission boundary separately from the int64 overflow
  // bound below. All these errors are valid inputs, even above the SIMD bound.
  for (int size : {8, 16, 32, 64, 128}) {
    MotionGrid boundary{34, 3, std::vector<MotionTriple>(102)};
    const PredictionGeometry g{size, size, size / 8, size / 8, 1, 1};
    const PredictionInterpolationPlan plan(boundary, g);
    const auto bound = plan.binary64_error_limit();
    for (auto error : {bound - 1, bound, bound + 1, plan.error_limit() - 1}) {
      for (auto& t : boundary.values)
        t = {{1, -1}, error};
      MotionGrid child{68, 6, std::vector<MotionTriple>(408)};
      neo_mv::simd::interpolate_predictions(boundary, g, child);
      // Uniform errors must survive interpolation exactly, including the
      // scalar fallback. An exception is a failure, not an accepted outcome.
      for (const auto& t : child.values)
        if (t.vector.x != 2 || t.vector.y != -2 || t.error != error)
          throw std::runtime_error("uniform overlap prediction changed at error " + std::to_string(error));
      compare(boundary, g, 68, 6);
    }
    // Nonuniform sums exercise fractional quotients just below the bound.
    // Then make one interior parent exceed it: affected SIMD groups fall back,
    // while the remaining groups still use binary64 division.
    for (int round = 0; round < 16; ++round) {
      for (auto& t : boundary.values)
        t = {{int(random() % 20001) - 10000, int(random() % 20001) - 10000}, bound - std::int64_t(random() % 2048)};
      for (bool fallback : {false, true}) {
        if (fallback)
          boundary.values[boundary.width + 9].error = bound + 1;
        compare(boundary, g, 68, 6);
        compare(boundary, g, 67, 7);
        compare(boundary, g, 69, 7);
      }
    }
  }
  MotionGrid parent{17, 3, std::vector<MotionTriple>(51)};
  // Large errors near the int64 accumulation bound exercise checked scalar
  // fallback when overlap makes them exceed the binary64 admission bound.
  for (int overlap : {0, 1, 4}) {
    PredictionGeometry g{8, 8, overlap, overlap, 1, 2};
    const auto bound = PredictionInterpolationPlan(parent, g).error_limit();
    for (int round = 0; round < 32; ++round) {
      for (auto& v : parent.values) {
        const auto bits = (std::uint64_t(random()) << 32) | random();
        v = {{int(random() % 20001) - 10000, int(random() % 20001) - 10000},
             round % 2 ? bound - std::int64_t(bits % 4096) : std::int64_t(bits % bound)};
      }
      compare(parent, g, 35, 7);
      compare(parent, g, 13, 2);
    }
  }
  for (auto& v : parent.values)
    v = {};
  const auto limit = (INT64_MAX - 8) / 16;
  for (auto error : {std::int64_t(-1), limit, limit + 1, INT64_MAX})
    for (auto& v : parent.values) {
      v.error = error;
      compare(parent, {8, 8, 0, 0, 1, 1}, 35, 7);
      compare(parent, {8, 8, 4, 4, 1, 1}, 35, 7);
      v.error = 0;
    }
  for (auto& v : parent.values)
    v = {{INT32_MIN, INT32_MAX}, 0};
  for (int pp : {1, 4})
    for (int cp : {1, 4}) {
      compare(parent, {128, 128, 64, 64, pp, cp}, 35, 7);
      compare(parent, {128, 128, 1, 63, pp, cp}, 35, 7);
      compare(parent, {128, 128, 0, 0, pp, cp}, 35, 7);
    }
}
} // namespace
int main() {
  try {
    for (auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << hwy::TargetName(target) << std::endl;
      run();
    }
    hwy::SetSupportedTargetsForTest(0);
  } catch (const std::exception& e) {
    hwy::SetSupportedTargetsForTest(0);
    std::cerr << e.what() << '\n';
    return 1;
  }
}
