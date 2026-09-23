#include "core/interpolation/dense.hpp"

#include <iostream>
#include <future>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("dense interpolation assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  } catch (const std::overflow_error&) {
    caught = true;
  }
  CHECK(caught);
}
AnalysisMetadata metadata(int delta = 1, int bits = 8) {
  AnalysisMetadata m;
  m.width = m.real_width = 8;
  m.height = m.real_height = 4;
  m.pad_x = m.pad_y = 50000;
  m.pel = m.levels = m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = 4;
  m.blocks_x = 2;
  m.blocks_y = 1;
  m.delta = delta;
  m.bits = bits;
  return m;
}
MotionGrid motion(int x0, int x1, int y = 0) {
  return {2, 1, {{{x0, y}, 0}, {{x1, y}, 0}}};
}
template <class T, class Allocator>
void rows(const std::vector<T, Allocator>& pixels, int width, int height, std::initializer_list<int> expected) {
  CHECK(expected.size() == static_cast<std::size_t>(width));
  CHECK(pixels.size() == static_cast<std::size_t>(width) * height);
  for (int y = 0; y < height; ++y)
    CHECK(std::equal(expected.begin(), expected.end(), pixels.begin() + std::ptrdiff_t(y) * width));
}
void examples() {
  for (int bits : {8, 10, 16, 32}) {
    const auto b = metadata(1, bits), f = metadata(-1, bits);
    const auto grid = motion(4, 0);
    const auto y = DenseInterpolationPlan<>(b, f, 1, 1, 80).generate(grid, grid, 256);
    rows(y.mF, y.width, y.height, {255, 255, 223, 159, 96, 32, 0, 0});
    rows(y.mB, y.width, y.height, {0, 0, 32, 96, 159, 223, 255, 255}); // Zero event time still writes a mask.
    rows(y.B.x, y.width, y.height, {4, 4, 4, 3, 2, 1, 0, 0});
    CHECK(y.B.x == y.F.x);
    CHECK(!y.BB && !y.FF);
    const auto chroma = DenseInterpolationPlan<>(b, f, 2, 2, 80).generate(grid, grid, 256);
    rows(chroma.mF, chroma.width, chroma.height, {255, 191, 64, 0});
    rows(chroma.mB, chroma.width, chroma.height, {0, 64, 191, 255});
    // Analysis precision/chroma do not change the luma event amplitude 255.
    auto analysis_b = b, analysis_f = f;
    analysis_b.chroma = analysis_f.chroma = true;
    analysis_b.ratio_x = analysis_f.ratio_x = 2;
    analysis_b.ratio_y = analysis_f.ratio_y = 2;
    CHECK(DenseInterpolationPlan<>(analysis_b, analysis_f, 2, 2, 80).generate(grid, grid, 256).mF == chroma.mF);
  }
}
void public_vectors_and_extras() {
  const auto b = metadata(), f = metadata(-1);
  const auto grid = motion(32772, 32768);
  const auto extra_b = motion(-1, -1, -3), extra_f = motion(40000, 40000, 40000);
  const auto result = DenseInterpolationPlan<>(b, f, 1, 1, 80).generate(grid, grid, 256, &extra_b, &extra_f);
  rows(result.B.x, result.width, result.height, {32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767});
  rows(result.F.x, result.width, result.height, {32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767});
  // Motion saturation erases the difference; occlusion still uses 32772-32768=4.
  rows(result.mF, result.width, result.height, {255, 255, 223, 159, 96, 32, 0, 0});
  CHECK(result.BB && result.FF);
  rows(result.BB->x, result.width, result.height, {-1, -1, -1, -1, -1, -1, -1, -1});
  rows(result.FF->y, result.width, result.height, {32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767});
  const auto chroma = DenseInterpolationPlan<>(b, f, 2, 2, 80).generate(extra_b, extra_b, 0);
  rows(chroma.B.x, chroma.width, chroma.height, {-1, -1, -1, -1});
  rows(chroma.B.y, chroma.width, chroma.height, {-2, -2, -2, -2});
  rows(chroma.mB, chroma.width, chroma.height, {0, 0, 0, 0});
  rows(chroma.mF, chroma.width, chroma.height, {0, 0, 0, 0});
  const auto basic = DenseInterpolationPlan<>(b, f, 1, 1, 80).generate(grid, grid, 256);
  CHECK(basic.mB == result.mB && basic.mF == result.mF); // Extra fields never create events.
  const auto zero_time = DenseInterpolationPlan<>(b, f, 1, 1, 80).generate(grid, grid, 0);
  CHECK(zero_time.B.x == result.B.x); // No time scaling before dense motion generation.
}
void failures() {
  const auto b = metadata(), f = metadata(-1);
  for (double ml : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(),
                    static_cast<double>(std::numeric_limits<float>::denorm_min()), 1.0e-37})
    rejects([&] { DenseInterpolationPlan<>(b, f, 1, 1, ml); });
  rejects([&] { DenseInterpolationPlan<>(f, b, 1, 1); });
  rejects([&] { DenseInterpolationPlan<>(b, f, 3, 1); });
  auto changed = f;
  changed.real_width = 7;
  rejects([&] { DenseInterpolationPlan<>(b, changed, 1, 1); });
  const DenseInterpolationPlan<> plan(b, f, 1, 1);
  const auto valid = motion(0, 0);
  rejects([&] { plan.generate(valid, valid, -1); });
  rejects([&] { plan.generate(valid, valid, 257); });
  rejects([&] { plan.generate(valid, valid, 128, &valid); });
  auto malformed = valid;
  malformed.values[1].vector.x = INT32_MAX;
  rejects([&] { plan.generate(malformed, valid, 0); }); // Saturation does not legalize a public-bound violation.
  rejects([&] { plan.generate(valid, malformed, 256); });
  rejects([&] { plan.generate(valid, valid, 128, &valid, &malformed); });
  malformed = valid;
  malformed.values[1].error = -1;
  rejects([&] { plan.generate(valid, malformed, 0); });
  malformed = valid;
  malformed.values.pop_back();
  rejects([&] { plan.generate(valid, malformed, 128); });
}

void reuse(int ratio) {
  const DenseInterpolationPlan<> plan(metadata(), metadata(-1), ratio, ratio, 80);
  const auto grid = motion(4, 0), extra = motion(-1, -1, -3);
  const auto first = plan.generate_reusing(grid, grid, 0, &extra, &grid);
  const auto next = plan.generate_reusing(grid, grid, 256, &extra, &grid);
  CHECK(first.motion == next.motion);
  CHECK(first.mF != next.mF);
  auto same_motion = grid;
  same_motion.values[0].error = 10;
  CHECK(plan.generate_reusing(same_motion, grid, 128, &extra, &grid).motion == first.motion);
  same_motion.values[0].error = -1;
  rejects([&] { plan.generate_reusing(same_motion, grid, 128, &extra, &grid); });
  same_motion = grid;
  same_motion.width = 1;
  rejects([&] { plan.generate_reusing(same_motion, grid, 128, &extra, &grid); });
  rejects([&] { plan.generate_reusing(grid, grid, 257, &extra, &grid); });
  rejects([&] { plan.generate_reusing(grid, grid, 128, &extra); });
  const auto changed = motion(5, 0);
  const auto replaced = plan.generate_reusing(changed, grid, 128, &extra, &grid);
  CHECK(replaced.motion != first.motion);
  CHECK(first.motion->B.x == plan.generate(grid, grid, 0).B.x);
  CHECK(plan.generate_reusing(changed, grid, 128, &grid, &grid).motion != replaced.motion);
  const auto basic = plan.generate_reusing(grid, grid, 128);
  CHECK(!basic.motion->BB && !basic.motion->FF);
  const auto copy = plan;
  CHECK(copy.generate_reusing(grid, grid, 128).motion == basic.motion);

  const auto clipped = motion(32772, 32768), clipped_changed = motion(32768, 32768);
  const auto a = plan.generate_reusing(clipped, clipped, 256);
  const auto b = plan.generate_reusing(clipped_changed, clipped_changed, 256);
  CHECK(a.motion != b.motion);
  CHECK(a.motion->F.x == b.motion->F.x);
  CHECK(a.mF != b.mF);

  // Interleaved publication must preserve each caller's fields and masks.
  auto run = [&](int seed) {
    for (int i = 0; i < 24; ++i) {
      const auto input = motion((i + seed) % 8, seed);
      const int time = i * 11 % 257;
      const auto expected = plan.generate(input, grid, time, &extra, &input);
      const auto actual = plan.generate_reusing(input, grid, time, &extra, &input);
      CHECK(actual.motion->B.x == expected.B.x && actual.motion->B.y == expected.B.y);
      CHECK(actual.motion->F.x == expected.F.x && actual.motion->F.y == expected.F.y);
      CHECK(actual.motion->BB->x == expected.BB->x && actual.motion->BB->y == expected.BB->y);
      CHECK(actual.motion->FF->x == expected.FF->x && actual.motion->FF->y == expected.FF->y);
      CHECK(actual.mB == expected.mB && actual.mF == expected.mF);
    }
  };
  auto task = std::async(std::launch::async, run, 1);
  run(2);
  task.get();
}

void oversized_not_retained() {
  auto b = metadata(), f = metadata(-1);
  b.width = b.real_width = f.width = f.real_width = 2048;
  b.height = b.real_height = f.height = f.real_height = 2048;
  b.blocks_x = f.blocks_x = b.blocks_y = f.blocks_y = 512;
  const DenseInterpolationPlan<> plan(b, f, 1, 1);
  MotionGrid grid{512, 512, std::vector<MotionTriple>(512 * 512, {{0, 0}, 0})};
  std::weak_ptr<const InterpolationMotionFields> released;
  {
    const auto result = plan.generate_reusing(grid, grid, 128, &grid, &grid);
    released = result.motion;
    CHECK(result.motion->B.x.front() == 0 && result.motion->FF->y.back() == 0);
  }
  CHECK(released.expired());
}
} // namespace

int main() {
  try {
    examples();
    public_vectors_and_extras();
    failures();
    reuse(1);
    reuse(2);
    oversized_not_retained();
    std::cout << "Dense interpolation checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
