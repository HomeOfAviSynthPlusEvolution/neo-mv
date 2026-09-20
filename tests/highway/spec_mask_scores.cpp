#include "highway/mask_scores.hpp"
#include "core/mask/scores.hpp"

#include <iostream>
#include <limits>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("mask score assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E = std::invalid_argument, class F>
bool rejects(F call) {
  try {
    call();
  } catch (const E&) {
    return true;
  }
  return false;
}
AnalysisMetadata metadata(int nx = 1, int ny = 1, int block = 4, int bits = 8) {
  AnalysisMetadata m;
  m.width = m.real_width = nx * block;
  m.height = m.real_height = ny * block;
  m.pad_x = m.pad_y = 32;
  m.pel = m.levels = m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = block;
  m.blocks_x = nx;
  m.blocks_y = ny;
  m.delta = 1;
  m.bits = bits;
  return m;
}
MotionGrid motion(int nx, int ny, std::initializer_list<MotionTriple> values) {
  return {nx, ny, std::vector<MotionTriple>(values)};
}

void magnitude() {
  auto m = metadata(2);
  const auto grid = motion(2, 1, {{{3, 4}, 0}, {{0, 0}, 123}});
  CHECK(simd::VectorLengthMaskPlan<std::uint8_t>(m, 0.2f, 1, 256).generate(grid) ==
        std::vector<std::uint8_t>({255, 0}));
  CHECK(simd::VectorLengthMaskPlan<std::uint8_t>(m, 0.2f, 1, 0).generate(grid) == std::vector<std::uint8_t>({255, 0}));
  m.pel = 2;
  const auto doubled = motion(2, 1, {{{6, 8}, 0}, {{0, 0}, 0}});
  CHECK(simd::VectorLengthMaskPlan<std::uint8_t>(m, 0.2f, 1, 256).generate(doubled) ==
        std::vector<std::uint8_t>({255, 0}));
  m.pel = 1;
  const auto four = motion(2, 1, {{{4, 0}, 0}, {{0, 0}, 0}});
  CHECK(simd::VectorLengthMaskPlan<std::uint8_t>(m, 0.125f, 2, 256).generate(four) ==
        std::vector<std::uint8_t>({63, 0}));
  CHECK(simd::VectorLengthMaskPlan<std::uint8_t>(m, 0, 0, 256).generate(four) == std::vector<std::uint8_t>({255, 255}));
  m.bits = 32;
  CHECK(simd::VectorLengthMaskPlan<float>(m, 0.125f, 2, 256).generate(four) == std::vector<float>({0.25f, 0}));
  // f*f may underflow to zero; zero is a valid normalization constant.
  CHECK(simd::VectorLengthMaskPlan<float>(m, std::numeric_limits<float>::denorm_min(), 2, 0).generate(four) ==
        std::vector<float>({0, 0}));
  // A representable subnormal square and result must survive.
  const float f = std::ldexp(1.0f, -65);
  const auto unit = motion(2, 1, {{{1, 0}, 0}, {{0, 0}, 0}});
  CHECK(simd::VectorLengthMaskPlan<float>(m, f, 2, 0).generate(unit)[0] == std::ldexp(1.0f, -130));
}

void sad() {
  auto m = metadata(2, 1, 8);
  auto grid = motion(2, 1, {{{0, 0}, 8}, {{0, 0}, 16}});
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 256).generate(grid) == std::vector<std::uint8_t>({127, 255}));
  m.bits = 10;
  CHECK(simd::SADMaskPlan<std::uint16_t>(m, 1, 1, 256).generate(grid) == std::vector<std::uint16_t>({127, 255}));
  m.bits = 32;
  grid.values[0].error = 2048;
  CHECK(simd::SADMaskPlan<float>(m, 1, 1, 256).generate(grid)[0] == 0.5f);
  m = metadata(3);
  grid = motion(3, 1, {{{0, 0}, 0}, {{4, 0}, 4}, {{0, 0}, 8}});
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 0).generate(grid) == std::vector<std::uint8_t>({0, 0, 255}));
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 256).generate(grid) == std::vector<std::uint8_t>({0, 255, 255}));
  grid.values[1] = {{-1, 0}, 2};
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 0).generate(grid)[1] == 127); // trunc(-1/4) == 0
  grid.values[1] = {{4, 4}, 2};
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 0).generate(grid)[1] == 127); // valid x, invalid y -> self
  m.chroma = true;
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 0).generate(grid)[1] == 127); // no chroma divisor
  // Projection products exceed int32 without wrapping, then fall back to self.
  m.pad_x = INT32_MAX;
  grid.values[1] = {{1000000000, 0}, 2};
  CHECK(simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 0).generate(grid)[1] == 127);
  grid.values[0].error = INT64_MAX;
  CHECK(rejects<std::overflow_error>([&] { simd::SADMaskPlan<std::uint8_t>(m, 1e37f, 0, 256).generate(grid); }));
}

void occlusion() {
  auto m = metadata(2);
  auto grid = motion(2, 1, {{{4, 0}, 0}, {{0, 0}, 0}});
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 256).generate(grid) ==
        std::vector<std::uint8_t>({255, 255}));
  m.delta = -1;
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 256).generate(grid) ==
        std::vector<std::uint8_t>({255, 0}));
  grid.values[0].vector.x = 8;
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 256).generate(grid) ==
        std::vector<std::uint8_t>({0, 0}));
  // Empty intervals are discarded before contribution arithmetic, even when
  // that arithmetic would overflow. Their endpoints must not be swapped.
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1e35f, 1, 256).generate(grid) == std::vector<std::uint8_t>({0, 0}));
  m.delta = 1;
  CHECK(rejects<std::overflow_error>([&] { simd::OcclusionMaskPlan<std::uint8_t>(m, 1e35f, 1, 256).generate(grid); }));
  grid.values[0].vector.x = 4;
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 0).generate(grid) ==
        std::vector<std::uint8_t>({0, 255}));
  m.delta = 0;
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 0).generate(grid) ==
        std::vector<std::uint8_t>({255, 255}));
  grid.values[0].vector.x = 0;
  grid.values[1].vector.x = 4;
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 0, 256).generate(grid) ==
        std::vector<std::uint8_t>({0, 0}));
  m = metadata(2, 2);
  grid = motion(2, 2, {{{4, 4}, 0}, {{0, 0}, 0}, {{0, 0}, 0}, {{0, 0}, 0}});
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 256).generate(grid) ==
        std::vector<std::uint8_t>({255, 255, 255, 0}));
  // Last-column vertical and last-row horizontal events are included.
  grid = motion(2, 2, {{{0, 0}, 0}, {{0, 4}, 0}, {{4, 0}, 0}, {{0, 0}, 0}});
  CHECK(simd::OcclusionMaskPlan<std::uint8_t>(m, 1.0f / 80, 1, 256).generate(grid) ==
        std::vector<std::uint8_t>({0, 255, 255, 255}));
  m = metadata(2, 1, 4, 16);
  grid = motion(2, 1, {{{3, 0}, 0}, {{0, 0}, 0}});
  // Normative (M*o)*a gives 41641; M*(o*a) gives 41640.99609375.
  CHECK(simd::OcclusionMaskPlan<std::uint16_t>(m, 0.010590014979243279f, 1, 256).generate(grid)[1] == 41641);
  m = metadata(2, 1, 4, 32);
  grid = motion(2, 1, {{{1, 0}, 0}, {{0, 0}, 0}});
  CHECK(simd::OcclusionMaskPlan<float>(m, 1.0f / 80, 2, 256).generate(grid)[1] == 0.0625f);
  m.pad_x = INT32_MAX;
  grid.values[0].vector.x = INT32_MAX - 1;
  grid.values[1].vector.x = -INT32_MAX;
  CHECK(simd::OcclusionMaskPlan<float>(m, 1e-12f, 1, 256).generate(grid)[0] > 0); // wide difference/product
}

void validation() {
  auto m = metadata();
  auto grid = motion(1, 1, {{{0, 0}, 0}});
  CHECK(rejects([&] { simd::VectorLengthMaskPlan<std::uint8_t>(m, 1, 1, -1); }));
  CHECK(rejects([&] { simd::SADMaskPlan<std::uint8_t>(m, 1, -1, 0); }));
  CHECK(rejects([&] { simd::OcclusionMaskPlan<std::uint8_t>(m, 1, 1, 257); }));
  CHECK(rejects([&] { simd::VectorLengthMaskPlan<std::uint8_t>(m, -1, 1, 0); }));
  CHECK(rejects([&] { simd::VectorLengthMaskPlan<float>(m, 1, 1, 0); }));
  CHECK(rejects<std::overflow_error>([&] { simd::VectorLengthMaskPlan<std::uint8_t>(m, 1e30f, 0, 0); }));
  CHECK(rejects<std::overflow_error>(
      [&] { simd::SADMaskPlan<std::uint8_t>(m, std::numeric_limits<float>::max(), 0, 0); }));
  CHECK(rejects<std::overflow_error>(
      [&] { simd::OcclusionMaskPlan<std::uint8_t>(m, std::numeric_limits<float>::max(), 0, 0); }));
  const simd::VectorLengthMaskPlan<std::uint8_t> length(m, 1, 1, 0);
  const simd::SADMaskPlan<std::uint8_t> sad(m, 1, 1, 0);
  const simd::OcclusionMaskPlan<std::uint8_t> occ(m, 1, 1, 0);
  grid.values[0].error = -1;
  CHECK(rejects([&] { length.generate(grid); }));
  CHECK(rejects([&] { sad.generate(grid); }));
  CHECK(rejects([&] { occ.generate(grid); }));
  grid.values[0].error = 0;
  grid.values[0].vector.x = INT32_MAX;
  CHECK(rejects([&] { length.generate(grid); }));
  grid.values.clear();
  CHECK(rejects([&] { sad.generate(grid); }));
  grid.width = 2;
  CHECK(rejects([&] { occ.generate(grid); }));
  m.real_width = m.width = 5;
  CHECK(rejects([&] { simd::SADMaskPlan<std::uint8_t>(m, 1, 1, 0); }));
  m = metadata();
  m.pel = 0;
  CHECK(rejects([&] { simd::OcclusionMaskPlan<std::uint8_t>(m, 1, 1, 0); }));
}
} // namespace

int main() {
  try {
    magnitude();
    sad();
    occlusion();
    validation();
    std::cout << "mask score specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
