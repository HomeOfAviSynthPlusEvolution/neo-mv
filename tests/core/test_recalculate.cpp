#include "core/motion/recalculate.hpp"
#include "motion_fixture.hpp"

#include <iostream>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("recalculate assertion failed at line " + std::to_string(line));
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

void mapping() {
  MotionFixture<std::uint8_t> old_buffers(16, 8, 8, 8), target(16, 8, 16, 8, 8);
  auto old = old_buffers.old_field(999);
  old.metadata.width = 24; // rightmost old vector 4 also satisfies its public bounds
  old.grid.values[1].vector.x = 4;
  validate_owned_field(old);
  const auto block = analysis_block(target.metadata, 0, 0);
  const auto omega = analysis_domain(target.metadata, block);
  CHECK(recalculate_detail::map(old, target.metadata, 0, 0, omega, false).x == 4);
  CHECK(recalculate_detail::map(old, target.metadata, 0, 0, omega, true).x == 2);
  old.metadata.pel = 2;
  old.grid.values[0].vector.x = old.grid.values[1].vector.x = -1;
  CHECK(recalculate_detail::map(old, target.metadata, 0, 0, omega, true).x == -1);
  old.state = FieldState::metadata_only;
  CHECK(recalculate_detail::map(old, target.metadata, 0, 0, omega, true).x == 0);
  old = old_buffers.old_field();
  old.metadata.block_width = old.metadata.block_height = 3;
  old.metadata.blocks_x = old.metadata.blocks_y = 2;
  old.grid = {2, 2, {{{-3, 0}, 1}, {{4, 0}, 2}, {{5, 0}, 3}, {{-7, 0}, 4}}};
  MotionFixture<std::uint8_t> small(4, 4, 4, 4);
  CHECK(recalculate_detail::map(old, small.metadata, 0, 0, {-4, -4, 4, 4}, true).x == 0);
  CHECK(recalculate_detail::map(old, small.metadata, 0, 0, {-4, -4, 4, 4}, false).x == -3);
  old.metadata.block_height = 2;
  old.grid.values = {{{-1, 0}, 0}, {{-1, 0}, 0}, {{-1, 0}, 0}, {{0, 0}, 0}};
  // sx=3, sy=2, dx=dy=1: omitting the inner truncation would give zero.
  CHECK(recalculate_detail::map(old, small.metadata, 0, 0, {-4, -4, 4, 4}, true).x == -1);
  old.grid.values[0].vector.x = -3;
  // Negative centre displacement is truncated before indices are clamped.
  old.metadata.block_width = old.metadata.block_height = 16;
  CHECK(recalculate_detail::map(old, small.metadata, 0, 0, {-4, -4, 4, 4}, true).x == -3);
  CHECK(recalculate_detail::fractional_product(INT64_MIN, 2147483646LL, 2147483647LL) == INT64_MIN + 4294967299LL);
}

void actual_pixels() {
  MotionFixture<std::uint8_t> fixture(4, 4, 4, 4);
  const int current[] = {10, 20, 30, 30}, reference[] = {0, 10, 20, 30};
  const auto extent = fixture.geometry.planes[0].current;
  for (int y = 0; y < extent.height; ++y)
    for (int x = 0; x < extent.width; ++x) {
      const auto i = std::size_t(y) * (extent.width + 1) + x;
      fixture.source[0][i] = static_cast<std::uint8_t>(current[std::clamp(x - 4, 0, 3)]);
      fixture.reference[0][0][i] = static_cast<std::uint8_t>(reference[std::clamp(x - 4, 0, 3)]);
    }
  auto old = fixture.old_field(999);
  RecalculateControls controls;
  controls.thsad = 0;
  controls.search = 4;
  controls.pnew = 0;
  controls.searchparam = 0; // effective range 1
  auto result = recalculate_vectors(old, fixture.metadata, fixture.geometry, fixture.frames, controls);
  CHECK(result.values[0].vector.x == 1 && result.values[0].error == 0);
  CHECK(old.grid.values[0].error == 999 && old.grid.values[0].vector.x == 0);
  controls.thsad = 480; // 4x4 area gives T=120; equality avoids search
  result = recalculate_vectors(old, fixture.metadata, fixture.geometry, fixture.frames, controls);
  CHECK(result.values[0].vector.x == 0 && result.values[0].error == 120);
  controls.thsad = 479;
  result = recalculate_vectors(old, fixture.metadata, fixture.geometry, fixture.frames, controls);
  CHECK(result.values[0].vector.x == 1 && result.values[0].error == 0);
  old.state = FieldState::metadata_only;
  old.grid.values.clear();
  result = recalculate_vectors(old, fixture.metadata, fixture.geometry, fixture.frames, controls);
  CHECK(result.values[0].vector.x == 1 && result.values[0].error == 0);
}

void metrics_and_admission() {
  MotionFixture<float> f(8, 8, 4, 4, 4, 1, true);
  for (auto& plane : f.source)
    std::fill(plane.begin(), plane.end(), 1.0f / 65536.0f);
  RecalculateControls controls;
  controls.thsad = INT32_MAX;
  controls.satd = true;
  auto a = recalculate_vectors(f.old_field(), f.metadata, f.geometry, f.frames, controls);
  for (auto v : a.values)
    CHECK(v.error == 16); // Y SATD 8 + U/V SAD 4 each
  controls.meander = false;
  const auto b = recalculate_vectors(f.old_field(), f.metadata, f.geometry, f.frames, controls);
  for (std::size_t i = 0; i < a.values.size(); ++i)
    CHECK(a.values[i].error == b.values[i].error && a.values[i].vector.x == b.values[i].vector.x);
  CHECK(scale_area(scale_precision(5, 8), 4, 4) == 1);
  CHECK(scale_precision(-2, 8) == -1);
  CHECK(scale_precision(scale_area(5, 4, 4), 10) == 4);
  MotionFixture<std::uint16_t> unsafe(16, 16, 8, 8, 3, 2, true);
  rejects<std::invalid_argument>([&] {
    recalculate_vectors(unsafe.old_field(), unsafe.metadata, unsafe.geometry, SamplingFrames<std::uint16_t>{},
                        controls);
  });
  // A malformed unused phase must fail admission even when the zero vector
  // already meets thsad and the search never samples that phase.
  MotionFixture<std::uint16_t> phased(16, 16, 8, 8, 4, 2, true);
  auto invalid_frames = phased.frames;
  invalid_frames.reference[2][3] = {};
  rejects<std::invalid_argument>(
      [&] { recalculate_vectors(phased.old_field(), phased.metadata, phased.geometry, invalid_frames, controls); });
  auto mismatch = f.old_field();
  mismatch.metadata.bits = 16;
  rejects<std::invalid_argument>([&] { recalculate_vectors(mismatch, f.metadata, f.geometry, f.frames, controls); });
  mismatch = f.old_field();
  mismatch.grid.values.back().error = -1;
  rejects<std::invalid_argument>([&] { recalculate_vectors(mismatch, f.metadata, f.geometry, f.frames, controls); });
}
} // namespace
int main() {
  try {
    mapping();
    actual_pixels();
    metrics_and_admission();
    std::cout << "Scalar Recalculate checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
