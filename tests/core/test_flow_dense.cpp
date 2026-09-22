#include "core/flow/dense.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/grid_resampling.hpp"
#endif

#include <iostream>

namespace {
using namespace neo_mv;
using TestResampler =
#if NEO_MV_TEST_HIGHWAY
    simd::GridResamplingPlan;
#else
    GridResamplingPlan;
#endif
using TestDensePlan = DenseFlowPlan<TestResampler>;

void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("dense flow assertion at line " + std::to_string(line));
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

AnalysisMetadata metadata(int nx = 2, int ny = 1, int block = 4) {
  AnalysisMetadata m;
  m.width = m.real_width = nx * block;
  m.height = m.real_height = ny * block;
  m.pad_x = m.pad_y = 8;
  m.pel = m.levels = m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = block;
  m.blocks_x = nx;
  m.blocks_y = ny;
  m.delta = 1;
  m.bits = 8;
  return m;
}
MotionGrid motion(std::initializer_list<MotionTriple> values) {
  return {static_cast<int>(values.size()), 1, std::vector<MotionTriple>(values)};
}
void rows(const DenseFlowField& field, std::initializer_list<std::int16_t> x, std::int16_t y = 0) {
  CHECK(field.width == static_cast<int>(x.size()));
  CHECK(field.x.size() == static_cast<std::size_t>(field.width) * field.height);
  CHECK(field.y.size() == field.x.size());
  for (int row = 0; row < field.height; ++row) {
    const auto start = field.x.begin() + static_cast<std::ptrdiff_t>(row) * field.width;
    CHECK(std::equal(x.begin(), x.end(), start));
  }
  CHECK(std::all_of(field.y.begin(), field.y.end(), [&](auto value) { return value == y; }));
}

void examples_and_cropping() {
  auto m = metadata();
  const auto grid = motion({{{0, 0}, 0}, {{4, 0}, 0}});
  rows(TestDensePlan(m, 1, 1).generate(grid, 0), {0, 0, 1, 2, 3, 4, 4, 4});
  rows(TestDensePlan(m, 2, 1).generate(grid, 0), {0, 1, 2, 2});
  // AnalysisChroma and its ratios cannot select the render-plane conversion.
  m.chroma = false;
  m.ratio_x = m.ratio_y = 2;
  rows(TestDensePlan(m, 1, 1).generate(grid, 0), {0, 0, 1, 2, 3, 4, 4, 4});
  m.real_width = 6;
  rows(TestDensePlan(m, 1, 1).generate(grid, 0), {0, 0, 1, 2, 3, 4});
  rows(TestDensePlan(m, 2, 1).generate(grid, 0), {0, 1, 2});
  m = metadata();
  rows(TestDensePlan(m, 1, 1).generate(motion({{{-1, 0}, 0}, {{0, 0}, 0}}), 0), {-1, -1, -1, -1, 0, 0, 0, 0});
  rows(TestDensePlan(m, 2, 1).generate(motion({{{-1, 0}, 0}, {{1, 0}, 0}}), 0), {-1, -1, 0, 0});
  m.overlap_x = 2;
  m.real_width = m.width = 6;
  rows(TestDensePlan(m, 2, 1).generate(grid, 0), {0, 1, 2});
}

void correction_and_saturation() {
  auto m = metadata(1);
  auto grid = motion({{{0, 0}, 0}});
  m.pel = 2;
  rows(TestDensePlan(m, 1, 1).generate(grid, 1), {0, 0, 0, 0}, 1);
  rows(TestDensePlan(m, 2, 2).generate(grid, 1), {0, 0}, 0);
  rows(TestDensePlan(m, 2, 2).generate(grid, -1), {0, 0}, -1);
  m.pel = 4;
  m.pad_x = m.pad_y = 50000;
  grid.values[0].vector = {40000, 32767};
  rows(TestDensePlan(m, 1, 1).generate(grid, 2), {32767, 32767, 32767, 32767}, 32767);
  rows(TestDensePlan(m, 2, 2).generate(grid, 2), {16383, 16383}, 16383);
  grid.values[0].vector = {-40000, -32768};
  rows(TestDensePlan(m, 2, 2).generate(grid, -2), {-16384, -16384}, -16384);
  // The public bounds can admit int32 extremes; vy+f uses a wider type.
  m.pad_x = m.pad_y = INT32_MAX;
  grid.values[0].vector = {INT32_MIN, INT32_MAX};
  rows(TestDensePlan(m, 1, 1).generate(grid, 2), {-32768, -32768, -32768, -32768}, 32767);
  grid.values[0].vector = {INT32_MAX, INT32_MIN};
  rows(TestDensePlan(m, 1, 1).generate(grid, -2), {32767, 32767, 32767, 32767}, -32768);
  CHECK(grid.values[0].vector.x == INT32_MAX && grid.values[0].vector.y == INT32_MIN);
}

// This injected resampler verifies the generated signed views use byte stride,
// the intended per-plane geometry, and separately owned input/output storage.
struct AuditedResampler {
  GridResamplingPlan inner;
  static inline int calls = 0;
  explicit AuditedResampler(GridResamplingGeometry geometry) : inner(geometry) {}
  template <class T, bool Validated = false>
  void resize(span2d::Plane<const T> input, span2d::Plane<T> output, int bits) const {
    static_assert(std::is_same_v<T, std::int16_t>);
    ++calls;
    CHECK(bits == 16);
    CHECK(input.stride_bytes() == input.width() * std::ptrdiff_t(sizeof(std::int16_t)));
    CHECK(output.stride_bytes() == output.width() * std::ptrdiff_t(sizeof(std::int16_t)));
    CHECK(!active_rows_overlap(input, output));
    CHECK(input.width() == 2 && input.height() == 1);
    CHECK(output.width() == 3 && output.height() == 2);
    inner.resize(input, output, bits);
  }
};

void storage_and_reuse() {
  auto m = metadata();
  m.real_width = 6;
  auto grid = motion({{{0, 2}, 7}, {{4, 2}, 9}});
  const DenseFlowPlan<AuditedResampler> audited(m, 2, 2);
  AuditedResampler::calls = 0;
  auto output = audited.generate(grid, 0);
  CHECK(AuditedResampler::calls == 2);
  rows(output, {0, 1, 2}, 1);
  CHECK(output.x.data() != output.y.data());
  output.x[0] = 123;
  output.y[0] = 456;
  rows(audited.generate(grid, 0), {0, 1, 2}, 1);
  CHECK(grid.values[0].vector.x == 0 && grid.values[0].vector.y == 2 && grid.values[0].error == 7);
  CHECK(grid.values[1].vector.x == 4 && grid.values[1].vector.y == 2 && grid.values[1].error == 9);
  grid.values[1].vector.x = 0;
  rows(audited.generate(grid, 0), {0, 0, 0}, 1);
}

void failures() {
  auto m = metadata();
  auto grid = motion({{{0, 0}, 0}, {{0, 0}, 0}});
  const TestDensePlan plan(m, 1, 1);
  rejects([&] { plan.generate(grid, 1); }); // pel=1 has no field correction.
  grid.values[1].vector.x = 40000;
  rejects([&] { plan.generate(grid, 0); }); // Not legitimized by saturation.
  grid.values[1].vector.x = 0;
  grid.values[1].error = -1;
  rejects([&] { plan.generate(grid, 0); });
  grid.values[1].error = 0;
  grid.width = 1;
  rejects([&] { plan.generate(grid, 0); });
  grid.width = 2;
  grid.values.pop_back();
  rejects([&] { plan.generate(grid, 0); });
  grid = motion({{{0, 0}, 0}, {{0, 0}, 0}});
  m.pel = 4;
  rejects([&] { TestDensePlan(m, 1, 1).generate(grid, 1); });
  m.delta = -2;
  rejects([&] { TestDensePlan(m, 1, 1).generate(grid, 2); });
  m.delta = 0;
  rejects([&] { TestDensePlan(m, 1, 1).generate(grid, -2); });
  m.delta = -1;
  rows(TestDensePlan(m, 1, 1).generate(grid, -2), {0, 0, 0, 0, 0, 0, 0, 0}, -2);
  rejects([&] { TestDensePlan({}, 1, 1); });
  rejects([&] { TestDensePlan(m, 0, 1); });
  rejects([&] { TestDensePlan(m, 1, 3); });
  m.overlap_x = 1;
  m.real_width = 6;
  rejects([&] { TestDensePlan(m, 2, 1); });
  m = metadata();
  m.real_width = 7;
  rejects([&] { TestDensePlan(m, 2, 1); });
  m = metadata();
  m.width = m.real_width = 9;
  rejects([&] { TestDensePlan(m, 1, 1); });
  m = metadata();
  m.width = m.real_width = 7;
  rejects([&] { TestDensePlan(m, 1, 1); });
}
} // namespace

int main() {
  try {
    examples_and_cropping();
    correction_and_saturation();
    storage_and_reuse();
    failures();
    std::cout << "Dense flow field specifications passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
