#include "core/mask/frame.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/mask.hpp"
#endif

#include <iostream>

namespace {
using namespace neo_mv;
template <class T>
using TestKernels =
#if NEO_MV_TEST_HIGHWAY
    HighwayMaskKernels<T>;
#else
    ScalarMaskKernels<T>;
#endif
template <class T>
using TestMaskFrame = MaskFramePlan<T, TestKernels<T>>;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("mask frame assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E = std::invalid_argument, class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const E&) {
    caught = true;
  }
  CHECK(caught);
}
template <class T>
struct Buffer {
  int width, height, stride;
  std::vector<T> data;
  Buffer(int w, int h) : width(w), height(h), stride(w + 3), data(std::size_t(stride) * h + 1, T{19}) {}
  span2d::Plane<T> view() {
    return checked_plane(data.data() + 1, width, height, std::ptrdiff_t(stride) * sizeof(T),
                         (data.size() - 1) * sizeof(T));
  }
  void constant(T expected) {
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        CHECK(view().row(y)[x] == expected);
    guards();
  }
  void guards() const {
    CHECK(data[0] == T{19});
    for (int y = 0; y < height; ++y)
      for (int x = width; x < stride; ++x)
        CHECK(data[1 + std::size_t(y) * stride + x] == T{19});
  }
};
AnalysisField field(int bits = 8, int nx = 1, int ny = 1, int block = 8) {
  AnalysisMetadata m;
  m.width = m.real_width = nx * block;
  m.height = m.real_height = ny * block;
  m.pad_x = m.pad_y = 8;
  m.block_width = m.block_height = block;
  m.blocks_x = nx;
  m.blocks_y = ny;
  m.pel = m.levels = m.ratio_x = m.ratio_y = 1;
  m.delta = 1;
  m.bits = bits;
  return {m, FieldState::complete, {nx, ny, std::vector<MotionTriple>(std::size_t(nx) * ny, {{0, 0}, 0})}};
}

template <class T>
void precision(int bits) {
  auto input = field(bits);
  input.grid.values[0].vector = {4, 0};
  MaskParameters p;
  p.ml = 8;
  p.gamma = 2;
  p.scval = 20.5;
  const TestMaskFrame<T> plan(MaskKind::VectorLength, input.metadata, 2, p);
  CHECK(plan.metadata().bits == bits && plan.metadata().real_width == 8);
  Buffer<T> output(8, 8);
  plan.render(input, output.view());
  if constexpr (std::is_same_v<T, float>)
    output.constant(0.25f);
  else
    output.constant(static_cast<T>(((std::uint32_t{1} << bits) - 1) / 4));
  input.state = FieldState::metadata_only;
  input.grid.values.clear();
  plan.render(input, output.view());
  output.constant(std::is_same_v<T, float> ? T(20.5) : T(21));
}

void examples() {
  precision<std::uint8_t>(8);
  for (int bits = 9; bits <= 16; ++bits)
    precision<std::uint16_t>(bits);
  precision<float>(32);
  auto input = field();
  input.grid.values[0].error = 8;
  MaskParameters p;
  p.ml = 1;
  p.scval = 9;
  const TestMaskFrame<std::uint8_t> sad(MaskKind::SAD, input.metadata, 1, p);
  Buffer<std::uint8_t> output(8, 8);
  sad.render(input, output.view());
  output.constant(127);
  input.state = FieldState::metadata_only;
  sad.render(input, output.view());
  output.constant(9);

  input = field(8, 2, 1, 4);
  input.metadata.delta = -1;
  input.grid.values[0].vector = {4, 0};
  p = {};
  p.ml = 80;
  const TestMaskFrame<std::uint8_t> occ(MaskKind::Occlusion, input.metadata, 1, p);
  Buffer<std::uint8_t> occlusion(8, 4);
  occ.render(input, occlusion.view());
  const int expected[] = {255, 255, 223, 159, 96, 32, 0, 0};
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 8; ++x)
      CHECK(occlusion.view().row(y)[x] == expected[x]);
  occlusion.guards();
  // Resize using the covered 8-wide geometry, then crop to the real width.
  input.metadata.real_width = 6;
  const TestMaskFrame<std::uint8_t> crop(MaskKind::Occlusion, input.metadata, 1, p);
  Buffer<std::uint8_t> cropped(6, 4);
  crop.render(input, cropped.view());
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 6; ++x)
      CHECK(cropped.view().row(y)[x] == expected[x]);
  cropped.guards();
}

void fallback_and_errors() {
  auto input = field();
  MaskParameters p;
  p.gamma = 0;
  p.scval = 12.5;
  Buffer<std::uint8_t> output(8, 8);
  for (auto kind : {MaskKind::VectorLength, MaskKind::SAD, MaskKind::Occlusion}) {
    const TestMaskFrame<std::uint8_t> plan(kind, input.metadata, 1, p);
    auto current = input;
    current.state = FieldState::invalid_metadata;
    current.metadata = {}; // Output shape still comes from the saved descriptor.
    plan.render(current, output.view());
    output.constant(13);
    current = input;
    current.grid.values[0].error = INT64_MAX; // Scene cut bypasses score generation.
    plan.render(current, output.view());
    output.constant(13);
    current.grid.values[0].error = -1;
    rejects([&] { plan.render(current, output.view()); });
    output.constant(13);
    current = input;
    current.metadata.delta = -1;
    current.state = FieldState::metadata_only;
    rejects([&] { plan.render(current, output.view()); });
    output.constant(13);
    current = input;
    current.grid.values.clear();
    rejects([&] { plan.render(current, output.view()); });
    current = input;
    current.metadata.levels = 3;
    plan.render(current, output.view()); // Positive Levels changes are allowed.
  }
  // Reusing one immutable plan after a different input does not retain grids.
  p = {};
  p.ml = 8;
  p.gamma = 2;
  const TestMaskFrame<std::uint8_t> length(MaskKind::VectorLength, input.metadata, 1, p);
  input.grid.values[0].vector = {4, 0};
  length.render(input, output.view());
  output.constant(63);
  input.grid.values[0].vector = {0, 0};
  length.render(input, output.view());
  output.constant(0);
  input.grid.values[0].vector = {4, 0};
  length.render(input, output.view());
  output.constant(63);
  p.ml = 1e-37;
  p.gamma = 1;
  const TestMaskFrame<std::uint8_t> overflow(MaskKind::SAD, input.metadata, 1, p);
  input.grid.values[0].error = 8; // Still scene-eligible, but score overflows.
  rejects<std::overflow_error>([&] { overflow.render(input, output.view()); });
  output.constant(63);

  const auto fp = field(32);
  p = {};
  p.scval = -0.0;
  const TestMaskFrame<float> float_plan(MaskKind::SAD, fp.metadata, 1, p);
  auto missing = fp;
  missing.state = FieldState::metadata_only;
  Buffer<float> float_output(8, 8);
  float_plan.render(missing, float_output.view());
  float_output.constant(0);
  CHECK(std::signbit(float_output.view().row(0)[0]));
  p.scval = 1.25;
  TestMaskFrame<float>(MaskKind::Occlusion, fp.metadata, 1, p).render(missing, float_output.view());
  float_output.constant(1.25f);
}

void creation_and_views() {
  const auto input = field();
  MaskParameters p;
  p.ml = 1e-30;
  rejects<std::overflow_error>([&] { TestMaskFrame<std::uint8_t> plan(MaskKind::VectorLength, input.metadata, 1, p); });
  // Only the chosen operator's constants are evaluated during construction.
  const TestMaskFrame<std::uint8_t> sad(MaskKind::SAD, input.metadata, 1, p);
  const TestMaskFrame<std::uint8_t> occ(MaskKind::Occlusion, input.metadata, 1, p);
  rejects([&] { TestMaskFrame<std::uint8_t> plan(static_cast<MaskKind>(99), input.metadata, 1); });
  rejects([&] { TestMaskFrame<std::uint8_t> plan(MaskKind::SAD, input.metadata, 0); });
  rejects([&] { TestMaskFrame<float> plan(MaskKind::SAD, input.metadata, 1); });
  const TestMaskFrame<std::uint8_t> plan(MaskKind::SAD, input.metadata, 1);
  Buffer<std::uint8_t> wrong(7, 8);
  rejects([&] { plan.render(input, wrong.view()); });
  wrong.constant(19);
  rejects([&] { plan.render(input, {}); });
  auto missing = input;
  missing.state = FieldState::metadata_only;
  rejects([&] { plan.render(missing, wrong.view()); });
  auto small = field(8, 1, 1, 2);
  const TestMaskFrame<std::uint8_t> small_plan(MaskKind::SAD, small.metadata, 1);
  auto* bytes = reinterpret_cast<std::uint8_t*>(small.grid.values.data());
  const auto alias = checked_plane(bytes, 2, 2, 2, sizeof(MotionTriple));
  rejects([&] { small_plan.render(small, alias); });
  small.state = FieldState::metadata_only;
  rejects([&] { small_plan.render(small, alias); });
  auto* metadata_bytes = reinterpret_cast<std::uint8_t*>(&small);
  rejects([&] { small_plan.render(small, checked_plane(metadata_bytes, 2, 2, 2, sizeof(small))); });
}
} // namespace

int main() {
  try {
    examples();
    fallback_and_errors();
    creation_and_views();
    std::cout << "mask frame specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
