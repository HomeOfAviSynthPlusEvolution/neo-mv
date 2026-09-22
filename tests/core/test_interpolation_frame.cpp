#include "core/interpolation/frame.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/interpolation.hpp"
#endif

#include <iostream>
#include <string>

namespace {
using namespace neo_mv;
template <class T>
using TestKernels =
#if NEO_MV_TEST_HIGHWAY
    HighwayInterpolationKernels<T>;
#else
    ScalarInterpolationKernels<T>;
#endif
template <class T>
using Inter = InterpolationFramePlan<T, TestKernels<T>>;
template <class T>
using Blur = BlurFramePlan<T, TestKernels<T>>;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("interpolation frame assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call, const std::string& message = {}) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument& error) {
    caught = true;
    CHECK(message.empty() || std::string(error.what()).find(message) != std::string::npos);
  }
  CHECK(caught);
}
template <class T>
struct Image {
  SuperPlan<T> plan;
  std::vector<super_detail::PlaneBuffer<T>> storage;
  Image(SuperPlan<T> p, T base) : plan(std::move(p)) {
    const auto g = super_sampling_geometry(plan)[0];
    for (int k = 0; k < plan.geometry().plane_count; ++k)
      for (int a = 0; a < g.pel * g.pel; ++a) {
        const auto e = g.planes[k].reference[a];
        storage.emplace_back(e.width, e.height);
        auto v = storage.back().view();
        for (int y = 0; y < v.height(); ++y)
          for (int x = 0; x < v.width(); ++x)
            v.row(y)[x] = T(base + T(k * 16 + a));
      }
  }
  RenderImage<T> view() const {
    RenderImage<T> out{&plan, {}};
    std::size_t i = 0;
    for (int k = 0; k < plan.geometry().plane_count; ++k) {
      out.planes[k].pel = plan.params().pel;
      for (int a = 0; a < plan.params().pel * plan.params().pel; ++a)
        out.planes[k].planes[a] = storage[i++].view();
    }
    return out;
  }
  RenderPixels<T> pixels() const {
    RenderPixels<T> out{};
    const auto v = view();
    for (int k = 0; k < plan.geometry().plane_count; ++k) {
      const auto& p = plan.geometry().planes[k];
      out[k] = v.planes[k].planes[0].subplane(p.pad_x, p.pad_y, p.actual_width, p.actual_height);
    }
    return out;
  }
};
AnalysisField field(AnalysisMetadata m, MotionVector vector = {0, 0}) {
  return {m,
          FieldState::complete,
          {m.blocks_x, m.blocks_y, std::vector<MotionTriple>(std::size_t(m.blocks_x) * m.blocks_y, {vector, 0})}};
}
template <class T>
std::array<AnalysisMetadata, 2> metadata(const SuperPlan<T>& super) {
  std::array<AnalysisMetadata, 2> m{super_analysis_metadata(super, 1), super_analysis_metadata(super, -1)};
  m[0].bits = m[1].bits = 8; // Analysis precision is independent of render storage.
  return m;
}
template <class T>
void constant(const RenderOutput<T>& output, int k, T expected) {
  const auto view = output.at(k).view();
  for (int y = 0; y < view.height(); ++y)
    for (int x = 0; x < view.width(); ++x)
      CHECK(view.row(y)[x] == expected);
}
template <class T>
void complete_frames(int bits, bool chroma) {
  const int ratio = chroma ? 2 : 1;
  SuperPlan<T> super({8, 8, 8, 8, 0, 0, 4, 4, ratio, ratio, 1, chroma}, bits);
  const RenderVideo video{8, 8, bits, chroma, ratio, ratio, 3};
  const auto m = metadata(super);
  const InterpolationInputPlan<T> input(video, super, 3, m, {3, 3});
  const Inter<T> plan(input, video);
  const Blur<T> blur(input, video, 200, 1);
  Image<T> left(super, T{10}), right(super, T{21});
  auto B = field(m[0]), F = field(m[1]);
  const auto basic = plan.motion(B, F, nullptr, nullptr, left.view(), right.view(), 128);
  const auto extra = plan.motion(B, F, &B, &F, left.view(), right.view(), 128);
  const auto blended = plan.blend(left.pixels(), right.pixels(), 128);
  const auto blurred = blur.motion(B, F, left.view());
  for (int k = 0; k < (chroma ? 3 : 1); ++k) {
    const T expected = static_cast<T>((std::is_same_v<T, float> ? 15.5 : 15) + 16 * k);
    constant(basic, k, expected);
    constant(extra, k, expected);
    constant(blended, k, expected);
    constant(blurred, k, T(10 + 16 * k));
    CHECK(basic[k].view().width() == 8 / (k ? ratio : 1));
  }
  B.state = FieldState::metadata_only;
  B.grid = {0, 0, {}};
  rejects([&] { plan.motion(B, F, nullptr, nullptr, left.view(), right.view(), 128); });
  rejects([&] { blur.motion(B, F, left.view()); });
  B = field(m[0]);
  rejects([&] { plan.motion(B, F, &B, nullptr, left.view(), right.view(), 128); });
  auto bad = right.view();
  bad.planes[chroma ? 2 : 0].planes[0] = {};
  rejects([&] { plan.motion(B, F, nullptr, nullptr, left.view(), bad, 128); });
}
void negative_floor() {
  SuperPlan<std::uint8_t> super({8, 8, 8, 8, 0, 0, 4, 4, 1, 1, 2}, 8);
  const RenderVideo video{8, 8, 8, false, 1, 1, 3};
  const auto m = metadata(super);
  Inter<std::uint8_t> plan(InterpolationInputPlan<std::uint8_t>(video, super, 3, m, {3, 3}), video);
  Image<std::uint8_t> left(super, 10), right(super, 21);
  const auto B = field(m[0]), F = field(m[1], {-1, 0});
  // floor(-1 * 128 / 256) = -1 selects left phase1 (11), not phase0 (10).
  constant(plan.motion(B, F, nullptr, nullptr, left.view(), right.view(), 128), 0, std::uint8_t{16});
  constant(plan.motion(B, F, &B, &F, left.view(), right.view(), 128), 0, std::uint8_t{16});
}
void all_plane_preflight() {
  SuperPlan<float> super({16, 16, 8, 8, 0, 0, 3, 3, 2, 2, 2, true}, 32);
  const RenderVideo video{16, 16, 32, true, 2, 2, 3};
  const auto m = metadata(super);
  const InterpolationInputPlan<float> input(video, super, 3, m, {3, 3});
  const Inter<float> interpolation(input, video);
  const Blur<float> blur(input, video, 200, 1);
  Image<float> left(super, 10), right(super, 21);
  const auto B = field(m[0]), F = field(m[1], {-6, 0});
  auto y = left.storage[0].view();
  for (int row = 0; row < y.height(); ++row)
    std::fill_n(y.row(row).data(), y.width(), std::numeric_limits<float>::quiet_NaN());
  // Y is in bounds but non-finite; chroma's -3 plane-pel displacement is
  // out of bounds with floor(pad/2)=1. Domain failure must precede reading Y.
  rejects([&] { interpolation.motion(B, F, nullptr, nullptr, left.view(), right.view(), 256); },
          "logical phase domain");
  rejects([&] { blur.motion(B, F, left.view()); }, "logical phase domain");
}
template <class T>
void blur_trajectory(int bits) {
  SuperPlan<T> super({8, 8, 8, 8, 0, 0, 4, 4, 1, 1, 1}, bits);
  const RenderVideo video{8, 8, bits, false, 1, 1, 3};
  const auto m = metadata(super);
  const InterpolationInputPlan<T> input(video, super, 3, m, {3, 3});
  Image<T> image(super, 0);
  auto view = image.storage[0].view();
  for (int y = 0; y < view.height(); ++y)
    for (int x = 0; x < view.width(); ++x)
      view.row(y)[x] = T(x * x);
  const auto B = field(m[0], {-2, 0}), F = field(m[1], {2, 0});
  const auto full = Blur<T>(input, video, 200, 1).motion(B, F, image.view());
  const auto short_path = Blur<T>(input, video, 50, 1).motion(B, F, image.view());
  const auto coarse = Blur<T>(input, video, 200, 3).motion(B, F, image.view());
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 8; ++x) {
      // Mean of squares at center,+1,+2,-1,-2 is center^2+2.
      const int center = (x + 4) * (x + 4);
      CHECK(full[0].view().row(y)[x] == T(center + 2));
      CHECK(short_path[0].view().row(y)[x] == T(center));
      CHECK(coarse[0].view().row(y)[x] == T(center));
    }
}
void representations_and_equal_blend() {
  SuperPlan<float> super({8, 8, 8, 8, 0, 0, 4, 4, 1, 1, 1}, 32);
  const RenderVideo video{8, 8, 32, false, 1, 1, 3};
  const auto m = metadata(super);
  const InterpolationInputPlan<float> input(video, super, 3, m, {3, 3});
  const Inter<float> plan(input, video);
  Image<float> image(super, 0);
  const auto B = field(m[0]), F = field(m[1]);
  const std::uint32_t patterns[] = {0x7fa12345, 0x7fc54321, 0x80000000, 0, 0x7f800000, 0xff800000, 1, 0xbf800000};
  auto view = image.storage[0].view();
  for (int y = 0; y < 8; ++y)
    std::memcpy(view.row(y + 4).data() + 4, patterns, sizeof(patterns));
  const auto copied = plan.copy(image.pixels());
  const auto center_only = Blur<float>(input, video, 0, 1).motion(B, F, image.view());
  for (int y = 0; y < 8; ++y) {
    CHECK(std::memcmp(copied[0].view().row(y).data(), patterns, sizeof(patterns)) == 0);
    CHECK(std::memcmp(center_only[0].view().row(y).data(), patterns, sizeof(patterns)) == 0);
  }
  rejects([&] { plan.blend(image.pixels(), image.pixels(), 0); });
  rejects([&] { plan.motion(B, F, nullptr, nullptr, image.view(), image.view(), 0); });
  for (int y = 0; y < 8; ++y)
    std::fill_n(view.row(y + 4).data() + 4, 8, 1.75f + 0x1p-23f);
  constant(plan.blend(image.pixels(), image.pixels(), 85), 0, 1.75f + 0x1p-22f);
  constant(plan.copy(image.pixels()), 0, 1.75f + 0x1p-23f);
}
struct CountingKernels : TestKernels<std::uint8_t> {
  inline static int calls = 0;
  static std::int64_t scene_count(const AnalysisMetadata& m, const MotionGrid& grid, std::int64_t threshold) {
    ++calls;
    return TestKernels<std::uint8_t>::scene_count(m, grid, threshold);
  }
};
void scene_dispatch() {
  const SuperPlan<std::uint8_t> super({8, 8, 8, 8, 0, 0, 4, 4}, 8);
  const RenderVideo video{8, 8, 8, false, 1, 1, 3};
  const auto m = metadata(super);
  const InterpolationInputPlan<std::uint8_t> input(video, super, 3, m, {3, 3});
  const InterpolationFramePlan<std::uint8_t, CountingKernels> plan(input, video);
  auto B = field(m[0]), F = field(m[1]);
  B.grid.values.front().error = INT64_MAX;
  F.grid.values.back().error = -1;
  CountingKernels::calls = 0;
  // Scene rejection of the first field cannot hide corrupt second/extra fields.
  rejects([&] { plan.main_eligible(B, F); });
  CHECK(CountingKernels::calls == 2);
  rejects([&] { plan.extra_eligible(B, F); });
  CHECK(CountingKernels::calls == 4);
  B.state = FieldState::metadata_only;
  rejects([&] { plan.main_eligible(B, F); });
  CHECK(CountingKernels::calls == 5);
  const BlurFramePlan<std::uint8_t, CountingKernels> blur(input, video, 0, 1);
  rejects([&] { blur.main_eligible(B, F); });
  CHECK(CountingKernels::calls == 6);
}
} // namespace
int main() {
  try {
    complete_frames<std::uint8_t>(8, false);
    complete_frames<std::uint8_t>(8, true);
    complete_frames<std::uint16_t>(10, true);
    complete_frames<std::uint16_t>(16, true);
    complete_frames<float>(32, true);
    scene_dispatch();
    negative_floor();
    all_plane_preflight();
    blur_trajectory<std::uint8_t>(8);
    blur_trajectory<std::uint16_t>(16);
    blur_trajectory<float>(32);
    representations_and_equal_blend();
    std::cout << "Interpolation frame specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
