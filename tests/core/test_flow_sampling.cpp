#include "core/flow/sampling.hpp"
#include "core/super/pyramid.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/flow_sampling.hpp"
#include "hwy/targets.h"
#endif

#include <iostream>

namespace {
using namespace neo_mv;
using TestPlan =
#if NEO_MV_TEST_HIGHWAY
    simd::FlowSamplingPlan;
#else
    FlowSamplingPlan;
#endif
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Flow sampling assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
template <class T>
struct Image {
  RenderPhaseGeometry geometry;
  std::vector<super_detail::PlaneBuffer<T>> storage;
  Image(int width, int height, int pad, int pel, bool quarter = false) : geometry{pel, 1, 1, pad, pad, {}} {
    for (int a = 0; a < pel * pel; ++a) {
      const int w = width + 2 * pad - (quarter && a % pel == 3);
      const int h = height + 2 * pad - (quarter && a / pel == 3);
      geometry.phases[a] = {w, h};
      storage.emplace_back(w, h);
      auto plane = storage.back().view();
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
          plane.row(y)[x] = T(a * 37 + y * 11 + x);
    }
  }
  SubpixelPhases<T> view() const {
    SubpixelPhases<T> out;
    out.pel = geometry.pel;
    for (std::size_t i = 0; i < storage.size(); ++i)
      out.planes[i] = storage[i].view();
    return out;
  }
  T at(int phase, int x, int y) const { return storage[phase].view().row(y)[x]; }
};
DenseFlowField dense(int width, int height) {
  return {width, height, std::vector<std::int16_t>(std::size_t(width) * height),
          std::vector<std::int16_t>(std::size_t(width) * height)};
}
template <class T>
void rounding() {
  Image<T> image(3, 1, 2, 2);
  auto field = dense(3, 1);
  field.x = {1, -1, -3};
  std::vector<T> output(7, T{19});
  auto view = checked_plane(output.data() + 1, 3, 1, 6 * sizeof(T), 6 * sizeof(T));
  TestPlan(image.geometry, 3, 1, 128).sample(field, image.view(), view);
  CHECK(output[1] == image.at(1, 2, 2));
  CHECK(output[2] == image.at(0, 3, 2));
  CHECK(output[3] == image.at(1, 3, 2));
  CHECK(output.front() == T{19} && output[4] == T{19} && output.back() == T{19});
  field.x = {-3, 0, 0};
  TestPlan(image.geometry, 3, 1, 256).sample(field, image.view(), view);
  CHECK(output[1] == image.at(1, 0, 2));
  field.x = {INT16_MAX, INT16_MIN, INT16_MAX};
  field.y = {INT16_MIN, INT16_MAX, INT16_MIN};
  TestPlan(image.geometry, 3, 1, 0).sample(field, image.view(), view);
  for (int x = 0; x < 3; ++x)
    CHECK(output[x + 1] == image.at(0, x + 2, 2));
}
void negative_rounding_boundaries() {
  // For time128, odd negative components yield exact negative multiples
  // after adding128. Even neighbors exercise either side of those boundaries.
  const std::int16_t components[] = {-7, -6, -5, -4, -3, -2, -1, 0, 1};
  const int half_offsets[] = {-3, -3, -2, -2, -1, -1, 0, 0, 1};
  for (const int pel : {1, 2, 4}) {
    Image<std::uint16_t> image(1, 1, 8, pel);
    auto field = dense(1, 1);
    std::uint16_t pixel = 0;
    const auto out = checked_plane(&pixel, 1, 1, sizeof(pixel), sizeof(pixel));
    for (const int time : {128, 256})
      for (int i = 0; i < 9; ++i)
        for (const bool vertical : {false, true}) {
          field.x[0] = vertical ? 0 : components[i];
          field.y[0] = vertical ? components[i] : 0;
          const int offset = time == 128 ? half_offsets[i] : components[i];
          int integer = 0;
          while (integer * pel > offset)
            --integer;
          while ((integer + 1) * pel <= offset)
            ++integer;
          const int fraction = offset - integer * pel;
          TestPlan(image.geometry, 1, 1, time).sample(field, image.view(), out);
          CHECK(pixel == image.at(vertical ? fraction * pel : fraction, 8 + (vertical ? 0 : integer),
                                  8 + (vertical ? integer : 0)));
        }
  }
}
void wide_preflight() {
  RenderPhaseGeometry g{4, 1, 1, INT32_MAX - 1, INT32_MAX - 1, {}};
  for (auto& phase : g.phases)
    phase = {INT32_MAX, INT32_MAX};
  auto f = dense(1, 1);
  TestPlan plan(g, 1, 1, 256);
  plan.preflight(f);
  f.x[0] = 4;
  rejects([&] { plan.preflight(f); });
  f.x[0] = INT16_MIN;
  f.y[0] = INT16_MIN;
  plan.preflight(f);
}
void admission_and_errors() {
  Image<std::uint8_t> image(3, 1, 1, 2);
  auto field = dense(3, 1);
  field.x[0] = -3;
  TestPlan accepted(image.geometry, 3, 1, 256); // No whole-domain admission.
  rejects([&] { accepted.preflight(field); });
  std::uint8_t pixels[] = {19, 19, 19};
  auto output = checked_plane(pixels, 3, 1, 3, sizeof(pixels));
  field.x = {0, 0, INT16_MAX};
  rejects([&] { accepted.sample(field, image.view(), output); });
  CHECK(pixels[0] == 19 && pixels[1] == 19 && pixels[2] == 19);
  field.x[2] = 0;
  auto bad = image.view();
  bad.planes[3] = {}; // Even an unused declared phase must have valid storage.
  rejects([&] { accepted.sample(field, bad, output); });
  auto alias = image.storage[0].view().subplane(1, 1, 3, 1);
  rejects([&] { accepted.sample(field, image.view(), alias); });
  field.y.clear();
  rejects([&] { accepted.preflight(field); });
  auto broken = image.geometry;
  broken.phases[0].width = 3;
  rejects([&] { TestPlan bad_plan(broken, 3, 1, 0); });
  rejects([&] { TestPlan bad_plan(image.geometry, 3, 1, -1); });
  rejects([&] { TestPlan bad_plan(image.geometry, 3, 1, 257); });

  Image<std::uint8_t> quarter(1, 1, 1, 4, true), external(1, 1, 1, 4);
  auto edge = dense(1, 1);
  edge.x[0] = 7; // phase 3, logical column 2, excluded by built-in quarter phase.
  rejects([&] { TestPlan(quarter.geometry, 1, 1, 256).preflight(edge); });
  auto one = output.subplane(0, 0, 1, 1);
  TestPlan(external.geometry, 1, 1, 256).sample(edge, external.view(), one);
  CHECK(pixels[0] == external.at(3, 2, 1));
}
void phase_extent_boundaries() {
  // Exercise every lane and phase at both sides of its exact logical limits,
  // including external phase layouts that cannot use the separable shortcut.
  for (const int pel : {1, 2, 4})
    for (const bool irregular : {false, true}) {
      Image<std::uint8_t> image(17, 1, 2, pel, pel == 4);
      auto geometry = image.geometry;
      if (irregular)
        for (int a = 1; a < pel * pel; ++a)
          geometry.phases[a] = {20 - a % 3, 4 + a % 2};
      for (int phase = 0; phase < pel * pel; ++phase)
        for (int lane = 0; lane < 17; ++lane)
          for (const bool vertical : {false, true})
            for (const bool outside : {false, true}) {
              auto field = dense(17, 1);
              const int column = vertical ? 2 : geometry.phases[phase].width - !outside;
              const int row = vertical ? geometry.phases[phase].height - !outside : 2;
              field.x[lane] = static_cast<std::int16_t>((column - 2 - lane) * pel + phase % pel);
              field.y[lane] = static_cast<std::int16_t>((row - 2) * pel + phase / pel);
              auto scalar = [&] {
                neo_mv::FlowSamplingPlan(geometry, 17, 1, 256).preflight(field);
              };
              auto tested = [&] {
                TestPlan(geometry, 17, 1, 256).preflight(field);
              };
              if (outside) {
                rejects(scalar);
                rejects(tested);
              } else {
                scalar();
                tested();
              }
            }
    }
}
void float_representation() {
  Image<float> image(8, 2, 1, 1);
  auto field = dense(8, 2);
  const std::uint32_t patterns[] = {0x7fc12345, 0x7fa12345, 0x80000000, 0, 0x7f800000, 0xff800000, 1, 0xbf800000};
  auto plane = image.storage[0].view();
  for (int y = 0; y < 2; ++y)
    std::memcpy(plane.row(y + 1).data() + 1, patterns, sizeof(patterns));
  std::vector<float> output(11 * 2, 19);
  auto view = checked_plane(output.data(), 8, 2, 11 * sizeof(float), output.size() * sizeof(float));
  TestPlan(image.geometry, 8, 2, 256).sample(field, image.view(), view);
  for (int y = 0; y < 2; ++y) {
    CHECK(std::memcmp(view.row(y).data(), patterns, sizeof(patterns)) == 0);
    for (int x = 8; x < 11; ++x)
      CHECK(output[y * 11 + x] == 19);
  }
}
#if NEO_MV_TEST_HIGHWAY
template <class T>
void vector_boundaries() {
  for (const int width : {1, 2, 3, 4, 7, 8, 9, 15, 16, 17, 31, 32, 33, 65, 129})
    for (const int pel : {1, 2, 4}) {
      Image<T> image(width, 3, 16, pel, pel == 4);
      auto field = dense(width, 3);
      for (std::size_t i = 0; i < field.x.size(); ++i) {
        field.x[i] = static_cast<std::int16_t>(int(i % 31) - 15);
        field.y[i] = static_cast<std::int16_t>(15 - int(i % 31));
      }
      const auto stride = width + 3;
      std::vector<T> a(stride * 3, T{19}), b = a;
      auto av = checked_plane(a.data(), width, 3, stride * sizeof(T), a.size() * sizeof(T));
      auto bv = checked_plane(b.data(), width, 3, stride * sizeof(T), b.size() * sizeof(T));
      for (int time = 0; time <= 256; ++time) {
        neo_mv::FlowSamplingPlan(image.geometry, width, 3, time).sample(field, image.view(), av);
        TestPlan(image.geometry, width, 3, time).sample(field, image.view(), bv);
        CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
      }
      // Each lane, including final partial groups, must fail before any write.
      for (int x = 0; x < width; ++x) {
        const auto saved = field.x[2 * width + x];
        field.x[2 * width + x] = INT16_MAX;
        std::fill(b.begin(), b.end(), T{19});
        rejects([&] { TestPlan(image.geometry, width, 3, 256).sample(field, image.view(), bv); });
        for (const auto value : b)
          CHECK(value == T{19});
        field.x[2 * width + x] = saved;
      }
      field.x.assign(field.x.size(), INT16_MIN);
      field.y.assign(field.y.size(), INT16_MAX);
      neo_mv::FlowSamplingPlan(image.geometry, width, 3, 0).sample(field, image.view(), av);
      TestPlan(image.geometry, width, 3, 0).sample(field, image.view(), bv);
      for (int y = 0; y < 3; ++y)
        CHECK(std::memcmp(av.row(y).data(), bv.row(y).data(), width * sizeof(T)) == 0);
    }
}
#endif
} // namespace
int main() {
  try {
#if NEO_MV_TEST_HIGHWAY
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << "Testing " << hwy::TargetName(target) << '\n';
      vector_boundaries<std::uint8_t>();
      vector_boundaries<std::uint16_t>();
      vector_boundaries<float>();
#endif
      rounding<std::uint8_t>();
      rounding<std::uint16_t>();
      rounding<float>();
      negative_rounding_boundaries();
      wide_preflight();
      admission_and_errors();
      float_representation();
      phase_extent_boundaries();
#if NEO_MV_TEST_HIGHWAY
    }
    hwy::SetSupportedTargetsForTest(0);
#endif
    std::cout << "Flow sampling specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
