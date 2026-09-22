#include "core/interpolation/blur.hpp"
#include "core/super/pyramid.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/blur_sampling.hpp"
#include "hwy/targets.h"
#endif

#include <iostream>
#include <string>
#include <utility>

namespace {
using namespace neo_mv;
using TestPlan =
#if NEO_MV_TEST_HIGHWAY
    simd::BlurSamplingPlan;
#else
    BlurSamplingPlan;
#endif
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("blur assertion at " + std::to_string(line));
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
RenderPhaseGeometry geometry(int width, int height, int pad, int pel) {
  RenderPhaseGeometry g{pel, 1, 1, pad, pad, {}};
  for (int a = 0; a < pel * pel; ++a)
    g.phases[a] = {width + 2 * pad, height + 2 * pad};
  return g;
}
DenseFlowField field(int width, int height, std::int16_t x, std::int16_t y) {
  const auto count = std::size_t(width) * height;
  return {width, height, std::vector<std::int16_t>(count, x), std::vector<std::int16_t>(count, y)};
}
using Point = std::pair<std::int64_t, std::int64_t>;
std::vector<Point> points(const TestPlan& plan, int fx, int fy, int bx, int by) {
  std::vector<Point> out;
  const auto count =
      plan.trajectory(0, 0, static_cast<std::int16_t>(fx), static_cast<std::int16_t>(fy), static_cast<std::int16_t>(bx),
                      static_cast<std::int16_t>(by), [&](auto x, auto y) { out.emplace_back(x, y); });
  CHECK(count == out.size());
  return out;
}
void trajectories() {
  const auto g = geometry(1, 1, 4, 1);
  CHECK(blur_time_coefficient(0) == 0 && blur_time_coefficient(50) == 64 && blur_time_coefficient(200) == 256);
  CHECK(blur_time_coefficient(std::nextafter(200.0f, 0.0f)) == 255);
  CHECK(points(TestPlan(g, 1, 1, 1, 256), 2, 0, -2, 0) ==
        (std::vector<Point>{{0, 0}, {1, 0}, {2, 0}, {-1, 0}, {-2, 0}}));
  CHECK(points(TestPlan(g, 1, 1, 1, 64), 2, 0, -2, 0) == (std::vector<Point>{{0, 0}}));
  CHECK(points(TestPlan(g, 1, 1, 3, 256), 2, 0, -2, 0) == (std::vector<Point>{{0, 0}}));
  CHECK(points(TestPlan(g, 1, 1, 1, 1), -257, 514, 0, 0) == (std::vector<Point>{{0, 0}, {-1, 1}, {-1, 2}}));
  CHECK(points(TestPlan(g, 1, 1, 1, 256), 1, 0, 1, 0) ==
        (std::vector<Point>{{0, 0}, {1, 0}, {1, 0}})); // Repeated points remain two contributions.
  const auto maximum = points(TestPlan(g, 1, 1, 1, 256), INT16_MIN, INT16_MAX, INT16_MIN, 0);
  CHECK(maximum.size() == 65537 && maximum[32768].first == INT16_MIN && maximum.back().first == INT16_MIN);
  CHECK(points(TestPlan(g, 1, 1, INT32_MAX, 256), INT16_MIN, INT16_MAX, INT16_MIN, 0).size() == 1);
  rejects([&] { TestPlan(g, 1, 1, 0, 256); });
  rejects([&] { TestPlan(g, 1, 1, 1, 257); });
  rejects([&] { TestPlan(g, 1, 1, 1, -1); });
  rejects([&] { TestPlan(g, 20, 1, 1, 0); });
  rejects([] { blur_time_coefficient(-1); });
  rejects([] { blur_time_coefficient(201); });
  rejects([] { blur_time_coefficient(std::numeric_limits<float>::quiet_NaN()); });
}
template <class T>
T average(std::initializer_list<T> values, int bits) {
  T result{};
  ordered_blur_average(values.begin(), values.size(), bits, &result);
  return result;
}
void averages() {
  CHECK(average<std::uint8_t>({10, 20, 30, 0, 1}, 8) == 12);
  CHECK(average<std::uint16_t>({1, 2}, 16) == 1);
  CHECK(average<float>({10, 20, 30, 0, 1}, 32) == 12.2f);
  CHECK(average<float>({1, 2}, 32) == 1.5f);
  CHECK(average<float>({0x1p60f, 1, -0x1p60f}, 32) == 0);
  CHECK(average<float>({0x1p60f, -0x1p60f, 1}, 32) == float(1.0 / 3.0));
  std::vector<std::uint16_t> maximal(65537, 65535);
  std::uint16_t output = 0;
  ordered_blur_average(maximal.data(), maximal.size(), 16, &output);
  CHECK(output == 65535); // Exact sum UINT32_MAX, no signed32 accumulator.
  const std::uint16_t invalid[] = {1023, 1024};
  rejects([&] { ordered_blur_average(invalid, 2, 10, &output); });
  rejects([&] { ordered_blur_average(invalid, 0, 10, &output); });
  rejects([&] { ordered_blur_average(invalid, 65538, 10, &output); });
  rejects([&] { ordered_blur_average(invalid, 2, 32, &output); });
  for (const auto pattern : {0x80000000u, 0x7fa12345u, 0x7fc12345u, 0x7f800000u, 0xff800000u}) {
    float input, result;
    std::memcpy(&input, &pattern, sizeof(input));
    ordered_blur_average(&input, 1, 32, &result);
    CHECK(std::memcmp(&input, &result, sizeof(input)) == 0);
    if (pattern != 0x80000000u)
      rejects([&] { average<float>({1, input}, 32); });
  }
}
template <class T>
void sampling(int bits) {
  const auto g = geometry(1, 1, 4, 1);
  super_detail::PlaneBuffer<T> storage(9, 9);
  auto source = storage.view();
  const T samples[] = {10, 20, 30, 0, 1};
  const int positions[] = {4, 5, 6, 3, 2};
  for (int i = 0; i < 5; ++i)
    source.row(4)[positions[i]] = samples[i];
  SubpixelPhases<T> phases{1, {}};
  phases.planes[0] = source;
  T pixels[] = {T(19), T(19), T(19)};
  auto output = checked_plane(pixels + 1, 1, 1, 2 * sizeof(T), 2 * sizeof(T));
  const auto f = field(1, 1, 2, 0), b = field(1, 1, -2, 0);
  TestPlan(g, 1, 1, 1, 256).sample(f, b, phases, output, bits);
  CHECK(pixels[1] == (std::is_same_v<T, float> ? T(12.2f) : T(12)));
  CHECK(pixels[0] == T(19) && pixels[2] == T(19));
  TestPlan(g, 1, 1, 1, 0).sample(f, b, phases, output, bits);
  CHECK(pixels[1] == T(10));
  rejects([&] { TestPlan(g, 1, 1, 1, 256).sample(f, b, phases, source.subplane(4, 4, 1, 1), bits); });
  auto bad = phases;
  bad.planes[0] = {};
  rejects([&] { TestPlan(g, 1, 1, 1, 0).sample(f, b, bad, output, bits); });
}
void domains_and_copy() {
  const auto f = field(1, 1, -3, 0), b = field(1, 1, 0, 0);
  const auto g = geometry(1, 1, 1, 2);
  rejects([&] { TestPlan(g, 1, 1, 1, 256).preflight(f, b); });
  TestPlan(g, 1, 1, 1, 0).preflight(f, b);
  TestPlan(geometry(1, 1, 2, 2), 1, 1, 1, 256).preflight(f, b);
  auto malformed = b;
  malformed.y.clear();
  rejects([&] { TestPlan(g, 1, 1, 1, 0).preflight(f, malformed); });
  auto quarter = geometry(1, 1, 1, 4);
  const auto right = field(1, 1, 7, 0);
  TestPlan(quarter, 1, 1, 1, 256).preflight(right, b);
  for (int a = 0; a < 16; ++a)
    if (a % 4 == 3)
      --quarter.phases[a].width;
  rejects([&] { TestPlan(quarter, 1, 1, 1, 256).preflight(right, b); });
  const auto flat = geometry(2, 1, 0, 1);
  const std::uint32_t patterns[] = {0x80000000, 0x7fa12345};
  float input[2], output[2] = {};
  std::memcpy(input, patterns, sizeof(input));
  SubpixelPhases<float> source{1, {}};
  source.planes[0] = checked_plane(static_cast<const float*>(input), 2, 1, sizeof(input), sizeof(input));
  const auto zeros = field(2, 1, 0, 0);
  TestPlan(flat, 2, 1, 1, 0)
      .sample(zeros, zeros, source, checked_plane(output, 2, 1, sizeof(output), sizeof(output)), 32);
  CHECK(std::memcmp(input, output, sizeof(input)) == 0);
  auto unsafe = zeros;
  unsafe.x[1] = 1;
  output[0] = output[1] = 19;
  rejects([&] {
    TestPlan(flat, 2, 1, 1, 256)
        .sample(unsafe, zeros, source, checked_plane(output, 2, 1, sizeof(output), sizeof(output)), 32);
  });
  CHECK(output[0] == 19 && output[1] == 19); // Last pixel preflight precedes the first sample.
}
#if NEO_MV_TEST_HIGHWAY
template <class T>
void vertical_sequences(int bits) {
  for (int pel : {1, 2, 4})
    for (int length : {1, 7, 8, 9, 15, 16, 17, 31, 32, 33, 65}) {
      const int pad = (length + pel - 1) / pel + 2;
      auto g = geometry(1, 1, pad, pel);
      SubpixelPhases<T> source{pel, {}};
      std::vector<std::vector<T>> storage(pel * pel);
      for (int a = 0; a < pel * pel; ++a) {
        if (pel == 4 && a % pel == 3)
          --g.phases[a].width;
        if (pel == 4 && a / pel == 3)
          --g.phases[a].height;
        const auto e = g.phases[a];
        const int stride = e.width + 3 + a;
        storage[a].assign(std::size_t(stride) * e.height, T(19));
        auto view = checked_plane(storage[a].data(), e.width, e.height, stride * sizeof(T), storage[a].size() * sizeof(T));
        for (int y = 0; y < e.height; ++y)
          for (int x = 0; x < e.width; ++x)
            view.row(y)[x] = T((x * 11 + y * 17 + a * 29) % 251);
        source.planes[a] = view;
      }
      const auto f = field(1, 1, static_cast<std::int16_t>(length / 2), static_cast<std::int16_t>(-length));
      const auto b = field(1, 1, static_cast<std::int16_t>(-length), static_cast<std::int16_t>(length));
      for (int time : {127, 256}) {
        std::vector<T> a, b_samples;
        T output = 19;
        const auto out = checked_plane(&output, 1, 1, sizeof(T), sizeof(T));
        auto save_a = [&](const T* values, std::size_t count, int, T*) { a.assign(values, values + count); };
        auto save_b = [&](const T* values, std::size_t count, int, T*) { b_samples.assign(values, values + count); };
        neo_mv::BlurSamplingPlan(g, 1, 1, 1, time).sample(f, b, source, out, bits, save_a);
        TestPlan(g, 1, 1, 1, time).sample(f, b, source, out, bits, save_b);
        CHECK(a.size() == b_samples.size());
        CHECK(std::memcmp(a.data(), b_samples.data(), a.size() * sizeof(T)) == 0);
      }
    }
}
template <class T>
void sampled_sequences(int bits) {
  for (int pel : {1, 2, 4})
    for (int component : {0, 1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 65, 129, 32767}) {
      const int pad = (component + pel - 1) / pel + 2;
      RenderPhaseGeometry g{pel, 1, 1, pad, 0, {}};
      SubpixelPhases<T> source{pel, {}};
      std::vector<super_detail::PlaneBuffer<T>> planes;
      for (int a = 0; a < pel * pel; ++a) {
        const int width = 2 * pad + 1 - (pel == 4 && a % pel == 3);
        g.phases[a] = {width, 1};
        planes.emplace_back(width, 1);
        for (int x = 0; x < width; ++x)
          planes.back().view().row(0)[x] = T((x * 17 + a * 29) % 251);
        source.planes[a] = planes.back().view();
      }
      const auto f = field(1, 1, static_cast<std::int16_t>(component), 0);
      const auto b = field(1, 1, component == 32767 ? INT16_MIN : static_cast<std::int16_t>(-component), 0);
      for (int time : {0, 1, 127, 128, 255, 256})
        for (int precision : {1, 2, 7, INT32_MAX}) {
          std::vector<T> a, b_samples;
          T output[3] = {T(19), T(19), T(19)};
          const auto out = checked_plane(output + 1, 1, 1, sizeof(T), sizeof(T));
          auto save_a = [&](const T* values, std::size_t count, int, T*) { a.assign(values, values + count); };
          auto save_b = [&](const T* values, std::size_t count, int, T*) { b_samples.assign(values, values + count); };
          neo_mv::BlurSamplingPlan(g, 1, 1, precision, time).sample(f, b, source, out, bits, save_a);
          TestPlan(g, 1, 1, precision, time).sample(f, b, source, out, bits, save_b);
          CHECK(a.size() == b_samples.size());
          CHECK(std::memcmp(a.data(), b_samples.data(), a.size() * sizeof(T)) == 0);
          CHECK(output[0] == T(19) && output[2] == T(19));
        }
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
      vertical_sequences<std::uint8_t>(8);
      vertical_sequences<std::uint16_t>(16);
      vertical_sequences<float>(32);
      sampled_sequences<std::uint8_t>(8);
      sampled_sequences<std::uint16_t>(16);
      sampled_sequences<float>(32);
#endif
    trajectories();
    averages();
    sampling<std::uint8_t>(8);
    sampling<std::uint16_t>(10);
    sampling<std::uint16_t>(16);
    sampling<float>(32);
    domains_and_copy();
#if NEO_MV_TEST_HIGHWAY
    }
    hwy::SetSupportedTargetsForTest(0);
#endif
    std::cout << "Blur trajectory and average specifications passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
