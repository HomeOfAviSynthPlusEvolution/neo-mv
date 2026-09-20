#include "core/render/weighted_samples.hpp"
#include "core/render/change_limit.hpp"

#include <iostream>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Degrain assertion failed at line " + std::to_string(line));
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
AnalysisMetadata metadata() {
  return super_analysis_metadata(SuperPlan<std::uint8_t>({8, 8, 8, 8, 0, 0, 4, 4}, 8), -1);
}
void weights() {
  const auto m = metadata();
  CHECK(degrain_reliability(0, 100) == 256);
  CHECK(degrain_reliability(50, 100) == 153);
  CHECK(degrain_reliability(100, 100) == 0);
  CHECK(degrain_reliability(0, 0) == 0);
  CHECK(rejects([&] { degrain_reliability(-1, 100); }));
  const DegrainWeightPlan plan(m, 1, {100, 50}, {200, 100}, {1, 1, 1});
  auto w = plan({{true, 0}, {true, 0}}, 0);
  CHECK(w.centre == 86 && w.reference == std::vector<int>({85, 85}));
  w = plan({{true, 0}, {false, 0}}, 0);
  CHECK(w.centre == 129 && w.reference == std::vector<int>({127, 0}));
  w = DegrainWeightPlan(m, 1, {100, 100}, {100, 100}, {1, 0, 1})({{true, 0}, {true, 0}}, 0);
  CHECK(w.centre == 2 && w.reference == std::vector<int>({127, 127}));
  w = DegrainWeightPlan(m, 1, {100, 100}, {100, 100}, {0, 0, 0})({{true, 0}, {true, 0}}, 0);
  CHECK(w.centre == 256 && w.reference == std::vector<int>({0, 0}));
  w = DegrainWeightPlan(m, 2, {100, 100}, {100, 100},
                        {10, 20, 30, 40, 50})({{true, 0}, {true, 0}, {true, 0}, {true, 0}}, 0);
  CHECK(w.centre == 52 && w.reference == std::vector<int>({34, 68, 17, 85}));
  const DegrainWeightPlan slope(m, 3, {100, 300}, {300, 100}, std::vector<std::int64_t>(7, 1));
  CHECK(slope.threshold(0, 0) == 100 && slope.threshold(1, 0) == 200 && slope.threshold(2, 0) == 300);
  CHECK(slope.threshold(0, 2) == 300 && slope.threshold(1, 2) == 200 && slope.threshold(2, 2) == 100);
  for (int pairs = 1; pairs <= 25; ++pairs) {
    const auto maximum = 2147483646LL / (256 * (2 * pairs + 1));
    const DegrainWeightPlan bounded(m, pairs, {1000, 500}, {200, 0}, std::vector<std::int64_t>(2 * pairs + 1, maximum));
    for (int trial = 0; trial < 30; ++trial) {
      std::vector<ReferenceReliability> inputs;
      for (int i = 0; i < 2 * pairs; ++i)
        inputs.push_back({(trial + i) % 5 != 0, (trial * 67 + i * 79) % 1100});
      for (int plane = 0; plane < 3; ++plane) {
        const auto normalized = bounded(inputs, plane);
        int sum = normalized.centre;
        CHECK(sum >= 0 && sum <= 256);
        for (int v : normalized.reference) {
          CHECK(v >= 0 && v <= 256);
          sum += v;
        }
        CHECK(sum == 256);
      }
    }
    CHECK(rejects([&] {
      DegrainWeightPlan bad(m, pairs, {100, 100}, {100, 100}, std::vector<std::int64_t>(2 * pairs + 1, maximum + 1));
    }));
  }
  CHECK(rejects([&] { DegrainWeightPlan bad(m, 1, {100, 100}, {INT32_MAX, 100}, {1, 1, 1}); }));
  CHECK(rejects([&] { DegrainWeightPlan bad(m, 1, {100, -1}, {100, 100}, {1, 1, 1}); }));
  CHECK(rejects([&] { DegrainWeightPlan bad(m, 1, {100, 100}, {100, 100}, {}); }));
  CHECK(rejects([&] { DegrainWeightPlan bad(m, 1, {100, 100}, {100, 100}, {INT64_MAX, 1, 1}); }));
  auto ten_bit = m;
  ten_bit.bits = 10;
  CHECK(DegrainWeightPlan(ten_bit, 1, {400, 400}, {400, 400}, {1, 1, 1}).threshold(0, 0) == 1605);
}
void pairing() {
  auto a = metadata(), b = a, c = a, d = a;
  a.delta = -1;
  b.delta = 1;
  c.delta = 4;
  d.delta = -4;
  d.levels = 9;
  validate_degrain_pairs({a, b, c, d});
  CHECK(rejects([&] { validate_degrain_pairs({a}); }));
  CHECK(rejects([&] { validate_degrain_pairs({a, b, a, b}); }));
  b.delta = 2;
  CHECK(rejects([&] { validate_degrain_pairs({a, b}); }));
  a.delta = INT32_MIN;
  b.delta = INT32_MAX;
  CHECK(rejects([&] { validate_degrain_pairs({a, b}); }));
  a.delta = b.delta = 0;
  CHECK(rejects([&] { validate_degrain_pairs({a, b}); }));
}

template <class T>
void samples(int bits) {
  super_detail::PlaneBuffer<T> c(4, 4), a(4, 4), b(4, 4), output(4, 4);
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x) {
      c.view().row(y)[x] = 100;
      a.view().row(y)[x] = 80;
      b.view().row(y)[x] = 140;
    }
  std::vector<WeightedReferenceBlock<T>> refs{{true, a.view()}, {true, b.view()}};
  weighted_render_block<T>(c.view(), refs, {86, {85, 85}}, output.view(), bits);
  CHECK(output.view().row(0)[0] == (std::is_same_v<T, float> ? T(106.640625) : T(107)));
  refs[1] = {false, {}};
  weighted_render_block<T>(c.view(), refs, {129, {127, 0}}, output.view(), bits);
  CHECK(output.view().row(0)[0] == (std::is_same_v<T, float> ? T(90.078125) : T(90)));
  refs[0] = {false, {}};
  weighted_render_block<T>(c.view(), refs, {256, {0, 0}}, output.view(), bits);
  CHECK(output.view().row(0)[0] == 100);
  CHECK(rejects([&] { weighted_render_block<T>(c.view(), refs, {255, {1, 0}}, output.view(), bits); }));
  CHECK(rejects([&] { weighted_render_block<T>(c.view(), refs, {255, {0, 0}}, output.view(), bits); }));
  CHECK(rejects([&] { weighted_render_block<T>(c.view(), refs, {256, {0, 0}}, c.view(), bits); }));
  refs[0] = {true, a.view()};
  if constexpr (std::is_same_v<T, float>) {
    c.view().row(0)[0] = -0.0f;
    weighted_render_block<T>(c.view(), refs, {256, {0, 0}}, output.view(), bits);
    CHECK(!std::signbit(output.view().row(0)[0]));
    a.view().row(3)[3] = NAN;
    CHECK(rejects([&] { weighted_render_block<T>(c.view(), refs, {256, {0, 0}}, output.view(), bits); }));
    a.view().row(3)[3] = 80;
    c.view().row(0)[0] = std::numeric_limits<float>::max();
    CHECK(rejects<std::overflow_error>(
        [&] { weighted_render_block<T>(c.view(), refs, {256, {0, 0}}, output.view(), bits); }));
  }
}
void limits() {
  CHECK(ChangeLimit<std::uint8_t>(2.2, 8)(107, 100) == 102);
  CHECK(ChangeLimit<std::uint8_t>(0.1, 8)(107, 100) == 100);
  CHECK(ChangeLimit<std::uint16_t>(2, 10)(1023, 1020) == 1022);
  CHECK(!ChangeLimit<std::uint8_t>(255, 8).active());
  const ChangeLimit<float> small(0.1, 32), large(2, 32);
  CHECK(small(0.9f, 0.25f) == 0.25f + 0.1f);
  CHECK(large.active() && large(5, 0) == 2);
  CHECK(std::signbit(small(-0.0f, 0)));
  for (double v : {double(INFINITY), double(-INFINITY), double(NAN)}) {
    CHECK(!ChangeLimit<float>(v, 32).active());
    CHECK(ChangeLimit<float>(v, 32)(9, 0) == 9);
  }
  for (double v : {0.0, -0.0, -1.0, std::numeric_limits<double>::denorm_min()})
    CHECK(rejects([&] { ChangeLimit<float> invalid(v, 32); }));
  CHECK(ChangeLimit<float>(std::nextafter(double(std::numeric_limits<float>::max()), INFINITY), 32).active());
  CHECK(!ChangeLimit<float>(0x1.ffffffp+127, 32).active());
  CHECK(ChangeLimit<float>(std::nextafter(0x1.ffffffp+127, 0.0), 32).active());
  CHECK(!ChangeLimit<float>(-0x1.ffffffp+127, 32).active());
  CHECK(rejects([&] { ChangeLimit<float> invalid(std::nextafter(-0x1.ffffffp+127, 0.0), 32); }));
  CHECK(rejects([&] { ChangeLimit<float> invalid(-double(std::numeric_limits<float>::max()), 32); }));
  CHECK(rejects<std::overflow_error>(
      [&] { ChangeLimit<float>(std::numeric_limits<float>::max(), 32)(1, std::numeric_limits<float>::max()); }));
  CHECK(rejects([&] { ChangeLimit<float>(INFINITY, 32)(NAN, 0); }));
  super_detail::PlaneBuffer<std::uint8_t> q(2, 2), c(2, 2), out(2, 2);
  q.view().row(0)[0] = 107;
  c.view().row(0)[0] = 100;
  limit_render_plane<std::uint8_t>(ChangeLimit<std::uint8_t>(2.2, 8), q.view(), c.view(), out.view());
  CHECK(out.view().row(0)[0] == 102);
  CHECK(rejects(
      [&] { limit_render_plane<std::uint8_t>(ChangeLimit<std::uint8_t>(2.2, 8), q.view(), c.view(), q.view()); }));
}
} // namespace
int main() {
  try {
    weights();
    pairing();
    samples<std::uint8_t>(8);
    samples<std::uint16_t>(16);
    samples<float>(32);
    limits();
    std::cout << "Degrain pairing, weights, samples and change limit checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
