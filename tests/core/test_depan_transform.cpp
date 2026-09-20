#include "core/depan/temporal.hpp"
#include "core/depan/diagnostics.hpp"
#include <iostream>
#include <vector>

namespace {
using namespace neo_mv::depan;
void check(bool ok, int line) {
  if (!ok)
    throw std::runtime_error("Depan transform assertion at " + std::to_string(line));
}
#define CHECK(x) check((x), __LINE__)
template <class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
void properties() {
  const auto m = decode_motion(2000000, -2000000, 1e300, -2, -3);
  CHECK(m.dx == 1000000 && m.dy == -1000000 && m.rotation == 360000 && m.zoom == 0.01f && m.good);
  CHECK(decode_motion(0, 0, 0, 1e300, 0).zoom == 100);
  CHECK(std::signbit(decode_motion(-0.0, 0, 0, 1, 1).dx));
  rejects([] { decode_motion(0, 0, std::numeric_limits<double>::infinity(), 1, 0); });
  CHECK(field_parity(0, true, 0) && !field_parity(1, true, 1));
  CHECK(field_parity(3, std::nullopt, -2));
  rejects([] { field_parity(0, std::nullopt, std::nullopt); });
  CHECK(fixed(1.125f, 2) == "1.12" && fixed(1.375f, 2) == "1.38");
  CHECK(fixed(-0.0f, 3) == "-0.000" && fixed(-0.0001f, 2) == "-0.00");
  CHECK(analysis_info(0, 0, 30, {}) == "fn=0 iter=0 error=30.000 dx=0.00 dy=0.00 rot=0.000 zoom=1.00000 bad=1");
}
void transforms() {
  for (bool forward : {false, true}) {
    const auto t = coordinates({2, -1, 0, 1, true}, 1, 10, 8, 1, forward);
    CHECK(t.tx == 2 && t.ty == -1 && t.u == 1 && t.h == 1 && t.v == 0 && t.w == 0);
    const auto m = motion(t, 1, 10, 8, forward);
    CHECK(m.dx == 2 && m.dy == -1 && m.rotation == 0 && m.zoom == 1 && m.good);
    const auto inv = analysis_inverse(t);
    CHECK(inv.tx == -2 && inv.ty == 1);
  }
  const Transform scale{0, 0, 2, 0, 0, 2}, shift{3, 0, 1, 0, 0, 1};
  CHECK(compose(scale, shift).tx == 3 && compose(shift, scale).tx == 6);
  const auto aspect = coordinates({2, -1, 0, 1, true}, 2, 10, 8, 1, true);
  CHECK(aspect.ty == -2);
  const auto z = coordinates({0, 0, 0, 4, true}, 1, 10, 8, 0.5f, true);
  CHECK(std::abs(z.u - 2) < 0.000001f && std::abs(z.tx + 10) < 0.00001f);
  rejects([] { analysis_inverse({0, 0, 0, -1, 1, 0}); });
  rejects([] { motion({0, 0, 0, 0, 0, 1}, 1, 0, 0, true); });
  const auto map422 = plane_transform({4, 2, 1, .25f, -.25f, 1}, 2, 1);
  CHECK(map422.tx == 2 && map422.ty == 2 && map422.v == .125f && map422.w == -.5f);
  const auto map420 = plane_transform({4, 2, 1, .25f, -.25f, 1}, 2, 2);
  CHECK(map420.tx == 2 && map420.ty == 1 && map420.v == .25f && map420.w == -.25f);
  // Preserve center arithmetic: the rounded output differs from the raw tx.
  const auto m = motion({0.49161040782928467f, 0, 1, 0, 0, 1}, 1, 24, 24, true);
  CHECK(m.dx == 0.4916095733642578f && std::signbit(m.rotation));
  CHECK(compensation_info(1, 4, 5, motion({2, 0, 1, 0, 0, 1}, 1, 8, 8, true)) ==
        "offset=1.00, 4 to 5, dx=2.00, dy=0.00, rot=-0.000 zoom=1.00000");
  const float tiny = std::numeric_limits<float>::denorm_min();
  const auto negative_u = motion({0, 0, -1, .25f, 0, -1}, 1, 0, 0, true);
  CHECK(negative_u.rotation > 0 && negative_u.rotation < 20 && negative_u.zoom < 0);
  const auto independent = compose({0, 0, 1, .25f, -.5f, 1}, {0, 0, 1, .5f, -.25f, 1});
  CHECK(independent.u == .75f && independent.h == .9375f);
  CHECK(mul(tiny, 1) == tiny && neo_mv::depan::div(tiny, 2) == 0 && !std::signbit(neo_mv::depan::div(tiny, 2)));
  CHECK(std::signbit(sin32(-0.0f)) && cos32(0) == 1 && log32(1) == 0 && exp32(0) == 1);
  rejects([] { mul(std::numeric_limits<float>::max(), 2); });
  rejects([] { neo_mv::depan::div(1, 0); });
  rejects([] { sqrt32(-1); });
}
void temporal() {
  for (float offset : {1.5f, -1.5f}) {
    TemporalPlan p(10, 16, 16, offset);
    const auto pos = p.position(4);
    CHECK(pos.source == (offset > 0 ? 2 : 6) && pos.fraction == (offset > 0 ? .5f : -.5f));
    std::vector<int> seen;
    const auto t = p.accumulate(pos, [&](int k) {
      seen.push_back(k);
      return Motion{2, 0, 0, 1, true};
    });
    CHECK(t.tx == (offset > 0 ? 2 : -2));
    CHECK(seen == std::vector<int>({offset > 0 ? 3 : 5, offset > 0 ? 4 : 6}));
    int calls = 0;
    const auto stopped = p.accumulate(pos, [&](int) {
      ++calls;
      return Motion{};
    });
    CHECK(calls == 1 && stopped.tx == 0 && stopped.u == 1);
  }
  TemporalPlan p(10, 16, 16, 1, 1, true, true);
  TemporalPlan ordered(10, 16, 16, 2);
  const auto noncommuting = ordered.accumulate(
      ordered.position(4), [](int k) { return k == 3 ? Motion{2, 0, 0, 1, true} : Motion{0, 0, 0, 2, true}; });
  CHECK(noncommuting.tx == -4 && noncommuting.u == 2);
  CHECK(p.position(0).bypass && p.needs_parity() && p.aspect() == .5f);
  CHECK(p.align({}, false).ty == .5f && p.align({}, true).ty == -.5f);
  CHECK(TemporalPlan(2, 1, 1, 0).position(1).bypass);
  CHECK(!TemporalPlan(2, 1, 1, 1, 1, true, false).needs_parity());
  rejects([] { TemporalPlan(1, 1, 1, 11); });
}
} // namespace
int main() {
  try {
    properties();
    transforms();
    temporal();
    std::cout << "Depan transform specifications passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
