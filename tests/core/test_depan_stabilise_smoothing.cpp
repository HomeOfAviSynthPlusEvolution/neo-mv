#include "core/depan/stabilise_smoothing.hpp"
#include <iostream>
#include <string>
namespace {
using namespace neo_mv::depan;
using namespace neo_mv::depan::stabilise;
void check(bool v, int line) {
  if (!v)
    throw std::runtime_error("stabilise smoothing assertion " + std::to_string(line));
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
void recurrences() {
  CHECK(smooth_component(0, 0, 2, 1, 0, 0, .25f, 0, 1) == .25f);
  CHECK(smooth_component(0, 0, 2, 1, 0, 0, .25f, 2, 1) == .75f);
  auto c = normalize({}, 48, 48, 24, 1);
  c.f = 1;
  c.cd = 0;
  c.cq = .25f;
  c.kx = c.ky = c.kr = 0;
  std::vector<Transform> u(3);
  u[1].tx = 1;
  u[2].tx = 2;
  u[2].u = 1.5f;
  auto r = inertial(u, c);
  CHECK(r[1].tx == 0 && r[1].u == 1 && r[2].tx == .25f && r[2].u == 1.25f && r[2].h == 1.25f);
  c.cd = .5f;
  CHECK(smooth_zoom(1, 1, .8f, 1, 1, 1, c) == .95f);
  CHECK(smooth_zoom(1, 1, 1.2f, 1, 1, 1, c) == 1);
  // The first trial is 0.6; exactly one reduced trial is 0.57, below
  // the clamp. Omitting or repeating the reduced trial changes the result.
  CHECK(smooth_zoom(.5f, .5f, .9f, .5f, .5f, 1, c) == sub(.5f, mul(.175f, sub(.5f, .9f))));
  Parameters p;
  p.addzoom = true;
  p.initzoom = 2;
  p.tzoom = 0;
  c = normalize(p, 48, 48, 24, 1);
  std::vector<Transform> two(2);
  two[1].tx = 2;
  auto out = correction(two, {0, 1}, 1, 8, 48, 48, p, c);
  CHECK(out.map.u == 1 && out.map.tx == -2);
  auto start = correction({Transform{}}, {0, 0}, 0, 8, 48, 48, p, c);
  CHECK(start.map.u == .5f);
  rejects([&] { correction(u, {0, 2}, 2, 8, 48, 48, p, c); });
  p.addzoom = false;
  CHECK(correction(two, {0, 1}, 1, 8, 48, 48, p, c).map.u == .5f);
  rejects([] { smooth_component(0, 0, 0, 0, 0, 1, 1, 1, 0); });
  rejects([] { smooth_component(std::numeric_limits<float>::max(), 0, 0, 0, 0, 1, 1, 1, 1); });
  // A finite large motion sequence can diverge before correction limiting.
  // A negative limit must not convert that arithmetic error into a reset.
  for (float limit : {10.0f, -10.0f}) {
    p = {};
    p.dxmax = limit;
    c = normalize(p, 48, 48, 24000, 1001);
    auto maps = cumulative({0, 7}, c, [](int) { return Motion{20, 0, 0, 1, true}; });
    rejects([&] { inertial(maps, c); });
    rejects([&] { correction(maps, {0, 7}, 7, 8, 48, 48, p, c); });
    maps.pop_back();
    CHECK(std::isfinite(inertial(maps, c).back().tx));
  }
}
void bounds_and_window() {
  Parameters p;
  auto c = normalize(p, 100, 80, 24, 1);
  CHECK(zoom_bound({10, -4, 1, 0, 0, 1}, 100, 80, c) == .8f);
  CHECK(zoom_bound({100, 0, 1, 0, 0, 1}, 100, 80, c) == -1);
  p.method = 1;
  p.cutoff = 6;
  c = normalize(p, 48, 48, 24, 1);
  std::vector<Transform> u(3);
  u[1].tx = 1;
  u[2].tx = 2;
  CHECK(correction(u, {0, 2}, 1, 8, 48, 48, p, c).map.tx == 0);
  p.cutoff = 3;
  c = normalize(p, 48, 48, 24, 1);
  u[2].tx = 1;
  const auto s = window(u, 48, 48, p, c);
  CHECK(s.tx == div(add(1, c.weights[1]), add(add(c.weights[1], 1), c.weights[1])));
  p.cutoff = 2;
  c = normalize(p, 48, 48, 24, 1);
  std::vector<Transform> five(5);
  five.front().u = five.back().u = 100;
  CHECK(window(five, 48, 48, p, c).u == 1);
  // A six-coefficient map with unequal diagonals does not self-cancel
  // under the stipulated similarity inverse. Its geometric bound is .75.
  p.addzoom = true;
  p.cutoff = 3;
  p.tzoom = .25f; // zoom radius 1: only the central adaptive sample has weight 1.
  c = normalize(p, 48, 48, 24, 1);
  for (auto& map : five) {
    map = {};
    map.h = 1.125f;
  }
  CHECK(window(five, 48, 48, p, c).u == .75f);
  p.addzoom = true;
  p.initzoom = 4.0f / 3.0f;
  c = normalize(p, 48, 48, 24, 1);
  CHECK(window({Transform{}}, 48, 48, p, c).u == c.z0);
  p.tzoom = 0;
  c = normalize(p, 48, 48, 24, 1);
  rejects([&] { window({Transform{}}, 48, 48, p, c); });
  p.addzoom = false;
  CHECK(window({Transform{}}, 48, 48, p, c).u == c.z0);
  // Even an endpoint with zero weight is evaluated; infinity is not ignored.
  p.cutoff = 6;
  c = normalize(p, 48, 48, 24, 1);
  u[0].tx = INFINITY;
  rejects([&] { window(u, 48, 48, p, c); });
}
void limits() {
  Parameters p;
  p.dxmax = 10;
  auto c = normalize(p, 48, 48, 24, 1);
  auto limited = [&](float dx) {
    return limit_correction({dx, 0, 1, 0, 0, 1}, 4, 10, 0, p, c);
  };
  CHECK(limited(20).map.tx == sqrt32(mul(sqrt32(200), 10)));
  CHECK(limited(-160).map.tx == -10);
  CHECK(limited(10).map.tx == 10);
  p.dxmax = -10;
  CHECK(limited(20).begin == 4 && limited(20).map.tx == 0);
  p.dxmax = 0;
  CHECK(limited(-1).map.tx == 0);
  p = {};
  p.zoommax = 1.0625f;
  c = normalize(p, 48, 48, 24, 1);
  const auto q = coordinates({0, 0, 0, 1.25f, true}, 1, 24, 24, 1, true);
  CHECK(limit_correction(q, 4, 10, 0, p, c).map.u == 1.125f);
  p = {};
  p.fitlast = 4;
  p.initzoom = 1.25f;
  c = normalize(p, 48, 48, 24, 1);
  auto t = limit_correction({8, -4, 1, 0, 0, 1}, 8, 10, 0, p, c);
  auto m = motion(t.map, 1, 24, 24, true);
  CHECK(m.dx == 2 && m.dy == -1);
  t = limit_correction({8, -4, 1, 0, 0, 1}, 9, 10, 0, p, c);
  CHECK(t.map.u == c.z0);
  p.fitlast = std::numeric_limits<int>::min();
  CHECK(limit_correction({8, -4, 1, 0, 0, 1}, 9, 10, 0, p, c).map.tx == 8);
}
} // namespace
int main() {
  try {
    recurrences();
    bounds_and_window();
    limits();
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
