#include "core/depan/stabilise_motion.hpp"
#include <iostream>
#include <string>

namespace {
using namespace neo_mv::depan;
using namespace neo_mv::depan::stabilise;
void check(bool value, int line) {
  if (!value)
    throw std::runtime_error("stabilise assertion at " + std::to_string(line));
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
void parameters() {
  Parameters p;
  p.cutoff = 3;
  p.tzoom = .25f;
  auto c = normalize(p, 48, 32, 24, 1);
  CHECK(c.radius == 2 && c.zoom_radius == 1);
  CHECK(c.weights.size() == 3 && c.weights[0] == 1 && c.weights[2] == 0 && !std::signbit(c.weights[2]));
  CHECK(std::abs(c.weights[1] - .70710677f) <= std::numeric_limits<float>::epsilon());
  CHECK(c.zoom_weights == std::vector<float>({1, 0, 0}));
  p.initzoom = 2;
  p.pixaspect = 1.5f;
  p.fields = true;
  c = normalize(p, 48, 32, 24, 1);
  CHECK(c.z0 == .5f && c.zoom_limit == 2 && c.aspect == .75f && c.cx == 24 && c.cy == 16);
  p.zoommax = 0;
  p.tzoom = 0;
  c = normalize(p, 48, 32, 24, 1);
  CHECK(c.zoom_limit == -2 && c.zoom_weights == std::vector<float>({0, 0, 0}));
  p.damping = -0.9f;
  CHECK(normalize(p, 48, 32, 24, 1).cd < 0);
  p.method = 1;
  p.damping = std::numeric_limits<float>::max();
  CHECK(normalize(p, 48, 32, 24, 1).f == 0);
  p.method = 0;
  rejects([&] { normalize(p, 48, 32, 24, 1); });
  p = {};
  p.cutoff = 0;
  rejects([&] { normalize(p, 48, 32, 24, 1); });
  p = {};
  p.tzoom = std::numeric_limits<float>::max();
  rejects([&] { normalize(p, 48, 32, 24, 1); });
  p = {};
  p.pixaspect = std::numeric_limits<float>::denorm_min();
  p.fields = true;
  rejects([&] { normalize(p, 48, 32, 24, 1); });
  p = {};
  p.cutoff = std::numeric_limits<float>::denorm_min();
  rejects([&] { normalize(p, 48, 32, 24, 1); });
  rejects([] { truncate32(2147483648.0f); });
  CHECK(truncate32(-2147483648.0f) == std::numeric_limits<int>::min());
  // Above the binary64 exact-integer range, conversion still rounds directly
  // to binary32 rather than rounding a midpoint through double first.
  const std::int64_t midpoint = (std::int64_t(1) << 60) + (std::int64_t(1) << 36);
  CHECK(positive_integer32(midpoint + 1) == std::nextafter(0x1p60f, INFINITY));
  CHECK(positive_integer32(midpoint) == 0x1p60f);
}
void intervals() {
  Parameters p;
  auto c = normalize(p, 48, 48, 25, 1);
  std::vector<int> visited;
  auto decode = [&](int k) {
    visited.push_back(k);
    CHECK(k >= 120);
    return Motion{0, 0, 0, 1, k != 120};
  };
  auto bounds = select_interval(140, 200, p, c, decode);
  CHECK(bounds.begin == 120 && bounds.end == 140 && visited.front() == 140 && visited.back() == 120);
  p.method = 1;
  c.radius = 3;
  visited.clear();
  bounds = select_interval(5, 12, p, c, [&](int k) {
    visited.push_back(k);
    return Motion{0, 0, 0, 1, k != 3 && k != 8};
  });
  CHECK(bounds.begin == 3 && bounds.end == 7);
  CHECK(visited == std::vector<int>({5, 4, 3, 6, 7, 8}));
  visited.clear();
  bounds = select_interval(0, 5, p, c, [&](int k) {
    visited.push_back(k);
    CHECK(k != 0);
    return Motion{0, 0, 0, 1, true};
  });
  CHECK(bounds.begin == 0 && bounds.end == 0 && visited == std::vector<int>({1, 2, 3}));
  rejects([&] { select_interval(0, 5, p, c, [](int) -> Motion { throw std::invalid_argument("malformed"); }); });
  p.method = 0;
  visited.clear();
  bounds = select_interval(0, 5, p, c, [&](int k) {
    visited.push_back(k);
    return Motion{};
  });
  CHECK(bounds.begin == 0 && visited.empty());
  p.cutoff = 2500;
  c = normalize(p, 48, 48, 25, 1);
  rejects([&] { select_interval(16777219, 20000000, p, c, [](int) { return Motion{}; }); });
  // Exercise the explicitly capped quotient independently of weight allocation.
  p.cutoff = std::numeric_limits<float>::denorm_min();
  c.fps = 25;
  bounds = select_interval(140, 200, p, c, [](int) { return Motion{0, 0, 0, 1, true}; });
  CHECK(bounds.begin == 15 && bounds.end == 140);
}
void transforms() {
  const auto c = normalize({}, 48, 48, 24, 1);
  auto maps = cumulative({0, 2}, c, [](int k) {
    CHECK(k != 0);
    return Motion{float(k), 0, 0, 1, true};
  });
  CHECK(maps.size() == 3 && maps[0].tx == 0 && maps[1].tx == 1 && maps[2].tx == 3);
  CHECK(inverse(maps[2]).tx == -3);
  // A +2 step followed by zoom 2 about (24,24) is noncommutative:
  // (x+2)*2-24 = 2*x-20, whereas reversing composition gives 2*x-22.
  const auto scaled =
      cumulative({0, 2}, c, [](int k) { return k == 1 ? Motion{2, 0, 0, 1, true} : Motion{0, 0, 0, 2, true}; });
  CHECK(scaled[2].tx == -20 && scaled[2].ty == -24 && scaled[2].u == 2);
  Transform t{2, 0, 1, 0, 0, std::nextafter(1.0f, 2.0f)};
  CHECK(inverse(t).tx == -2 && inverse(t).h == 1);
  rejects([&] { analysis_inverse(t); });
  auto q = compose(inverse({0, 0, 2, 0, 0, 2}), Transform{3, 0, 1, 0, 0, 1});
  CHECK(q.u == .5f && q.tx == 3);
  rejects([] { inverse({0, 0, 0, 1, -1, 0}); });
  rejects([&] { cumulative({0, 1}, c, [](int) { return Motion{}; }); });
  CHECK(zoom(-1, c).u == 1 && zoom(.5f, c).u == .5f);
}
} // namespace
int main() {
  try {
    parameters();
    intervals();
    transforms();
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
