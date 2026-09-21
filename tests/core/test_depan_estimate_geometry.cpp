#include "core/depan/estimate_geometry.hpp"
#include <iostream>
#include <string>

namespace {
using namespace neo_mv::depan::estimate;
void check(bool valid, int line) {
  if (!valid)
    throw std::runtime_error("DepanEstimate geometry assertion at " + std::to_string(line));
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
void equal(WindowGeometry actual, WindowGeometry expected) {
  CHECK(actual.width == expected.width && actual.height == expected.height && actual.left == expected.left &&
        actual.left2 == expected.left2 && actual.top == expected.top && actual.mx == expected.mx &&
        actual.my == expected.my && actual.two == expected.two);
}
void examples() {
  equal(make_geometry(1920, 1080), {1024, 1024, 448, 448, 28, 256, 256, false});
  equal(make_geometry(1920, 1080, {0, 0, -1, -1, -1, -1, 1.1f}),
        {512, 1024, 224, 1184, 28, 128, 256, true});
  equal(make_geometry(12, 5, {8, 3}), {8, 3, 2, 2, 1, 2, 0, false});
  rejects([] { make_geometry(100, 20, {12, 4, 45, 0, -1, -1, 1.1f}); });
  rejects([] { make_geometry(100, 20, {6, 4, -1, -1, -1, -1, 1.1f}); });
  rejects([] { make_geometry(2, 1, {2, 1, -1, -1, 0, 0}); });
  equal(make_geometry(2, 2, {2, 2, -1, -1, 0, 0}), {2, 2, 0, 0, 0, 0, 0, false});
  CHECK(make_geometry(20000, 20).width == 8192);
  CHECK(make_geometry(20000, 20, {10000}).width == 10000);
  // The separation uses half the clip width, including for odd clips.
  equal(make_geometry(15, 7, {8, 5, -1, -1, 0, 0, 0}), {4, 5, 1, 8, 1, 0, 0, true});
  equal(make_geometry(100, 20, {12, 4, 44, 16, 2, 1, -3}), {6, 4, 44, 94, 16, 2, 1, true});
}
void automatic_thresholds() {
  for (int power = 2; power <= 8192; power *= 2) {
    for (int delta : {-1, 0, 1}) {
      const int available = power + delta;
      if (available < 2) {
        rejects([&] { make_geometry(available, 8); });
        rejects([&] { make_geometry(8, available); });
        continue;
      }
      const int selected = delta < 0 ? power / 2 : power;
      const auto x = make_geometry(available + 3, 8, {0, 0, 3});
      CHECK(x.width == selected && x.left == 3);
      const auto y = make_geometry(8, available + 3, {0, 0, -1, 3});
      CHECK(y.height == selected && y.top == 3);
    }
  }
  CHECK(make_geometry(16383, 16385).width == 8192);
  CHECK(make_geometry(16383, 16385).height == 8192);
  rejects([] { make_geometry(3, 8, {0, 0, -1, -1, -1, -1, 2}); });
  equal(make_geometry(4, 2, {0, 0, -1, -1, -1, -1, 2}), {2, 2, 0, 2, 0, 0, 0, true});
}
void extremes() {
  constexpr auto lo = std::numeric_limits<std::int64_t>::min();
  constexpr auto hi = std::numeric_limits<std::int64_t>::max();
  constexpr int imax = std::numeric_limits<std::int32_t>::max();
  equal(make_geometry(1920, 1080, {0, 0, lo, lo, lo, lo}), make_geometry(1920, 1080));
  // Saturating height is observable: int64 max is admitted at int32 max.
  equal(make_geometry(imax, imax, {imax - 1, hi, 1, 0, lo, lo}),
        {imax - 1, imax, 1, 1, 0, (imax - 1) / 4, imax / 4, false});
  equal(make_geometry(imax, imax, {imax - 3, hi, -1, -1, 0, 0, 2}),
        {(imax - 3) / 2, imax, 0, imax / 2, 0, 0, 0, true});
  for (int member = 0; member < 6; ++member) {
    GeometryParameters p;
    switch (member) {
    case 0: p.winx = hi; break;
    case 1: p.winy = hi; break;
    case 2: p.wleft = hi; break;
    case 3: p.wtop = hi; break;
    case 4: p.dxmax = hi; break;
    case 5: p.dymax = hi; break;
    }
    rejects([&] { make_geometry(16, 16, p); });
  }
  rejects([&] { make_geometry(16, 16, {lo}); });
  rejects([&] { make_geometry(16, 16, {0, lo}); });
  rejects([&] { make_geometry(imax, imax, {hi}); });
  rejects([&] { make_geometry(imax, imax, {2, 2, imax, imax}); });
  for (int dimension : {0, -1, std::numeric_limits<int>::min()}) {
    rejects([&] { make_geometry(dimension, 8); });
    rejects([&] { make_geometry(8, dimension); });
  }
  for (float zoom : {std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::quiet_NaN()})
    rejects([&] { make_geometry(8, 8, {0, 0, -1, -1, -1, -1, zoom}); });
  for (float zoom : {0.0f, -0.0f, -1.0f, std::numeric_limits<float>::max(),
                     std::nextafter(1.0f, 0.0f), std::nextafter(1.0f, 2.0f)})
    CHECK(make_geometry(8, 8, {0, 0, -1, -1, -1, -1, zoom}).two);
}
void final_domains() {
  // Enumerate every explicit rectangle width/height and nearby search bound.
  // The oracle checks membership by counting legal nonnegative search indices,
  // independent of the implementation's division-based acceptance condition.
  for (bool two : {false, true}) {
    for (int width = 1; width <= 16; ++width) {
      for (int height = 1; height <= 17; ++height) {
        int x_count = 0, y_count = 0;
        for (int x = 0; 2 * x + 2 <= width; ++x)
          ++x_count;
        for (int y = 0; 2 * y + 2 <= height; ++y)
          ++y_count;
        for (int mx = 0; mx <= width; ++mx) {
          for (int my = 0; my <= height; ++my) {
            GeometryParameters p{two ? width * 2 : width, height, 0, 0, mx, my, two ? 2.0f : 1.0f};
            const bool valid = width % 2 == 0 && mx < x_count && my < y_count;
            if (valid)
              equal(make_geometry(64, 32, p), {width, height, 0, two ? 32 : 0, 0, mx, my, two});
            else
              rejects([&] { make_geometry(64, 32, p); });
          }
        }
      }
    }
  }
  // Explicit placements test both the pre-halving fit and final right edge.
  for (int left = 0; left <= 16; ++left) {
    if (left <= 6)
      equal(make_geometry(15, 8, {4, 3, left, 5, 0, 0, 2}), {2, 3, left, left + 7, 5, 0, 0, true});
    else
      rejects([&] { make_geometry(15, 8, {4, 3, left, 5, 0, 0, 2}); });
  }
  rejects([] { make_geometry(8, 8, {8, 2, 1, 0, 0, 0, 2}); });
  rejects([] { make_geometry(8, 8, {2, 3, 0, 6}); });
}
} // namespace

int main() {
  try {
    examples();
    automatic_thresholds();
    extremes();
    final_domains();
    std::cout << "DepanEstimate geometry tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
