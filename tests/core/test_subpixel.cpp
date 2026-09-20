#include "core/super/subpixel.hpp"

#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("subpixel assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class F>
void rejects(F call) {
  bool rejected = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  CHECK(rejected);
}

template <class T>
struct Storage {
  int width, height, stride;
  std::vector<T> base;
  std::array<std::vector<T>, 16> output;
  std::array<span2d::Plane<T>, 16> views{};
  explicit Storage(int w, int h) : width(w), height(h), stride(w + 3), base(stride * h, T(77)) {
    for (int i = 1; i < 16; ++i) {
      output[i].assign(stride * h, T(99));
      views[i] =
          neo_mv::checked_plane(output[i].data() + 1, w, h, stride * sizeof(T), (output[i].size() - 1) * sizeof(T));
    }
  }
  span2d::Plane<const T> input() const {
    return neo_mv::checked_plane(base.data() + 1, width, height, stride * sizeof(T), (base.size() - 1) * sizeof(T));
  }
  void set(int x, int y, T value) { base[y * stride + x + 1] = value; }
  void fill(T value) {
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        set(x, y, value);
  }
  void guards(int pel, bool restricted) const {
    for (int i = 1; i < pel * pel; ++i)
      for (int y = 0; y < height; ++y) {
        CHECK(output[i][y * stride] == T(99));
        CHECK(output[i][y * stride + width + 1] == T(99) && output[i][y * stride + width + 2] == T(99));
        if (restricted && pel == 4 && i % pel == 3)
          CHECK(views[i].row(y)[width - 1] == T(99));
        if (restricted && pel == 4 && i / pel == 3 && y == height - 1)
          for (int x = 0; x < width; ++x)
            CHECK(views[i].row(y)[x] == T(99));
      }
  }
};

template <class T>
constexpr int bits() {
  return std::is_same_v<T, float> ? 32 : sizeof(T) == 1 ? 8 : 16;
}

template <class T>
void phase_examples() {
  Storage<T> s(2, 2);
  s.set(0, 0, T(10));
  s.set(1, 0, T(14));
  s.set(0, 1, T(18));
  s.set(1, 1, T(22));
  const auto original = s.base;
  auto two = neo_mv::interpolate_subpixels(s.input(), 2, 0, bits<T>(), s.views);
  const int expected[4][4] = {{10, 14, 18, 22}, {12, 14, 20, 22}, {14, 18, 18, 22}, {16, 18, 20, 22}};
  for (int i = 0; i < 4; ++i)
    for (int y = 0; y < 2; ++y)
      for (int x = 0; x < 2; ++x)
        CHECK(two.planes[i].row(y)[x] == T(expected[i][y * 2 + x]));
  s.guards(2, false);
  Storage<T> q(2, 2);
  q.base = s.base;
  auto four = neo_mv::interpolate_subpixels(q.input(), 4, 0, bits<T>(), q.views);
  // For this linear 2x2 patch all sixteen phase values at (0,0) are exact.
  for (int ay = 0; ay < 4; ++ay)
    for (int ax = 0; ax < 4; ++ax) {
      const auto phase = four.planes[ay * 4 + ax];
      CHECK(phase.row(0)[0] == T(10 + ax + 2 * ay));
      CHECK(phase.width() == 2 - (ax == 3) && phase.height() == 2 - (ay == 3));
    }
  q.guards(4, true);
  CHECK(s.base == original && q.base == original);
  std::array<span2d::Plane<T>, 16> unused{};
  auto one = neo_mv::interpolate_subpixels(s.input(), 1, 2, bits<T>(), unused);
  CHECK(one.planes[0].data() == s.input().data());
}

template <class T>
void half_examples() {
  const int values[8] = {0, 10, 20, 40, 80, 100, 100, 100};
  Storage<T> horizontal(8, 8), vertical(8, 8);
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 8; ++x) {
      horizontal.set(x, y, T(values[x]));
      vertical.set(x, y, T(values[y]));
    }
  for (int sharp = 0; sharp < 3; ++sharp) {
    const T expected = std::is_same_v<T, float> ? T(sharp == 0   ? 30.0
                                                    : sharp == 1 ? 28.125
                                                                 : 26.5625)
                                                : T(sharp == 0   ? 30
                                                    : sharp == 1 ? 28
                                                                 : 27);
    auto h = neo_mv::interpolate_subpixels(horizontal.input(), 2, sharp, bits<T>(), horizontal.views);
    auto v = neo_mv::interpolate_subpixels(vertical.input(), 2, sharp, bits<T>(), vertical.views);
    CHECK(h.planes[1].row(2)[2] == expected && v.planes[2].row(2)[2] == expected);
    CHECK(h.planes[3].row(2)[2] == expected && v.planes[3].row(2)[2] == expected);
    CHECK(h.planes[1].row(0)[0] == T(5));   // fallback, even at sharp=2
    CHECK(h.planes[1].row(0)[7] == T(100)); // exact last sample
  }
}

void clipping_and_rounding() {
  Storage<std::uint16_t> s(8, 8);
  s.fill(0);
  for (int y = 0; y < 8; ++y) {
    s.set(2, y, 1023);
    s.set(3, y, 1023);
  }
  auto r = neo_mv::interpolate_subpixels(s.input(), 2, 2, 10, s.views);
  CHECK(r.planes[1].row(2)[2] == 1023); // positive overshoot clipped to bit depth
  s.fill(0);
  for (int y = 0; y < 8; ++y) {
    s.set(1, y, 1023);
    s.set(4, y, 1023);
  }
  r = neo_mv::interpolate_subpixels(s.input(), 2, 2, 10, s.views);
  CHECK(r.planes[1].row(2)[2] == 0);
  s.set(0, 0, 1024);
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 0, 10, s.views); });
  Storage<std::uint8_t> small(2, 2);
  small.fill(0);
  small.set(1, 1, 1);
  auto direct = neo_mv::interpolate_subpixels(small.input(), 2, 0, 8, small.views);
  CHECK(direct.planes[3].row(0)[0] == 0); // two rounded averages would give 1
}

template <class T>
void external_phases() {
  Storage<T> s(4, 4);
  s.fill(T(10));
  const std::array<T, 4> ext{T(99), T(14), T(18), T(22)};
  const auto e = neo_mv::checked_plane(ext.data(), 2, 2, 2 * sizeof(T), sizeof(ext));
  auto phases = neo_mv::extract_external_subpixels(s.input(), e, 1, 1, 1, 1, 2, bits<T>(), s.views);
  for (int i = 0; i < 4; ++i)
    for (int y = 0; y < 4; ++y)
      for (int x = 0; x < 4; ++x)
        CHECK(phases.planes[i].row(y)[x] == (i == 0 ? T(10) : ext[i]));
  s.guards(2, false);
  // pel=4 external phases fill even the built-in undefined right/bottom edges.
  std::array<T, 16> e4{};
  for (int i = 0; i < 16; ++i)
    e4[i] = T(i);
  auto p4 = neo_mv::extract_external_subpixels(
      s.input(), neo_mv::checked_plane<const T>(e4.data(), 4, 4, 4 * sizeof(T), sizeof(e4)), 1, 1, 1, 1, 4, bits<T>(),
      s.views);
  CHECK(p4.planes[15].width() == 4 && p4.planes[15].height() == 4 && p4.planes[15].row(3)[3] == T(15));
  s.guards(4, false);
  std::array<span2d::Plane<T>, 16> empty{};
  auto one = neo_mv::extract_external_subpixels(s.input(), span2d::Plane<const T>{}, 1, 1, 1, 1, 1, bits<T>(), empty);
  CHECK(one.planes[0].data() == s.input().data());
  rejects([&] { neo_mv::extract_external_subpixels(s.input(), e, 2, 1, 0, 0, 2, bits<T>(), s.views); });
}

void rejection_and_floats() {
  Storage<float> s(8, 8);
  s.fill(0.25f);
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 3, 0, 32, s.views); });
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 3, 32, s.views); });
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 0, 16, s.views); });
  auto alias = s.views;
  alias[1] =
      neo_mv::checked_plane(s.base.data() + 1, 8, 8, s.stride * sizeof(float), (s.base.size() - 1) * sizeof(float));
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 0, 32, alias); });
  alias = s.views;
  alias[2] = alias[1];
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 0, 32, alias); });
  s.set(0, 0, std::numeric_limits<float>::quiet_NaN());
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 0, 32, s.views); });
  CHECK(s.views[1].row(0)[0] == 99.0f);
  s.fill(std::numeric_limits<float>::max());
  rejects([&] { neo_mv::interpolate_subpixels(s.input(), 2, 0, 32, s.views); });
  Storage<float> small(2, 2);
  small.fill(-0.0f);
  rejects([&] { neo_mv::interpolate_subpixels(small.input(), 2, 1, 32, small.views); });
  auto zeros = neo_mv::interpolate_subpixels(small.input(), 2, 0, 32, small.views);
  CHECK(std::signbit(zeros.planes[1].row(1)[1]));
  small.set(0, 0, 16777216.0f);
  small.set(1, 0, 1.0f);
  small.set(0, 1, -16777216.0f);
  small.set(1, 1, 1.0f);
  auto grouped = neo_mv::interpolate_subpixels(small.input(), 2, 0, 32, small.views);
  CHECK(grouped.planes[3].row(0)[0] == 0.25f); // fixed D argument order
  // Unused integer-phase samples from the external clip need not be finite.
  const float ext[] = {std::numeric_limits<float>::quiet_NaN(), 14, 18, 22};
  auto e = neo_mv::checked_plane<const float>(ext, 2, 2, 2 * sizeof(float), sizeof(ext));
  auto external = neo_mv::extract_external_subpixels(small.input(), e, 1, 1, 0, 0, 2, 32, small.views);
  CHECK(external.planes[3].row(1)[1] == 22);
}
} // namespace

int main() {
  try {
    phase_examples<std::uint8_t>();
    phase_examples<std::uint16_t>();
    phase_examples<float>();
    half_examples<std::uint8_t>();
    half_examples<std::uint16_t>();
    half_examples<float>();
    external_phases<std::uint8_t>();
    external_phases<std::uint16_t>();
    external_phases<float>();
    clipping_and_rounding();
    rejection_and_floats();
    std::cout << "Scalar subpixel interpolation checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
