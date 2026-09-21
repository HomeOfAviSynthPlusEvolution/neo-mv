#include "core/depan/estimate_image.hpp"

#include <climits>
#include <iostream>
#include <limits>

namespace {
using namespace neo_mv;
using namespace neo_mv::depan;
using namespace neo_mv::depan::estimate;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("DepanEstimate image assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  } catch (const std::overflow_error&) {
    caught = true;
  }
  CHECK(caught);
}
template <class T>
struct Buffer {
  std::vector<T> data = std::vector<T>(1 + 9 * 5, T(77));
  span2d::Plane<T> view() {
    return checked_plane(data.data() + 1, 7, 5, 9 * sizeof(T), (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
  void untouched(const std::vector<T>& before, int left, int top, int width, int height) {
    CHECK(data[0] == before[0]);
    for (int y = 0; y < 5; ++y)
      for (int x = 0; x < 9; ++x)
        if (x < left || x >= left + width || y < top || y >= top + height)
          CHECK(data[1 + y * 9 + x] == before[1 + y * 9 + x]);
  }
};
void extraction() {
  Buffer<std::uint16_t> source;
  source.data.assign(source.data.size(), 65535); // Invalid unused high bits, including row gaps.
  for (int y = 0; y < 3; ++y)
    for (int x = 0; x < 3; ++x)
      source.view().row(y + 1)[x + 2] = std::uint16_t(100 * y + x);
  CHECK(extract_window(source.read(), 2, 1, 3, 3, 10) ==
        std::vector<float>({0, 1, 2, 100, 101, 102, 200, 201, 202}));
  source.view().row(3)[4] = 1024;
  rejects([&] { extract_window(source.read(), 2, 1, 3, 3, 10); });
  CHECK(extract_window(source.read(), 2, 1, 3, 3, 16).back() == 1024);
  rejects([&] { extract_window(source.read(), 0, 0, 1, 1, 8); });

  Buffer<float> floating;
  floating.data.assign(floating.data.size(), std::numeric_limits<float>::quiet_NaN());
  floating.view().row(2)[3] = -0.0f;
  CHECK(std::signbit(extract_window(floating.read(), 3, 2, 1, 1, 32)[0]));
  floating.view().row(2)[3] = -123.5f;
  CHECK(extract_window(floating.read(), 3, 2, 1, 1, 32)[0] == -123.5f);
  rejects([&] { extract_window(floating.read(), 2, 2, 2, 1, 32); });
  floating.view().row(2)[3] = std::numeric_limits<float>::infinity();
  rejects([&] { extract_window(floating.read(), 3, 2, 1, 1, 32); });
  rejects([&] { extract_window(floating.read(), 3, 2, 1, 1, 8); });
  Buffer<std::uint8_t> byte;
  CHECK(extract_window(byte.read(), 0, 0, 1, 1, 8)[0] == 77);
  rejects([&] { extract_window(byte.read(), 0, 0, 1, 1, 9); });
  rejects([&] { extract_window(byte.read(), INT_MAX, 0, INT_MAX, 1, 8); });
  rejects([&] { extract_window(byte.read(), 0, INT_MAX, 1, INT_MAX, 8); });
  rejects([&] { extract_window(byte.read(), 0, 0, 0, 1, 8); });
  rejects([&] { extract_window(byte.read(), -1, 0, 1, 1, 8); });
  rejects([&] { extract_window(span2d::Plane<const std::uint8_t>{}, 0, 0, 1, 1, 8); });
}
void display() {
  Buffer<std::uint8_t> byte;
  auto before = byte.data;
  display_surface(byte.view(), {0, 10, 1, 2, 3, 4, 5, 6, 7}, 2, 1, 3, 3, 8);
  CHECK(byte.view().row(1)[2] == 0 && byte.view().row(1)[3] == 255 && byte.view().row(1)[4] == 25);
  byte.untouched(before, 2, 1, 3, 3);
  display_surface(byte.view(), {0, 336}, 0, 0, 2, 1, 8);
  CHECK(byte.view().row(0)[1] == 255); // q exceeds M, but trunc(q) is representable.
  display_surface(byte.view(), {100, 200}, 4, 0, 2, 1, 8);
  CHECK(byte.view().row(0)[4] == 0 && byte.view().row(0)[5] == 255);
  Buffer<std::uint16_t> word;
  display_surface(word.view(), {0, 10, 1}, 2, 2, 3, 1, 10);
  CHECK(word.view().row(2)[2] == 0 && word.view().row(2)[3] == 1023 && word.view().row(2)[4] == 102);
  display_surface(word.view(), {0, 1}, 0, 0, 2, 1, 16);
  CHECK(word.view().row(0)[1] == 65535);
  Buffer<float> floating;
  display_surface(floating.view(), {0, 10, 1}, 1, 1, 3, 1, 32);
  CHECK(floating.view().row(1)[3] == 0.1f);
  display_surface(floating.view(), {0, 0x1.fffff4p127f}, 1, 1, 2, 1, 32);
  CHECK(floating.view().row(1)[2] == 0x1.000002p0f); // Retain subnormal-normalization excursion.

  before = byte.data;
  for (const auto& surface : std::vector<std::vector<float>>{
           {1, 1}, {0, 0}, {0, std::numeric_limits<float>::quiet_NaN()},
           {0, std::numeric_limits<float>::infinity()}, {-std::numeric_limits<float>::max(),
                                                       std::numeric_limits<float>::max()},
           {0, std::numeric_limits<float>::denorm_min()}, {0}}) {
    rejects([&] { display_surface(byte.view(), surface, 0, 0, 2, 1, 8); });
    CHECK(byte.data == before);
  }
  rejects([&] { display_surface(byte.view(), {0, 1}, INT_MAX, 0, 2, 1, 8); });
  CHECK(byte.data == before);
}
void diagnostics() {
  CHECK(diagnostic(0, {}, 0) == "fn=0 dx=0.00 dy=0.00 zoom=1.00000 trust=0.00 bad=1");
  CHECK(diagnostic(2, {1, 0, 0, 1, true}, 88.00879669189453f) ==
        "fn=2 dx=1.00 dy=0.00 zoom=1.00000 trust=88.01 bad=0");
  CHECK(diagnostic(3, {}, 5) == "fn=3 dx=0.00 dy=0.00 zoom=1.00000 trust=5.00 bad=1");
  CHECK(diagnostic(1, {-0.0f, 0.125f, 0, 1, true}, 0.375f) ==
        "fn=1 dx=-0.00 dy=0.12 zoom=1.00000 trust=0.38 bad=0");
  const float large = std::numeric_limits<float>::max();
  const auto text = diagnostic(INT_MAX, {large, large, 0, large, true}, large);
  CHECK(text.size() == 127 && text.find('\n') == std::string::npos && text.find('\0') == std::string::npos);
  rejects([&] { diagnostic(1, {}, std::numeric_limits<float>::quiet_NaN()); });
}
} // namespace
int main() {
  try {
    extraction();
    display();
    diagnostics();
    std::cout << "DepanEstimate image tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
