#include "core/interpolation/pixel.hpp"

#include <iostream>
#include <string>
#include <limits>

namespace {
using namespace neo_mv;
void check(bool value, int line) {
  if (!value)
    throw std::runtime_error("interpolation pixel assertion at " + std::to_string(line));
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
void integers(int bits) {
  CHECK(interpolation_basic<T>(10, 21, 100, 200, 0, 0, 128, bits) == 15);
  CHECK(interpolation_basic<T>(10, 30, 100, 200, 255, 0, 128, bits) == 29);
  CHECK(interpolation_extra<T>(10, 30, 50, 0, 255, 0, 128, bits) == 20);
  CHECK(interpolation_blend<T>(10, 21, 128, bits) == 15);
  CHECK(interpolation_blend<T>(10, 21, 0, bits) == 10);
  CHECK(interpolation_blend<T>(10, 21, 256, bits) == 21);
  // High-valued nested terms require wider intermediates than render storage.
  const T maximum = static_cast<T>((1u << bits) - 1);
  for (const T constant : {T{0}, T{1}, maximum})
    for (const int forward : {0, 1, 128, 255})
      for (const int backward : {0, 1, 128, 255})
        for (const int time : {0, 1, 85, 128, 255, 256}) {
          CHECK(interpolation_basic(constant, constant, constant, constant, forward, backward, time, bits) == constant);
          CHECK(interpolation_extra(constant, constant, constant, constant, forward, backward, time, bits) == constant);
          CHECK(interpolation_blend(constant, constant, time, bits) == constant);
        }
  // Extra K is clamped against the main interval; E controls the other side.
  CHECK(interpolation_extra<T>(10, 30, 0, maximum, 255, 255, 0, bits) == 29);
  CHECK(interpolation_extra<T>(10, 30, 0, maximum, 255, 255, 256, bits) == 10);
  rejects([&] { interpolation_basic<T>(0, 0, 0, 0, -1, 0, 0, bits); });
  rejects([&] { interpolation_extra<T>(0, 0, 0, 0, 0, 256, 0, bits); });
  rejects([&] { interpolation_blend<T>(0, 0, -1, bits); });
  rejects([&] { interpolation_blend<T>(0, 0, 257, bits); });
}
void floating() {
  CHECK(interpolation_basic<float>(10, 21, 100, 200, 0, 0, 128, 32) == 15.5f);
  CHECK(interpolation_extra<float>(10, 30, 50, 0, 255, 0, 128, 32) == 20.0f);
  CHECK(interpolation_basic<float>(-10, -21, 100, 200, 0, 0, 128, 32) == -15.5f);
  CHECK(interpolation_blend<float>(10, 21, 128, 32) == 15.5f);
  const float equal = 1.75f + 0x1p-23f;
  CHECK(interpolation_blend(equal, equal, 85, 32) == 1.75f + 0x1p-22f);
  CHECK(std::signbit(interpolation_basic(-0.0f, -0.0f, -0.0f, -0.0f, 128, 128, 128, 32)));
  CHECK(std::signbit(interpolation_extra(-0.0f, -0.0f, -0.0f, -0.0f, 128, 128, 128, 32)));
  CHECK(std::signbit(interpolation_blend(-0.0f, -0.0f, 128, 32)));
  const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
  rejects([&] { interpolation_basic(0.0f, 0.0f, nan, 0.0f, 0, 0, 0, 32); });
  rejects([&] { interpolation_extra(0.0f, 0.0f, 0.0f, inf, 0, 0, 0, 32); });
  rejects([&] { interpolation_blend(0.0f, nan, 0, 32); });
  rejects([&] { interpolation_blend(inf, 0.0f, 256, 32); });
  const float maximum = std::numeric_limits<float>::max();
  rejects([&] { interpolation_blend(maximum, 0.0f, 0, 32); });
  rejects([&] { interpolation_basic(0.0f, maximum, 0.0f, 0.0f, 0, 0, 0, 32); });
}
void planes() {
  const std::uint16_t first[] = {10, 20, 777, 30, 40, 777};
  const std::uint16_t second[] = {21, 31, 777, 777, 41, 51, 777, 777};
  std::uint16_t output[] = {999, 999, 999, 999, 999, 999, 999, 999, 999, 999};
  const auto a = checked_plane(first, 2, 2, 3 * sizeof(first[0]), sizeof(first));
  const auto b = checked_plane(second, 2, 2, 4 * sizeof(second[0]), sizeof(second));
  const auto out = checked_plane(output, 2, 2, 5 * sizeof(output[0]), sizeof(output));
  interpolation_blend_planes(a, b, out, 128, 10);
  CHECK(output[0] == 15 && output[1] == 25 && output[5] == 35 && output[6] == 45);
  for (const int i : {2, 3, 4, 7, 8, 9})
    CHECK(output[i] == 999);
  rejects([&] { interpolation_blend_planes(a, b.subplane(0, 0, 1, 2), out, 128, 10); });
  rejects([&] { interpolation_blend_planes<std::uint16_t>(out, out, out, 128, 10); });
  rejects([&] { interpolation_blend<std::uint16_t>(1024, 0, 256, 10); });
  rejects([&] { interpolation_blend<std::uint8_t>(0, 0, 0, 10); });
  float f0[] = {1, 2}, f1[] = {3, std::numeric_limits<float>::quiet_NaN()}, result[] = {19, 19};
  rejects([&] {
    interpolation_blend_planes<float>(checked_plane(f0, 2, 1, sizeof(f0), sizeof(f0)),
                                      checked_plane(f1, 2, 1, sizeof(f1), sizeof(f1)),
                                      checked_plane(result, 2, 1, sizeof(result), sizeof(result)), 0, 32);
  });
  CHECK(result[0] == 19 && result[1] == 19);
}
} // namespace
int main() {
  try {
    integers<std::uint8_t>(8);
    integers<std::uint16_t>(10);
    integers<std::uint16_t>(16);
    floating();
    planes();
    std::cout << "Interpolation pixel specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
