#include "highway/grid_resampling.hpp"
#include "core/mask/grid_resampling.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <random>
#include <vector>

namespace {
using namespace neo_mv;
void check(bool value, int line) {
  if (!value)
    throw std::runtime_error("grid resampling assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& fn) {
  bool caught = false;
  try {
    fn();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
template <class T>
struct Buffer {
  int width, height, stride;
  std::vector<T> data;
  Buffer(int w, int h, int gap = 3) : width(w), height(h), stride(w + gap), data(std::size_t(stride) * h + 1, T(19)) {}
  span2d::Plane<T> view() {
    return checked_plane(data.data() + 1, width, height, std::ptrdiff_t(stride) * sizeof(T),
                         (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
  void gaps() const {
    CHECK(data[0] == T(19));
    for (int y = 0; y < height; ++y)
      for (int x = width; x < stride; ++x)
        CHECK(data[1 + std::size_t(y) * stride + x] == T(19));
  }
};

void examples() {
  Buffer<std::uint8_t> input(2, 1), output(8, 4), cropped(6, 4);
  input.view().row(0)[0] = 0;
  input.view().row(0)[1] = 100;
  simd::GridResamplingPlan({2, 1, 4, 4, 0, 0, 8, 4}).resize(input.read(), output.view(), 8);
  simd::GridResamplingPlan({2, 1, 4, 4, 0, 0, 6, 4}).resize(input.read(), cropped.view(), 8);
  const int expected[] = {0, 0, 13, 38, 63, 88, 100, 100};
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 8; ++x) {
      CHECK(output.view().row(y)[x] == expected[x]);
      if (x < 6)
        CHECK(cropped.view().row(y)[x] == expected[x]);
    }
  output.gaps();
  cropped.gaps();
  input.gaps();
  Buffer<std::int16_t> signed_input(2, 1), signed_output(8, 4);
  signed_input.view().row(0)[0] = -1;
  signed_input.view().row(0)[1] = 0;
  simd::GridResamplingPlan({2, 1, 4, 4, 0, 0, 8, 4}).resize(signed_input.read(), signed_output.view(), 16);
  for (int x = 0; x < 8; ++x)
    CHECK(signed_output.view().row(0)[x] == (x < 4 ? -1 : 0));
  Buffer<std::uint8_t> square(2, 2), enlarged(6, 6);
  square.view().row(0)[0] = 0;
  square.view().row(0)[1] = 1;
  square.view().row(1)[0] = 0;
  square.view().row(1)[1] = 3;
  simd::GridResamplingPlan({2, 2, 3, 3, 0, 0, 6, 6}).resize(square.read(), enlarged.view(), 8);
  CHECK(enlarged.view().row(2)[2] == 1); // Double horizontal rounding would give zero.
  Buffer<float> fp(2, 1), out(8, 4);
  fp.view().row(0)[0] = 0;
  fp.view().row(0)[1] = 1;
  simd::GridResamplingPlan({2, 1, 4, 4, 0, 0, 8, 4}).resize(fp.read(), out.view(), 32);
  const float values[] = {0, 0, 0.125f, 0.375f, 0.625f, 0.875f, 1, 1};
  for (int x = 0; x < 8; ++x)
    CHECK(out.view().row(0)[x] == values[x]);
  for (float value : {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::denorm_min()}) {
    fp.view().row(0)[0] = fp.view().row(0)[1] = value;
    simd::GridResamplingPlan({2, 1, 3, 3, 0, 0, 8 - 2, 4 - 1}).resize(fp.read(), out.view().subplane(0, 0, 6, 3), 32);
    for (int x = 0; x < 6; ++x)
      CHECK(out.view().row(0)[x] == value);
  }
}

void exact_wide_arithmetic() {
  std::mt19937 rng(20260920);
  // Scaling both coordinates leaves the rational weights unchanged. The small
  // independent formula fits uint64; its scaled counterpart needs >128 bits.
  for (int i = 0; i < 1000; ++i) {
    const std::uint64_t dx = 2 * (rng() % 32 + 1), dy = 2 * (rng() % 32 + 1), rx = rng() % dx, ry = rng() % dy;
    std::array<std::uint32_t, 4> q{};
    for (auto& value : q)
      value = rng() % 65536;
    const auto n = (dx - rx) * (dy - ry) * q[0] + rx * (dy - ry) * q[1] + (dx - rx) * ry * q[2] + rx * ry * q[3];
    const auto expected = (n + dx * dy / 2) / (dx * dy);
    CHECK(grid_detail::rounded_integer(dx, dy, rx, ry, q) == expected);
    constexpr std::uint64_t scale = 100000000000000000ULL;
    CHECK(grid_detail::rounded_integer(dx * scale, dy * scale, rx * scale, ry * scale, q) == expected);
  }
  Buffer<std::uint16_t> input(1, 1), output(1, 1);
  input.view().row(0)[0] = 65535;
  simd::GridResamplingPlan({1, 1, INT32_MAX, INT32_MAX, 0, 0, 1, 1}).resize(input.read(), output.view(), 16);
  CHECK(output.view().row(0)[0] == 65535);
  // Even the maximum int32 Nx,Bx geometry is valid at plan construction;
  // constructing the plan never allocates its huge covered image.
  simd::GridResamplingPlan({INT32_MAX, INT32_MAX, INT32_MAX, INT32_MAX, 0, 0, 1, 1});
}

void failures() {
  rejects([] { simd::GridResamplingPlan({2, 1, 4, 4, 0, 0, 9, 4}); });
  rejects([] { simd::GridResamplingPlan({2, 1, 4, 4, 4, 0, 8, 4}); });
  Buffer<float> input(2, 1), output(8, 4);
  const simd::GridResamplingPlan plan({2, 1, 4, 4, 0, 0, 8, 4});
  input.view().row(0)[0] = 0;
  input.view().row(0)[1] = std::numeric_limits<float>::quiet_NaN();
  const auto before = output.data;
  rejects([&] { plan.resize(input.read(), output.view(), 32); });
  CHECK(std::memcmp(before.data(), output.data.data(), before.size() * sizeof(float)) == 0);
  rejects([&] { plan.resize(output.read().subplane(0, 0, 2, 1), output.view(), 32); });
  Buffer<std::uint16_t> u16(2, 1), out16(8, 4);
  u16.view().row(0)[1] = 1024;
  rejects([&] { plan.resize(u16.read(), out16.view(), 10); });
  rejects([&] { plan.resize(input.read(), output.view(), 16); });
}
} // namespace
int main() {
  try {
    examples();
    exact_wide_arithmetic();
    failures();
    std::cout << "Exact grid resampling checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
