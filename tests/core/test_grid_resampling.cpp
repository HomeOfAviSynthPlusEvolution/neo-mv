#include "core/mask/grid_resampling.hpp"
#ifdef NEO_MV_TEST_HIGHWAY
#include "highway/grid_resampling.hpp"
#endif

#include <string>
#include <cstring>
#include <iostream>
#include <vector>

namespace {
using namespace neo_mv;
#ifdef NEO_MV_TEST_HIGHWAY
using TestGridPlan = simd::GridResamplingPlan;
#else
using TestGridPlan = GridResamplingPlan;
#endif
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
  TestGridPlan({2, 1, 4, 4, 0, 0, 8, 4}).resize(input.read(), output.view(), 8);
  TestGridPlan({2, 1, 4, 4, 0, 0, 6, 4}).resize(input.read(), cropped.view(), 8);
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
  TestGridPlan({2, 1, 4, 4, 0, 0, 8, 4}).resize(signed_input.read(), signed_output.view(), 16);
  for (int x = 0; x < 8; ++x)
    CHECK(signed_output.view().row(0)[x] == (x < 4 ? -1 : 0));
  Buffer<std::uint8_t> square(2, 2), enlarged(6, 6);
  square.view().row(0)[0] = 0;
  square.view().row(0)[1] = 1;
  square.view().row(1)[0] = 0;
  square.view().row(1)[1] = 3;
  TestGridPlan({2, 2, 3, 3, 0, 0, 6, 6}).resize(square.read(), enlarged.view(), 8);
  CHECK(enlarged.view().row(2)[2] == 0); // Horizontal rows round to 0 and 1 before the vertical pass.
  Buffer<std::uint8_t> cropped_square(6, 3);
  TestGridPlan({2, 2, 3, 3, 0, 0, 6, 3}).resize(square.read(), cropped_square.view(), 8);
  // Full 6x6 selects horizontal first. Visible 6x3 would incorrectly select
  // vertical first on equality, producing 1 here instead of 0.
  CHECK(cropped_square.view().row(2)[2] == 0);
  for (int y = 0; y < 3; ++y)
    for (int x = 0; x < 6; ++x)
      CHECK(cropped_square.view().row(y)[x] == enlarged.view().row(y)[x]);
  TestGridPlan({2, 2, 3, 2, 0, 1, 6, 3}).resize(square.read(), cropped_square.view(), 8);
  CHECK(cropped_square.view().row(1)[3] == 1); // Equality: vertical gives 1, horizontal gives 2.
  cropped_square.gaps();
  Buffer<float> fp(2, 1), out(8, 4);
  fp.view().row(0)[0] = 0;
  fp.view().row(0)[1] = 1;
  TestGridPlan({2, 1, 4, 4, 0, 0, 8, 4}).resize(fp.read(), out.view(), 32);
  const float values[] = {0, 0, 0.125f, 0.375f, 0.625f, 0.875f, 1, 1};
  for (int x = 0; x < 8; ++x)
    CHECK(out.view().row(0)[x] == values[x]);
  for (float value : {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::denorm_min()}) {
    fp.view().row(0)[0] = fp.view().row(0)[1] = value;
    TestGridPlan({2, 1, 3, 3, 0, 0, 8 - 2, 4 - 1}).resize(fp.read(), out.view().subplane(0, 0, 6, 3), 32);
    for (int x = 0; x < 6; ++x)
      CHECK(out.view().row(0)[x] == value);
  }
}

void integer_passes() {
  Buffer<std::uint16_t> input(2, 1), output(6, 4);
  input.view().row(0)[0] = 0;
  input.view().row(0)[1] = 65535;
  TestGridPlan({2, 1, 3, 4, 0, 0, 6, 4}).resize(input.read(), output.view(), 16);
  const int expected[] = {0, 0, 21844, 43691, 65535, 65535};
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 6; ++x)
      CHECK(output.view().row(y)[x] == expected[x]);
  output.gaps();
  Buffer<std::int16_t> signed_input(2, 1), signed_output(3, 1);
  for (const auto values : {std::array<std::int16_t, 3>{-1, 0, 0}, {-2, -1, -1}, {-32768, 32767, 0}}) {
    signed_input.view().row(0)[0] = values[0];
    signed_input.view().row(0)[1] = values[1];
    TestGridPlan({2, 1, 2, 1, 1, 0, 3, 1}).resize(signed_input.read(), signed_output.view(), 16);
    CHECK(signed_output.view().row(0)[0] == values[0]);
    CHECK(signed_output.view().row(0)[1] == values[2]); // Negative half rounds toward positive infinity.
    CHECK(signed_output.view().row(0)[2] == values[1]);
    signed_output.gaps();
  }
  Buffer<std::uint16_t> square(2, 2), enlarged(32, 3);
  square.view().row(0)[0] = 0;
  square.view().row(0)[1] = 1170;
  square.view().row(1)[0] = 2340;
  square.view().row(1)[1] = 8191;
  TestGridPlan({2, 2, 16, 2, 0, 1, 32, 3}).resize(square.read(), enlarged.view(), 16);
  CHECK(enlarged.view().row(1)[9] == 1499); // Horizontal first would give 1500.
  enlarged.gaps();
  Buffer<float> fp(2, 2), fp_output(32, 3);
  for (int y = 0; y < 2; ++y)
    for (int x = 0; x < 2; ++x)
      fp.view().row(y)[x] = static_cast<float>(square.view().row(y)[x]);
  TestGridPlan({2, 2, 16, 2, 0, 1, 32, 3}).resize(fp.read(), fp_output.view(), 32);
  CHECK(fp_output.view().row(1)[9] == 1499.109375f); // Float retains unrounded double interpolation.
  fp_output.gaps();
}

void exact_wide_arithmetic() {
  // Literal rational examples distinguish coefficient ties-to-even from sample
  // half-up. Scaling preserves each rational without forming a Q*R product.
  struct Example {
    std::uint64_t numerator, denominator;
    std::uint32_t result;
  };
  const Example examples[] = {{0, 32768, 0}, {1, 32768, 0},         {3, 32768, 2}, {5, 32768, 2},
                              {7, 32768, 4}, {32767, 32768, 16384}, {1, 65536, 0}, {3, 65536, 1},
                              {5, 65536, 1}, {7, 65536, 2},         {1, 3, 5461},  {2, 3, 10923}};
  for (const auto example : examples) {
    CHECK(grid_detail::coefficient(example.numerator, example.denominator) == example.result);
    constexpr std::uint64_t scale = std::uint64_t(1) << 46;
    CHECK(grid_detail::coefficient(example.numerator * scale, example.denominator * scale) == example.result);
  }
  CHECK(grid_detail::coefficient((std::uint64_t(1) << 62) - 1, std::uint64_t(1) << 62) == 16384);
  CHECK(grid_detail::horizontal_first(6, 6, 2, 2));
  CHECK(!grid_detail::horizontal_first(32, 3, 2, 2));
  CHECK(!grid_detail::horizontal_first(6, 3, 2, 2)); // Equality is vertical first.
  const auto huge = std::int64_t(INT32_MAX) * INT32_MAX;
  CHECK(grid_detail::horizontal_first(huge, huge, INT32_MAX, INT32_MAX));
  CHECK(!grid_detail::horizontal_first(INT32_MAX, 1, INT32_MAX, 1));
  Buffer<std::uint16_t> input(1, 1), output(1, 1);
  input.view().row(0)[0] = 65535;
  TestGridPlan({1, 1, INT32_MAX, INT32_MAX, 0, 0, 1, 1}).resize(input.read(), output.view(), 16);
  CHECK(output.view().row(0)[0] == 65535);
  // Even the maximum int32 Nx,Bx geometry is valid at plan construction;
  // constructing the plan never allocates its huge covered image.
  TestGridPlan({INT32_MAX, INT32_MAX, INT32_MAX, INT32_MAX, 0, 0, 1, 1});
}

void failures() {
  rejects([] { TestGridPlan({2, 1, 4, 4, 0, 0, 9, 4}); });
  rejects([] { TestGridPlan({2, 1, 4, 4, 4, 0, 8, 4}); });
  Buffer<float> input(2, 1), output(8, 4);
  const TestGridPlan plan({2, 1, 4, 4, 0, 0, 8, 4});
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
    integer_passes();
    exact_wide_arithmetic();
    failures();
    std::cout << "Exact grid resampling checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
