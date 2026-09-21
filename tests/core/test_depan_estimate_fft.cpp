#include "core/depan/estimate_fft.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using neo_mv::depan::estimate::FftPlan;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("DepanEstimate FFT assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try { call(); }
  catch (const std::invalid_argument&) { caught = true; }
  catch (const std::overflow_error&) { caught = true; }
  CHECK(caught);
}
void close(float actual, long double expected, long double scale) {
  CHECK(std::isfinite(actual));
  // Fixed small-shape float-profile bound, including cancellation near zero.
  CHECK(std::abs(static_cast<long double>(actual) - expected) <=
        64 * std::numeric_limits<float>::epsilon() * (std::max)(1.0L, scale));
}
void transforms(int width, int height) {
  const FftPlan plan(width, height);
  const auto count = std::size_t(width) * height;
  CHECK(plan.real_count() == count);
  CHECK(plan.complex_count() == std::size_t(width / 2 + 1) * height);
  std::vector<float> a(count), b(count);
  for (std::size_t i = 0; i < count; ++i) {
    a[i] = float(int((i * 13 + 7) % 31) - 15) / 8;
    b[i] = float(int((i * 17 + 3) % 23) - 11) / 4;
  }
  const auto saved_a = a, saved_b = b;
  const auto spectrum = plan.forward(a);
  long double scale = 0;
  for (float value : a) scale += std::abs(value);
  const long double pi = std::acos(-1.0L);
  for (int ky = 0; ky < height; ++ky)
    for (int kx = 0; kx <= width / 2; ++kx) {
      std::complex<long double> expected{};
      for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
          const auto angle = -2 * pi * (static_cast<long double>(kx) * x / width +
                                        static_cast<long double>(ky) * y / height);
          expected += static_cast<long double>(a[std::size_t(y) * width + x]) *
                      std::complex<long double>(std::cos(angle), std::sin(angle));
        }
      const auto actual = spectrum[std::size_t(ky) * (width / 2 + 1) + kx];
      close(actual.real(), expected.real(), scale);
      close(actual.imag(), expected.imag(), scale);
    }
  const auto before = spectrum;
  const auto back = plan.inverse(spectrum);
  for (std::size_t i = 0; i < count; ++i)
    close(back[i], a[i] * static_cast<long double>(count), scale * count);
  CHECK(std::memcmp(spectrum.data(), before.data(), spectrum.size() * sizeof(spectrum[0])) == 0);
  const auto correlation = plan.correlate(a, b);
  for (int j = 0; j < height; ++j)
    for (int i = 0; i < width; ++i) {
      long double sum = 0, absolute = 0;
      for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
          const auto product = static_cast<long double>(a[std::size_t(y) * width + x]) *
                               b[std::size_t((y + j) % height) * width + (x + i) % width];
          sum += product;
          absolute += std::abs(product);
        }
      close(correlation[std::size_t(j) * width + i], count * sum, count * absolute);
    }
  CHECK(a == saved_a && b == saved_b);
}
void examples() {
  FftPlan plan(4, 4);
  std::vector<float> a(16), b(16);
  a[0] = 1; b[1] = 10;
  const auto c = plan.correlate(a, b);
  for (int i = 0; i < 16; ++i) CHECK(c[i] == (i == 1 ? 160 : 0));
  const auto reversed = plan.correlate(b, a);
  CHECK(reversed[3] == 160);
  std::fill(a.begin(), a.end(), 2);
  for (float value : plan.correlate(a, a)) CHECK(value == 1024);
  FftPlan odd(4, 3);
  a.assign(12, 0); a[0] = 1;
  const auto spectrum = odd.forward(a);
  CHECK(spectrum.size() == 9);
  for (auto value : spectrum) CHECK(value == std::complex<float>(1, 0));
  const auto back = odd.inverse(spectrum);
  CHECK(back[0] == 12);
  for (std::size_t i = 1; i < back.size(); ++i) CHECK(back[i] == 0);
}
void concurrent_requests() {
  const FftPlan plan(6, 5);
  std::array<std::vector<float>, 3> input;
  std::array<std::vector<std::complex<float>>, 3> expected;
  for (std::size_t k = 0; k < input.size(); ++k) {
    input[k].resize(plan.real_count());
    input[k][3 * k + 1] = float(k + 1);
    expected[k] = plan.forward(input[k]);
  }
  std::array<std::future<void>, 3> work;
  for (std::size_t k = 0; k < work.size(); ++k)
    work[k] = std::async(std::launch::async, [&, k] {
      for (int repeat = 0; repeat < 20; ++repeat) {
        FftPlan temporary(4, 3);
        temporary.forward(std::vector<float>(12, float(k)));
        const auto actual = plan.forward(input[k]);
        CHECK(std::memcmp(actual.data(), expected[k].data(), actual.size() * sizeof(actual[0])) == 0);
      }
    });
  for (auto& result : work) result.get();
}
void errors() {
  rejects([] { FftPlan(0, 4); });
  rejects([] { FftPlan(3, 4); });
  rejects([] { FftPlan(4, 1); });
  rejects([] { FftPlan(INT32_MAX - 1, INT32_MAX); });
  FftPlan plan(4, 3);
  rejects([&] { plan.forward({}); });
  rejects([&] { plan.inverse({}); });
  std::vector<float> input(12, 0);
  input.back() = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { plan.forward(input); });
  std::vector<std::complex<float>> spectrum(9);
  spectrum.back() = {0, std::numeric_limits<float>::infinity()};
  rejects([&] { plan.inverse(spectrum); });
  input.assign(12, std::numeric_limits<float>::max());
  rejects([&] { plan.forward(input); });
  input.assign(12, 0); input[0] = 1e30f;
  rejects([&] { plan.correlate(input, input); });
}
} // namespace
int main() {
  try {
    examples();
    for (int width : {2, 4, 6, 10, 14})
      for (int height : {2, 3, 5, 7}) transforms(width, height);
    concurrent_requests();
    errors();
    std::cout << "DepanEstimate scalar FFT specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
