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
using neo_mv::depan::estimate::FftProfile;
using neo_mv::depan::estimate::fft_lanes;
using neo_mv::depan::estimate::fft_profile_name;
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
void transforms(int width, int height, FftProfile profile) {
  const FftPlan plan(width, height, profile);
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
void examples(FftProfile profile) {
  FftPlan plan(4, 4, profile);
  std::vector<float> a(16), b(16);
  a[0] = 1; b[1] = 10;
  const auto c = plan.correlate(a, b);
  for (int i = 0; i < 16; ++i) CHECK(c[i] == (i == 1 ? 160 : 0));
  const auto reversed = plan.correlate(b, a);
  CHECK(reversed[3] == 160);
  std::fill(a.begin(), a.end(), 2);
  for (float value : plan.correlate(a, a)) CHECK(value == 1024);
  FftPlan odd(4, 3, profile);
  a.assign(12, 0); a[0] = 1;
  const auto spectrum = odd.forward(a);
  CHECK(spectrum.size() == 9);
  for (auto value : spectrum) CHECK(value == std::complex<float>(1, 0));
  const auto back = odd.inverse(spectrum);
  CHECK(back[0] == 12);
  for (std::size_t i = 1; i < back.size(); ++i) CHECK(back[i] == 0);
}
void concurrent_requests() {
  const std::array<FftPlan, 2> plans{{
      FftPlan(18, 5, FftProfile::scalar), FftPlan(18, 5, FftProfile::native)}};
  std::array<std::vector<float>, 3> input;
  std::array<std::array<std::vector<std::complex<float>>, 3>, 2> expected;
  for (std::size_t k = 0; k < input.size(); ++k) {
    input[k].resize(plans[0].real_count());
    for (std::size_t i = 0; i < input[k].size(); ++i)
      input[k][i] = float(int((i * 13 + k * 7) % 31) - 15) / 8;
    for (std::size_t p = 0; p < plans.size(); ++p)
      expected[p][k] = plans[p].forward(input[k]);
  }
  std::array<std::future<void>, 3> work;
  for (std::size_t k = 0; k < work.size(); ++k)
    work[k] = std::async(std::launch::async, [&, k] {
      for (int repeat = 0; repeat < 20; ++repeat) {
        const auto profile = repeat % 2 ? FftProfile::scalar : FftProfile::native;
        {
          FftPlan temporary(16, 4, profile);
          temporary.forward(std::vector<float>(64, float(k)));
        }
        for (std::size_t p = 0; p < plans.size(); ++p) {
          const auto actual = plans[p].forward(input[k]);
          CHECK(std::memcmp(actual.data(), expected[p][k].data(), actual.size() * sizeof(actual[0])) == 0);
        }
      }
    });
  for (auto& result : work) result.get();
}
void errors(FftProfile profile) {
  rejects([&] { FftPlan(0, 4, profile); });
  rejects([&] { FftPlan(3, 4, profile); });
  rejects([&] { FftPlan(4, 1, profile); });
  rejects([&] { FftPlan(INT32_MAX - 1, INT32_MAX, profile); });
  FftPlan plan(4, 3, profile);
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

struct Difference {
  long double absolute = 0;
  long double normalized = 0;
  bool exact = true;
  void observe(float scalar, float native, long double scale) {
    CHECK(std::isfinite(scalar) && std::isfinite(native));
    const auto error = std::abs(static_cast<long double>(scalar) - native);
    const auto denominator = (std::max)(1.0L, scale);
    absolute = (std::max)(absolute, error);
    normalized = (std::max)(normalized, error / denominator);
    exact = exact && std::memcmp(&scalar, &native, sizeof(float)) == 0;
    // Keep the existing FFT-boundary error budget. This does not permit
    // differences in peak selection, motion validity, or rendered images.
    CHECK(error <= 64 * std::numeric_limits<float>::epsilon() * denominator);
  }
};

void profile_differences() {
  Difference spectrum_difference, correlation_difference;
  for (const auto shape : {std::array<int, 2>{32, 16}, {66, 17}, {128, 32}}) {
    const FftPlan scalar(shape[0], shape[1], FftProfile::scalar);
    const FftPlan native(shape[0], shape[1], FftProfile::native);
    const auto count = scalar.real_count();
    std::vector<float> a(count), b(count);
    long double l1 = 0, energy_a = 0, energy_b = 0;
    for (std::size_t i = 0; i < count; ++i) {
      a[i] = float(int((i * 37 + i / 11 + 3) % 257) - 128) / 32;
      b[i] = float(int((i * 19 + i / 7 + 5) % 251) - 125) / 16;
      l1 += std::abs(a[i]);
      energy_a += static_cast<long double>(a[i]) * a[i];
      energy_b += static_cast<long double>(b[i]) * b[i];
    }
    const auto scalar_spectrum = scalar.forward(a);
    const auto native_spectrum = native.forward(a);
    CHECK(scalar_spectrum.size() == native_spectrum.size());
    for (std::size_t i = 0; i < scalar_spectrum.size(); ++i) {
      spectrum_difference.observe(scalar_spectrum[i].real(), native_spectrum[i].real(), l1);
      spectrum_difference.observe(scalar_spectrum[i].imag(), native_spectrum[i].imag(), l1);
    }
    const auto scalar_correlation = scalar.correlate(a, b);
    const auto native_correlation = native.correlate(a, b);
    CHECK(scalar_correlation.size() == native_correlation.size());
    // Cauchy-Schwarz bounds the absolute sum of products for every shift;
    // the inverse is unnormalized and contributes the additional count.
    const auto scale = count * std::sqrt(energy_a * energy_b);
    for (std::size_t i = 0; i < count; ++i)
      correlation_difference.observe(scalar_correlation[i], native_correlation[i], scale);
  }
  const auto report = [](const char* name, const Difference& difference) {
    std::cout << name << ": bit_exact=" << difference.exact
              << " max_absolute=" << difference.absolute
              << " max_normalized=" << difference.normalized << '\n';
  };
  report("scalar/native spectrum", spectrum_difference);
  report("scalar/native correlation", correlation_difference);
}
} // namespace
int main() {
  try {
    CHECK(fft_lanes(FftProfile::scalar) == 1);
    CHECK(fft_lanes(FftProfile::native) >= 1);
    std::vector<FftProfile> profiles = {FftProfile::scalar, FftProfile::native};
#if NEO_MV_FFT_X86_TARGETS
    for (auto target_profile : {FftProfile::sse2, FftProfile::avx2, FftProfile::avx512}) {
      if (fft_lanes(target_profile) > 1)
        profiles.push_back(target_profile);
    }
#endif
    for (const auto profile : profiles) {
      CHECK(fft_profile_name(profile) != nullptr);
      std::cout << "FFT profile=" << fft_profile_name(profile)
                << " lanes=" << fft_lanes(profile) << '\n';
      examples(profile);
      for (int width : {2, 4, 6, 10, 14, 16, 18})
        for (int height : {2, 3, 4, 5, 7, 8}) transforms(width, height, profile);
      errors(profile);
    }
    concurrent_requests();
    profile_differences();
    std::cout << "DepanEstimate FFT profile specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
