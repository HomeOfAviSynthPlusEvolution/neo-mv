#include "highway/depan_estimate.hpp"
#include "highway/estimate_image.hpp"
#include "highway/rows.hpp"
#include "core/depan/numeric.hpp"
#include "hwy/targets.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <random>
#include <string>
#include <type_traits>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace {
using Complex = std::complex<float>;
using namespace neo_mv;
void check(bool value, int line) {
  if (!value)
    throw std::runtime_error("DepanEstimate SIMD assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class T = Complex>
struct EndRow {
  void* memory;
  T* data;
  std::size_t page, committed;
  explicit EndRow(std::size_t count) {
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    page = info.dwPageSize;
#else
    const auto size = sysconf(_SC_PAGESIZE);
    CHECK(size > 0);
    page = static_cast<std::size_t>(size);
#endif
    committed = ((count * sizeof(T) + page - 1) / page) * page;
#if defined(_WIN32)
    memory = VirtualAlloc(nullptr, committed + page, MEM_RESERVE, PAGE_NOACCESS);
    if (!memory)
      throw std::bad_alloc();
    if (!VirtualAlloc(memory, committed, MEM_COMMIT, PAGE_READWRITE)) {
      VirtualFree(memory, 0, MEM_RELEASE);
      throw std::bad_alloc();
    }
#else
    memory = mmap(nullptr, committed + page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (memory == MAP_FAILED)
      throw std::bad_alloc();
    if (mprotect(memory, committed, PROT_READ | PROT_WRITE) != 0) {
      munmap(memory, committed + page);
      throw std::bad_alloc();
    }
#endif
    data = reinterpret_cast<T*>(static_cast<unsigned char*>(memory) + committed - count * sizeof(T));
    for (std::size_t i = 0; i < count; ++i)
      ::new (data + i) T(0);
  }
  ~EndRow() {
#if defined(_WIN32)
    VirtualFree(memory, 0, MEM_RELEASE);
#else
    munmap(memory, committed + page);
#endif
  }
  EndRow(const EndRow&) = delete;
  EndRow& operator=(const EndRow&) = delete;
};
Complex expected(Complex a, Complex b) {
  using namespace depan;
  if (simd::estimate::native_fma()) {
    const float ii = mul(a.imag(), b.imag()), ri = mul(a.real(), b.imag());
    return {finite(std::fma(a.real(), b.real(), ii)),
            finite(std::fma(-a.imag(), b.real(), ri))};
  }
  return {add(mul(a.real(), b.real()), mul(a.imag(), b.imag())), sub(mul(a.real(), b.imag()), mul(a.imag(), b.real()))};
}
void products() {
  std::mt19937 random(71821);
  const auto sample = [&] {
    return std::ldexp(float(int(random() % 2000001) - 1000000), int(random() % 41) - 30);
  };
  simd::estimate::product(nullptr, nullptr, 0);
  for (const int count : {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129, 257}) {
    EndRow a(count + 1), b(count + 1);
    const Complex sentinel{123, -456};
    a.data[0] = b.data[0] = sentinel;
    std::vector<Complex> oracle(count), original(count);
    for (int i = 0; i < count; ++i) {
      a.data[i + 1] = {sample(), sample()};
      b.data[i + 1] = {sample(), sample()};
      if (i % 7 == 0)
        a.data[i + 1] = {0.0f, -0.0f};
      if (i % 11 == 0)
        b.data[i + 1] = {std::numeric_limits<float>::denorm_min(), -0.0f};
      oracle[i] = expected(a.data[i + 1], b.data[i + 1]);
      original[i] = b.data[i + 1];
    }
    simd::estimate::product(a.data + 1, b.data + 1, count);
    CHECK(std::memcmp(a.data + 1, oracle.data(), count * sizeof(Complex)) == 0);
    CHECK(std::memcmp(b.data + 1, original.data(), count * sizeof(Complex)) == 0);
    CHECK(a.data[0] == sentinel && b.data[0] == sentinel);
  }
}
void errors() {
  const auto max = std::numeric_limits<float>::max(), inf = std::numeric_limits<float>::infinity();
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  // Non-finite inputs, individual products, final addition and subtraction.
  const std::array<std::array<Complex, 2>, 7> bad{{{{{nan, 0}, {1, 0}}},
                                                   {{{1, 0}, {0, inf}}},
                                                   {{{max, max}, {2, -2}}},
                                                   {{{max, max}, {1, 1}}},
                                                   {{{max, -max}, {1, 1}}},
                                                   {{{0, inf}, {1, 0}}},
                                                   {{{1, 0}, {nan, 0}}}}};
  for (const auto& pair : bad)
    for (const int index : {0, 32}) {
      EndRow a(33), b(33);
      for (int i = 0; i < 33; ++i) {
        a.data[i] = {1, 2};
        b.data[i] = {3, 4};
      }
      a.data[index] = pair[0];
      b.data[index] = pair[1];
      bool caught = false;
      try {
        simd::estimate::product(a.data, b.data, 33);
      } catch (const std::invalid_argument&) {
        caught = true;
      }
      CHECK(caught);
    }
}
void correlations() {
  std::mt19937 random(31782);
  for (int width : {2, 4, 6, 10, 14, 32, 66})
    for (int height : {2, 3, 7}) {
      depan::estimate::FftPlan plan(width, height);
      std::vector<float> a(plan.real_count()), b(a.size());
      for (std::size_t i = 0; i < a.size(); ++i) {
        a[i] = float(int(random() % 1001) - 500) / 127;
        b[i] = float(int(random() % 1001) - 500) / 255;
      }
      const depan::estimate::FftPlan native(width, height, depan::estimate::FftProfile::native);
      // Isolate the product differential from any FFT profile rounding.
      const auto scalar = native.correlate(a, b), vector = simd::estimate::correlate(native, a, b);
      CHECK(scalar.size() == vector.size());
      if (simd::estimate::native_fma()) {
        for (std::size_t i = 0; i < scalar.size(); ++i) {
          CHECK(std::isfinite(scalar[i]) && std::isfinite(vector[i]));
          const auto diff = std::abs(double(scalar[i]) - double(vector[i]));
          const auto limit = 1e-3 + 1e-4 * (std::max)(std::abs(double(scalar[i])), std::abs(double(vector[i])));
          CHECK(diff <= limit);
        }
      } else {
        CHECK(std::memcmp(scalar.data(), vector.data(), scalar.size() * sizeof(float)) == 0);
      }
    }
}
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
bool same(float a, float b) {
  return std::memcmp(&a, &b, sizeof(float)) == 0;
}
template <class T>
void images(int bits) {
  namespace scalar = depan::estimate;
  namespace vector = simd::estimate;
  const float maximum = scalar::image_detail::maximum<T>(bits);
  for (int width : {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 129, 257}) {
    // Source and destination end exactly at a guard page; row tails cannot overread.
    EndRow<T> source(width + 1), output(width + 1);
    EndRow<float> converted(width + 1), surface(width + 1);
    source.data[0] = output.data[0] = T(77);
    converted.data[0] = surface.data[0] = 77;
    for (int x = 0; x < width; ++x) {
      if constexpr (std::is_same_v<T, float>)
        source.data[x + 1] = x % 3 == 0 ? -0.0f : float(x - 10) / 7;
      else
        source.data[x + 1] = T((x * 113) % (int(maximum) + 1));
      surface.data[x + 1] = float(x % 19);
    }
    vector::extract_row(source.data + 1, converted.data + 1, width, maximum);
    CHECK(vector::matches_row(source.data + 1, converted.data + 1, width));
    const float last = converted.data[width];
    converted.data[width] = last + 1;
    CHECK(!vector::matches_row(source.data + 1, converted.data + 1, width));
    converted.data[width] = last;
    if constexpr (std::is_same_v<T, float>) {
      converted.data[1] = 0.0f;
      CHECK(!vector::matches_row(source.data + 1, converted.data + 1, width));
      converted.data[1] = -0.0f;
    }
    for (int x = 0; x < width; ++x)
      CHECK(same(converted.data[x + 1], float(source.data[x + 1])));
    vector::display_row(surface.data + 1, output.data + 1, width, 0, maximum / 18, maximum);
    for (int x = 0; x < width; ++x) {
      const auto q = depan::mul(surface.data[x + 1], maximum / 18);
      const T expected = static_cast<T>(q);
      CHECK(std::memcmp(output.data + x + 1, &expected, sizeof(T)) == 0);
    }
    CHECK(source.data[0] == T(77) && output.data[0] == T(77));
    CHECK(converted.data[0] == 77 && surface.data[0] == 77);

    const int stride = width + 5, height = 4;
    const T unused = std::is_same_v<T, float> ? T(std::numeric_limits<float>::quiet_NaN()) : T(77);
    std::vector<T> padded(stride * height, unused), a(stride * height, T(77)), b = a;
    auto source_view =
        checked_plane<const T>(padded.data(), width + 2, height, stride * sizeof(T), padded.size() * sizeof(T));
    auto av = checked_plane(a.data(), width + 2, height, stride * sizeof(T), a.size() * sizeof(T));
    auto bv = checked_plane(b.data(), width + 2, height, stride * sizeof(T), b.size() * sizeof(T));
    for (int y = 1; y <= 2; ++y)
      std::copy_n(source.data + 1, width, padded.data() + y * stride + 1);
    const auto before_source = padded;
    auto sa = scalar::extract_window(source_view, 1, 1, width, 2, bits);
    auto sb = vector::extract_window(source_view, 1, 1, width, 2, bits);
    CHECK(std::memcmp(sa.data(), sb.data(), sa.size() * sizeof(float)) == 0);
    CHECK(vector::matches_window(source_view, 1, 1, width, 2, sb));
    sb.back() += 1;
    CHECK(!vector::matches_window(source_view, 1, 1, width, 2, sb));
    CHECK(std::memcmp(padded.data(), before_source.data(), padded.size() * sizeof(T)) == 0);
    for (std::size_t i = 0; i < sa.size(); ++i)
      sa[i] = float(i % 23);
    scalar::display_surface(av, sa, 1, 1, width, 2, bits);
    vector::display_surface(bv, sa, 1, 1, width, 2, bits);
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
    const auto before = b;
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
      sa.back() = bad;
      rejects([&] { vector::display_surface(bv, sa, 1, 1, width, 2, bits); });
      CHECK(std::memcmp(before.data(), b.data(), b.size() * sizeof(T)) == 0);
    }
    sa.assign(sa.size(), 1);
    rejects([&] { vector::display_surface(bv, sa, 1, 1, width, 2, bits); });
    CHECK(std::memcmp(before.data(), b.data(), b.size() * sizeof(T)) == 0);
    sa.front() = -std::numeric_limits<float>::max();
    sa.back() = std::numeric_limits<float>::max();
    rejects([&] { vector::display_surface(bv, sa, 1, 1, width, 2, bits); });
    CHECK(std::memcmp(before.data(), b.data(), b.size() * sizeof(T)) == 0);
    if constexpr (std::is_same_v<T, float>) {
      source.data[width] = std::numeric_limits<float>::infinity();
      rejects([&] { vector::extract_row(source.data + 1, converted.data + 1, width, maximum); });
    } else if (bits < int(sizeof(T) * 8)) {
      source.data[width] = T(int(maximum) + 1);
      rejects([&] { vector::extract_row(source.data + 1, converted.data + 1, width, maximum); });
    }
  }
}
void scans() {
  namespace scalar = depan::estimate;
  namespace vector = simd::estimate;
  vector::samples_finite(nullptr, 0);
  for (int count : {1, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 65, 129, 257}) {
    EndRow<float> values(count);
    for (int i = 0; i < count; ++i)
      values.data[i] = i % 3 == 0 ? 16777216.0f : i % 3 == 1 ? 1.0f : -16777216.0f;
    float ss = 0, vs = 0, sm = -1, vm = -1;
    const auto si = scalar::motion_detail::ScalarScan::scan(values.data, count, ss, sm);
    const auto vi = vector::scan_row(values.data, count, vs, vm);
    CHECK(si == vi && same(ss, vs) && same(sm, vm));
    vector::samples_finite(values.data, count);
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
      values.data[count - 1] = bad;
      rejects([&] { vector::samples_finite(values.data, count); });
    }
    for (int i = 0; i < count; ++i)
      values.data[i] = i % 2 ? 0.0f : -0.0f;
    float minimum, maximum;
    vector::extrema(values.data, count, minimum, maximum);
    CHECK(same(minimum, -0.0f) && same(maximum, -0.0f));
    values.data[0] = 0.0f;
    vector::extrema(values.data, count, minimum, maximum);
    CHECK(same(minimum, 0.0f) && same(maximum, 0.0f));
  }
  constexpr int width = 66, height = 7, stride = 69;
  std::vector<float> surface(stride * height, std::numeric_limits<float>::quiet_NaN());
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      surface[y * stride + x] = float((x * 13 + y * 17) % 101) / 8;
  surface[2] = surface[width - 1] = surface[stride + 1] = 100;
  auto view =
      checked_plane<const float>(surface.data(), width, height, stride * sizeof(float), surface.size() * sizeof(float));
  for (int mx : {0, 1, 15, 16, 31, 32})
    for (int my : {0, 1, 2}) {
      const auto a = scalar::find_peak(view, mx, my, 0.5f, 0);
      const auto b = scalar::find_peak<vector::MotionScan>(view, mx, my, 0.5f, 0);
      CHECK(a.ix == b.ix && a.iy == b.iy && a.dx == b.dx && a.dy == b.dy);
      CHECK(same(a.confidence, b.confidence) && a.good == b.good);
      if (mx >= 2)
        CHECK(b.ix == 2 && b.iy == 0);
      const auto ar = scalar::refine_motion(view, a, mx, my, 1.1f, true, false);
      const auto br = scalar::refine_motion<vector::MotionScan>(view, b, mx, my, 1.1f, true, false);
      CHECK(same(ar.dx, br.dx) && same(ar.dy, br.dy) && same(ar.confidence, br.confidence) && ar.good == br.good);
    }
  surface[3 * stride + 33] = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { scalar::find_peak<vector::MotionScan>(view, 1, 1, 0, 0); });
  rejects([&] { scalar::refine_motion<vector::MotionScan>(view, {}, 1, 1, 1, false, false); });
}
std::vector<std::size_t> validation_counts;
void recording_validator(const float* data, std::size_t count) {
  validation_counts.push_back(count);
  simd::estimate::samples_finite(data, count);
}
void fft_validation() {
  const depan::estimate::FftPlan plan(10, 3);
  std::vector<float> input(plan.real_count(), 1);
  validation_counts.clear();
  const auto frequency = plan.forward(input, recording_validator);
  CHECK(validation_counts == std::vector<std::size_t>({plan.real_count(), 2 * plan.complex_count()}));
  validation_counts.clear();
  const auto result = plan.inverse(frequency, recording_validator);
  CHECK(validation_counts == std::vector<std::size_t>({2 * plan.complex_count(), plan.real_count()}));
  const auto expected = plan.inverse(frequency);
  CHECK(std::memcmp(result.data(), expected.data(), result.size() * sizeof(float)) == 0);
  validation_counts.clear();
  rejects([&] { plan.forward({}, recording_validator); });
  CHECK(validation_counts.empty());
  input.back() = std::numeric_limits<float>::infinity();
  rejects([&] { plan.forward(input, recording_validator); });
  CHECK(validation_counts == std::vector<std::size_t>({plan.real_count()}));
  auto bad_frequency = frequency;
  bad_frequency.back().imag(std::numeric_limits<float>::quiet_NaN());
  validation_counts.clear();
  rejects([&] { plan.inverse(bad_frequency, recording_validator); });
  CHECK(validation_counts == std::vector<std::size_t>({2 * plan.complex_count()}));
}
} // namespace
int main() {
  try {
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      CHECK(std::strcmp(simd::detail::target_name(), hwy::TargetName(target)) == 0);
      std::cout << "Testing " << hwy::TargetName(target) << '\n';
      products();
      errors();
      correlations();
      images<std::uint8_t>(8);
      for (int bits : {9, 10, 12, 14, 16})
        images<std::uint16_t>(bits);
      images<float>(32);
      scans();
      fft_validation();
    }
    hwy::SetSupportedTargetsForTest(0);
    return 0;
  } catch (const std::exception& error) {
    hwy::SetSupportedTargetsForTest(0);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
