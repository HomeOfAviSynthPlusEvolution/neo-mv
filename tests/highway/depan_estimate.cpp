#include "highway/depan_estimate.hpp"
#include "highway/rows.hpp"
#include "core/depan/numeric.hpp"
#include "hwy/targets.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <random>
#include <string>
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
struct EndRow {
  void* memory;
  Complex* data;
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
    committed = ((count * sizeof(Complex) + page - 1) / page) * page;
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
    data = reinterpret_cast<Complex*>(static_cast<unsigned char*>(memory) + committed - count * sizeof(Complex));
    for (std::size_t i = 0; i < count; ++i)
      ::new (data + i) Complex(0);
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
      CHECK(std::memcmp(scalar.data(), vector.data(), scalar.size() * sizeof(float)) == 0);
    }
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
    }
    hwy::SetSupportedTargetsForTest(0);
    return 0;
  } catch (const std::exception& error) {
    hwy::SetSupportedTargetsForTest(0);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
