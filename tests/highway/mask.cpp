#include "highway/grid_resampling.hpp"
#include "highway/mask_scores.hpp"
#include "highway/rows.hpp"
#include "hwy/targets.h"
#include <iostream>
#include <cstring>
#include <random>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif
#include <new>
namespace {
void check(bool condition) {
  if (!condition)
    throw std::runtime_error("phase3 differential mismatch");
}
template <class T>
struct Buffer {
  int w, h, stride;
  std::vector<T> data;
  Buffer(int width, int height) : w(width), h(height), stride(width + 3), data(std::size_t(stride) * h + 1, T(19)) {}
  span2d::Plane<T> view() {
    return neo_mv::checked_plane(data.data() + 1, w, h, std::ptrdiff_t(stride) * sizeof(T),
                                 (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
};
template <class T>
void same(const std::vector<T>& a, const std::vector<T>& b) {
  check(a.size() == b.size());
  check(std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}
template <class T>
void grids(int bits) {
  std::mt19937 rng(5367);
  for (int nx : {1, 2, 9})
    for (int ny : {1, 3})
      for (int width : {1, 7, 16, 33, 129}) {
        const int bx = (width + nx - 1) / nx + 3, ox = 1, by = 5, oy = 2, height = ny * (by - oy) + oy - 1;
        neo_mv::GridResamplingGeometry g{nx, ny, bx, by, ox, oy, width, height};
        Buffer<T> input(nx, ny), a(width, height), b(width, height);
        for (int y = 0; y < ny; ++y)
          for (int x = 0; x < nx; ++x) {
            if constexpr (std::is_same_v<T, float>)
              input.view().row(y)[x] = float(int(rng() % 10001) - 5000) / 731.17f;
            else if constexpr (std::is_same_v<T, std::int16_t>)
              input.view().row(y)[x] = std::int16_t(int(rng() % 65536) - 32768);
            else
              input.view().row(y)[x] = T(rng() & ((1u << bits) - 1));
          }
        neo_mv::GridResamplingPlan(g).resize(input.read(), a.view(), bits);
        neo_mv::simd::GridResamplingPlan(g).resize(input.read(), b.view(), bits);
        same(a.data, b.data);
      }
  // Wide covered geometry and coefficient arithmetic, with bounded output.
  for (int side : {32765, 32767, 32769, INT32_MAX}) {
    const int width = side == INT32_MAX ? 17 : std::min(32767, 2 * (side / 2 + 1));
    neo_mv::GridResamplingGeometry g{2, 2, side / 2 + 1, side / 2 + 1, 0, 0, width, 3};
    Buffer<T> input(2, 2), a(width, 3), b(width, 3);
    input.view().row(0)[0] = T(0);
    input.view().row(0)[1] = T(1);
    input.view().row(1)[0] = T(17);
    input.view().row(1)[1] = T(255);
    neo_mv::GridResamplingPlan(g).resize(input.read(), a.view(), bits);
    neo_mv::simd::GridResamplingPlan(g).resize(input.read(), b.view(), bits);
    same(a.data, b.data);
  }
}

template <class T>
void integer_extremes(int bits) {
  const T low = std::is_signed_v<T> ? std::numeric_limits<T>::min() : T(0);
  const T high = std::is_signed_v<T> ? std::numeric_limits<T>::max() : T((1u << bits) - 1);
  // Widths straddle SIMD lane boundaries; the short overlapping vertical axis
  // also exercises vertical-first, with real work in both passes.
  for (int width : {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129})
    for (int height : {2, 3, 6}) {
      const int bx = (width + 1) / 2;
      const neo_mv::GridResamplingGeometry g{2, 2, bx, height == 6 ? 3 : 2, 0, height == 6 ? 0 : 1, width, height};
      Buffer<T> input(2, 2), a(width, height), b(width, height);
      for (int pattern = 0; pattern < 3; ++pattern) {
        input.view().row(0)[0] = pattern == 0 ? high : low;
        input.view().row(0)[1] = pattern == 1 ? low : high;
        input.view().row(1)[0] = high;
        input.view().row(1)[1] = pattern == 2 ? low : high;
        neo_mv::GridResamplingPlan(g).resize(input.read(), a.view(), bits);
        neo_mv::simd::GridResamplingPlan(g).resize(input.read(), b.view(), bits);
        same(a.data, b.data);
        if (pattern == 0)
          for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
              check(b.view().row(y)[x] == high);
      }
    }
}
void special_float() {
  Buffer<float> in(2, 2), a(33, 5), b(33, 5);
  const neo_mv::GridResamplingGeometry g{2, 2, 17, 3, 0, 0, 33, 5};
  for (float v : {-0.0f, 0.0f, std::numeric_limits<float>::denorm_min(), -std::numeric_limits<float>::denorm_min(),
                  std::numeric_limits<float>::max()}) {
    for (int y = 0; y < 2; ++y)
      for (int x = 0; x < 2; ++x)
        in.view().row(y)[x] = v;
    neo_mv::GridResamplingPlan(g).resize(in.read(), a.view(), 32);
    neo_mv::simd::GridResamplingPlan(g).resize(in.read(), b.view(), 32);
    same(a.data, b.data);
  }
  in.view().row(1)[1] = std::numeric_limits<float>::infinity();
  const auto before = b.data;
  bool threw = false;
  try {
    neo_mv::simd::GridResamplingPlan(g).resize(in.read(), b.view(), 32);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  check(threw);
  same(before, b.data);
}
neo_mv::AnalysisMetadata metadata(int nx, int ny, int bits, int pel, int delta) {
  neo_mv::AnalysisMetadata m{};
  m.width = m.real_width = nx * 8;
  m.height = m.real_height = ny * 8;
  m.pad_x = m.pad_y = 1024;
  m.pel = pel;
  m.levels = m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = 8;
  m.blocks_x = nx;
  m.blocks_y = ny;
  m.delta = delta;
  m.bits = bits;
  return m;
}
template <class T>
void scores(int bits) {
  for (int pel : {1, 2, 4})
    for (int delta : {-1, 1})
      for (float gamma : {0.0f, 1.0f, 2.0f, 0.75f}) {
        const auto m = metadata(65, 5, bits, pel, delta);
        neo_mv::MotionGrid grid{65, 5, {}};
        for (int y = 0; y < 5; ++y)
          for (int x = 0; x < 65; ++x)
            grid.values.push_back({{x % 2 ? -256 : 256, y % 2 ? -64 : 64}, std::int64_t(x + 73 * y) * 1000});
        for (int t : {0, 128, 256}) {
          same(neo_mv::VectorLengthMaskPlan<T>(m, 0.001f, gamma, t).generate(grid),
               neo_mv::simd::VectorLengthMaskPlan<T>(m, 0.001f, gamma, t).generate(grid));
          same(neo_mv::SADMaskPlan<T>(m, 0.002f, gamma, t).generate(grid),
               neo_mv::simd::SADMaskPlan<T>(m, 0.002f, gamma, t).generate(grid));
          same(neo_mv::OcclusionMaskPlan<T>(m, 0.0001f, gamma, t).generate(grid),
               neo_mv::simd::OcclusionMaskPlan<T>(m, 0.0001f, gamma, t).generate(grid));
        }
      }
}
void zero_and_subnormal() {
  const auto m = metadata(65, 2, 32, 1, 1);
  neo_mv::MotionGrid grid{65, 2, {}};
  for (int i = 0; i < 130; ++i)
    grid.values.push_back({{i % 2 ? -256 : 256, 0}, 100});
  same(neo_mv::SADMaskPlan<float>(m, -0.0f, 1, 256).generate(grid),
       neo_mv::simd::SADMaskPlan<float>(m, -0.0f, 1, 256).generate(grid));
  same(neo_mv::OcclusionMaskPlan<float>(m, -0.0f, 1, 256).generate(grid),
       neo_mv::simd::OcclusionMaskPlan<float>(m, -0.0f, 1, 256).generate(grid));
  Buffer<float> in(2, 1), a(5, 1), b(5, 1);
  const auto tiny = std::numeric_limits<float>::denorm_min();
  const neo_mv::GridResamplingGeometry g{2, 1, 3, 1, 1, 0, 5, 1};
  for (auto values :
       {std::array<float, 2>{0, tiny}, std::array<float, 2>{tiny, 2 * tiny}, std::array<float, 2>{-0.0f, -tiny}}) {
    in.view().row(0)[0] = values[0];
    in.view().row(0)[1] = values[1];
    neo_mv::GridResamplingPlan(g).resize(in.read(), a.view(), 32);
    neo_mv::simd::GridResamplingPlan(g).resize(in.read(), b.view(), 32);
    same(a.data, b.data);
  }
}

void repeated_powers() {
  // More distinct values than cache slots, followed by exact repeats. Compare
  // the complete unquantized results to libm, including both vector tails.
  constexpr std::size_t count = 1025;
  std::vector<double> x(count), y(count), result(count);
  std::vector<float> sad(count), sad_result(count);
  for (std::size_t i = 0; i < count; ++i) {
    x[i] = double(i % 257);
    y[i] = double((i / 3) % 131);
    sad[i] = float(i % 257);
  }
  for (float exponent : {0.375f, 0.5f, 1.25f}) {
    neo_mv::simd::mask_rows::magnitude(x.data(), y.data(), count, 2, 0.125f, exponent, 255, result.data());
    neo_mv::simd::mask_rows::sad(sad.data(), count, 0.25f, exponent, 255, sad_result.data());
    for (std::size_t i = 0; i < count; ++i) {
      const double a = x[i] / 2, b = y[i] / 2;
      const double expected = 255 * neo_mv::mask_detail::power((a * a + b * b) * 0.125, double(exponent));
      const float expected_sad = 255 * neo_mv::mask_detail::power(sad[i] * 0.25f, exponent);
      check(std::memcmp(&expected, &result[i], sizeof(double)) == 0);
      check(std::memcmp(&expected_sad, &sad_result[i], sizeof(float)) == 0);
    }
  }
}
template <class T>
struct EndRow {
  void* memory;
  T* data;
  std::size_t page;
  int width;
  explicit EndRow(int width) : width(width) {
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    page = info.dwPageSize;
#else
    const auto size = sysconf(_SC_PAGESIZE);
    if (size <= 0)
      throw std::runtime_error("cannot query page size");
    page = static_cast<std::size_t>(size);
#endif
    check(width > 0 && std::size_t(width) <= page / sizeof(T));
#if defined(_WIN32)
    memory = VirtualAlloc(nullptr, 2 * page, MEM_RESERVE, PAGE_NOACCESS);
    if (!memory)
      throw std::bad_alloc();
    if (!VirtualAlloc(memory, page, MEM_COMMIT, PAGE_READWRITE)) {
      VirtualFree(memory, 0, MEM_RELEASE);
      throw std::bad_alloc();
    }
#else
    memory = mmap(nullptr, 2 * page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (memory == MAP_FAILED)
      throw std::bad_alloc();
    if (mprotect(memory, page, PROT_READ | PROT_WRITE) != 0) {
      munmap(memory, 2 * page);
      throw std::bad_alloc();
    }
#endif
    data = reinterpret_cast<T*>(static_cast<unsigned char*>(memory) + page - width * sizeof(T));
    for (int x = 0; x < width; ++x)
      ::new (data + x) T(1);
  }
  ~EndRow() {
#if defined(_WIN32)
    VirtualFree(memory, 0, MEM_RELEASE);
#else
    munmap(memory, 2 * page);
#endif
  }
  EndRow(const EndRow&) = delete;
  EndRow& operator=(const EndRow&) = delete;
  span2d::Plane<T> view() { return neo_mv::checked_plane(data, width, 1, width * sizeof(T), width * sizeof(T)); }
  span2d::Plane<const T> read() { return view(); }
};
template <class T>
void guarded_rows(int bits) {
  for (int width : {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129}) {
    EndRow<T> src(3), out(width);
    if constexpr (std::is_same_v<T, float>) {
      src.data[0] = 0;
      src.data[1] = 1;
    } else {
      src.data[0] = std::is_signed_v<T> ? std::numeric_limits<T>::min() : T(0);
      src.data[1] = std::is_signed_v<T> ? std::numeric_limits<T>::max() : T((1u << bits) - 1);
    }
    src.data[2] = T(1);
    Buffer<T> expected(width, 1);
    const neo_mv::GridResamplingGeometry geometry{3, 1, (width + 2) / 3, 1, 0, 0, width, 1};
    neo_mv::GridResamplingPlan(geometry).resize(src.read(), expected.view(), bits);
    neo_mv::simd::GridResamplingPlan(geometry).resize(src.read(), out.view(), bits);
    check(std::memcmp(expected.view().row(0).data(), out.data, std::size_t(width) * sizeof(T)) == 0);
  }
}
} // namespace
int main() {
  try {
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << "Testing " << hwy::TargetName(target) << '\n';
      check(std::strcmp(neo_mv::simd::detail::target_name(), hwy::TargetName(target)) == 0);
      grids<std::uint8_t>(8);
      grids<std::uint16_t>(10);
      grids<std::uint16_t>(16);
      grids<std::int16_t>(16);
      grids<float>(32);
      integer_extremes<std::uint8_t>(8);
      integer_extremes<std::uint16_t>(10);
      integer_extremes<std::uint16_t>(16);
      integer_extremes<std::int16_t>(16);
      special_float();
      scores<std::uint8_t>(8);
      scores<std::uint16_t>(10);
      scores<std::uint16_t>(16);
      scores<float>(32);
      guarded_rows<std::uint8_t>(8);
      guarded_rows<std::uint16_t>(16);
      guarded_rows<std::int16_t>(16);
      guarded_rows<float>(32);
      zero_and_subnormal();
      repeated_powers();
    }
    hwy::SetSupportedTargetsForTest(0);
    std::cout << "Phase3 exact differentials passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
