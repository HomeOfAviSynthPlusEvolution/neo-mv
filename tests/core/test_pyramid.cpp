#include "core/super/geometry.hpp"
#include "core/super/pyramid_reduction.hpp"

#include <array>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("pyramid assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)

template <class F>
void rejects(F call) {
  bool rejected = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    rejected = true;
  } catch (const std::overflow_error&) {
    rejected = true;
  }
  CHECK(rejected);
}

void geometry() {
  neo_mv::SuperGeometryParams p{18, 10, 8, 8, 4, 4, 2, 2};
  auto g = neo_mv::make_super_geometry(p);
  CHECK(g.blocks_x == 4 && g.blocks_y == 2 && g.plane_count == 1);
  CHECK(g.planes[0].levels.size() == 1);
  const auto& level = g.planes[0].levels[0];
  CHECK(level.width == 20 && level.height == 12 && level.padded_width == 24 && level.padded_height == 16);
  p = {64, 64, 8, 8, 0, 0, 16, 16};
  g = neo_mv::make_super_geometry(p);
  CHECK(g.planes[0].levels.size() == 3);
  CHECK(g.planes[0].levels[0].width == 64 && g.planes[0].levels[1].width == 32 && g.planes[0].levels[2].width == 16);
  p = {72, 72, 8, 8, 0, 0, 3, 3, 2, 2, 4, true};
  g = neo_mv::make_super_geometry(p);
  CHECK(g.plane_count == 3 && g.planes[1].pad_x == 1);
  CHECK(g.planes[0].levels[2].width == 18 && g.planes[1].levels[2].width == 8);
  CHECK(g.planes[2].levels[1].width == 18 && g.planes[1].levels[0].phase_count == 16);
  CHECK(g.planes[1].levels[1].phase_count == 1);
  p.one_level = true;
  CHECK(neo_mv::make_super_geometry(p).planes[0].levels.size() == 1);
  p.pad_x = p.pad_y = 1;
  g = neo_mv::make_super_geometry(p);
  CHECK(g.planes[1].pad_x == 0 && g.planes[1].levels[0].padded_width == 36);
  // All supported chroma layouts, including 440, retain their axis ratios.
  for (int rx : {1, 2})
    for (int ry : {1, 2}) {
      p = {64, 64, 8, 8, 0, 0, 4, 4, rx, ry, 1, true};
      g = neo_mv::make_super_geometry(p);
      CHECK(g.planes[1].actual_width == 64 / rx && g.planes[1].actual_height == 64 / ry);
    }
  const auto invalid = [&](auto mutate) {
    auto copy = p;
    mutate(copy);
    rejects([&] { neo_mv::make_super_geometry(copy); });
  };
  invalid([](auto& v) { v.width = 7; });
  invalid([](auto& v) { v.width = 63; });
  invalid([](auto& v) { v.block_width = 12; });
  invalid([](auto& v) { v.overlap_x = 5; });
  invalid([](auto& v) { v.overlap_x = 1; });
  invalid([](auto& v) { v.pad_x = 0; });
  invalid([](auto& v) { v.pad_x = INT32_MAX; });
  invalid([](auto& v) { v.ratio_x = 3; });
  invalid([](auto& v) { v.chroma = false; });
  invalid([](auto& v) { v.pel = 3; });
  p = {INT32_MAX, 8, 8, 8, 0, 0, 1, 1};
  rejects([&] { neo_mv::make_super_geometry(p); });
}

template <class T>
void impulse_and_gaps() {
  // Read from an offset origin. Non-finite float padding verifies it is not read.
  const T outside = [] {
    if constexpr (std::is_same_v<T, float>)
      return std::numeric_limits<float>::quiet_NaN();
    else
      return T(99);
  }();
  std::vector<T> source(35 * 34, outside), output(19 * 16, T(77)), scratch(35 * 16, T(88));
  for (int y = 0; y < 32; ++y)
    for (int x = 0; x < 32; ++x)
      source[(y + 1) * 35 + x + 1] = T(0);
  source[5 * 35 + 5] = T(10);
  auto src = neo_mv::checked_plane<const T>(source.data(), 34, 34, 35 * sizeof(T), source.size() * sizeof(T));
  auto dst = neo_mv::checked_plane(output.data() + 1, 16, 16, 19 * sizeof(T), (output.size() - 1) * sizeof(T));
  auto tmp = neo_mv::checked_plane(scratch.data() + 1, 32, 16, 35 * sizeof(T), (scratch.size() - 1) * sizeof(T));
  for (int filter = 0; filter <= 2; ++filter) {
    neo_mv::reduce_pyramid(src, 1, 1, dst, filter, tmp);
    const T expected = [](int f) {
      if constexpr (std::is_same_v<T, float>)
        return f == 0 ? T(2.5) : f == 1 ? T(1.40625) : T(0.9765625);
      else
        return f == 0 ? T(3) : f == 1 ? T(2) : T(1);
    }(filter);
    CHECK(dst.row(2)[2] == expected);
    CHECK(dst.row(0)[0] == T(0) && dst.row(15)[15] == T(0));
    for (int y = 0; y < 16; ++y) {
      CHECK(output[y * 19] == T(77) && output[y * 19 + 17] == T(77) && output[y * 19 + 18] == T(77));
      CHECK(scratch[y * 35] == T(88) && scratch[y * 35 + 33] == T(88) && scratch[y * 35 + 34] == T(88));
    }
    CHECK(source[5 * 35 + 5] == T(10));
  }
  rejects([&] { neo_mv::reduce_pyramid(src, -1, 1, dst, 0); });
  rejects([&] { neo_mv::reduce_pyramid(src, 3, 1, dst, 0); });
  rejects([&] { neo_mv::reduce_pyramid(src, 1, 1, dst, 3); });
  rejects([&] { neo_mv::reduce_pyramid(src, 1, 1, dst, 1); });
  rejects([&] { neo_mv::reduce_pyramid(src, 1, 1, dst, 1, dst); });
  auto overlap = neo_mv::checked_plane(source.data(), 16, 16, 35 * sizeof(T), source.size() * sizeof(T));
  rejects([&] { neo_mv::reduce_pyramid(src, 1, 1, overlap, 0); });
  auto aliased_scratch = neo_mv::checked_plane(source.data(), 32, 16, 35 * sizeof(T), source.size() * sizeof(T));
  rejects([&] { neo_mv::reduce_pyramid(src, 1, 1, dst, 1, aliased_scratch); });
}

template <class T>
void constant_and_small() {
  const T value = [] {
    if constexpr (std::is_same_v<T, float>)
      return T(-0.125);
    else
      return std::numeric_limits<T>::max();
  }();
  for (int width : {1, 2, 3, 5})
    for (int height : {1, 2, 3, 5}) {
      const auto area = std::size_t(width) * height;
      std::vector<T> source(4 * area, value), output(area), scratch(2 * area);
      auto src = neo_mv::checked_plane<const T>(source.data(), 2 * width, 2 * height, 2 * width * sizeof(T),
                                                source.size() * sizeof(T));
      auto dst = neo_mv::checked_plane(output.data(), width, height, width * sizeof(T), output.size() * sizeof(T));
      auto tmp =
          neo_mv::checked_plane(scratch.data(), 2 * width, height, 2 * width * sizeof(T), scratch.size() * sizeof(T));
      for (int filter = 0; filter <= 2; ++filter) {
        neo_mv::reduce_pyramid(src, 0, 0, dst, filter, tmp);
        for (T pixel : output)
          CHECK(pixel == value);
      }
    }
}

void rounding_and_nonfinite() {
  std::array<std::uint8_t, 4> input{0, 0, 0, 1};
  std::array<std::uint8_t, 2> scratch{};
  std::uint8_t output = 99;
  auto src = neo_mv::checked_plane<const std::uint8_t>(input.data(), 2, 2, 2, 4);
  auto dst = neo_mv::checked_plane(&output, 1, 1, 1, 1);
  auto tmp = neo_mv::checked_plane(scratch.data(), 2, 1, 2, 2);
  neo_mv::reduce_pyramid(src, 0, 0, dst, 0);
  CHECK(output == 0);
  for (int filter : {1, 2}) {
    neo_mv::reduce_pyramid(src, 0, 0, dst, filter, tmp);
    CHECK(output == 1);
  }
  std::array<float, 4> huge{};
  huge.fill(std::numeric_limits<float>::max());
  float result = 7;
  auto fsrc = neo_mv::checked_plane<const float>(huge.data(), 2, 2, 2 * sizeof(float), sizeof(huge));
  auto fdst = neo_mv::checked_plane(&result, 1, 1, sizeof(float), sizeof(float));
  rejects([&] { neo_mv::reduce_pyramid(fsrc, 0, 0, fdst, 0); });
  CHECK(result == 7);
  huge[0] = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { neo_mv::reduce_pyramid(fsrc, 0, 0, fdst, 0); });
}
} // namespace

int main() {
  try {
    geometry();
    impulse_and_gaps<std::uint8_t>();
    impulse_and_gaps<std::uint16_t>();
    impulse_and_gaps<float>();
    constant_and_small<std::uint8_t>();
    constant_and_small<std::uint16_t>();
    constant_and_small<float>();
    rounding_and_nonfinite();
    std::cout << "Pyramid geometry and scalar reduction checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
