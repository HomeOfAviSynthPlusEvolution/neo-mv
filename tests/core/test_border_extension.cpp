#include "core/super/border_extension.hpp"

#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition) {
  if (!condition)
    throw std::runtime_error("test assertion failed");
}

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
  require(rejected);
}

void admission() {
  alignas(64) std::array<std::uint16_t, 32> data{};
  auto view = neo_mv::checked_plane(data.data() + 1, 8, 2, 18, 34);
  require(view.stride() == 9 && view.row(1).data() == data.data() + 10);
  for (auto stride : {17, 0, -18, 14})
    rejects([&] { neo_mv::checked_plane(data.data(), 8, 2, stride, sizeof(data)); });
  rejects([&] { neo_mv::checked_plane(data.data(), 8, 2, 18, 33); });
  rejects([&] { neo_mv::checked_plane(data.data(), 0, 2, 18, sizeof(data)); });
  rejects([&] { neo_mv::checked_plane(data.data(), 8, -1, 18, sizeof(data)); });
  rejects([&] { neo_mv::checked_plane<std::uint16_t>(nullptr, 8, 2, 18, sizeof(data)); });
  rejects([&] {
    auto* misaligned = reinterpret_cast<std::uint16_t*>(reinterpret_cast<unsigned char*>(data.data()) + 1);
    neo_mv::checked_plane(misaligned, 8, 2, 18, sizeof(data) - 1);
  });
  rejects(
      [&] { neo_mv::checked_plane(data.data(), 1, 1, static_cast<std::ptrdiff_t>(INT32_MAX) * 2 + 2, sizeof(data)); });
  rejects([&] {
    neo_mv::checked_plane(data.data(), INT32_MAX, INT32_MAX, static_cast<std::ptrdiff_t>(INT32_MAX) * 2, sizeof(data));
  });
}

template <class T>
void borders() {
  // Both views have row gaps and non-SIMD-aligned starts. Sentinels must survive.
  alignas(64) std::array<T, 12> source{};
  alignas(64) std::array<T, 40> output{};
  source.fill(T(99));
  output.fill(T(77));
  source[1] = T(10);
  source[2] = T(20);
  source[3] = T(30);
  source[6] = T(40);
  source[7] = T(50);
  source[8] = T(60);
  const auto before = source;
  auto src = neo_mv::checked_plane<const T>(source.data() + 1, 3, 2, 5 * sizeof(T), 8 * sizeof(T));
  auto dst = neo_mv::checked_plane(output.data() + 1, 6, 5, 8 * sizeof(T), 38 * sizeof(T));
  neo_mv::extend_border(src, dst, 4, 3, 1, 1);
  const int expected[5][6] = {{10, 10, 20, 30, 30, 30},
                              {10, 10, 20, 30, 30, 30},
                              {40, 40, 50, 60, 60, 60},
                              {40, 40, 50, 60, 60, 60},
                              {40, 40, 50, 60, 60, 60}};
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 6; ++x)
      require(dst.row(y)[x] == T(expected[y][x]));
    require(output[y * 8] == T(77) && output[y * 8 + 7] == T(77));
  }
  require(source == before);
  rejects([&] { neo_mv::extend_border(src, dst, 2, 3, 1, 1); });
  rejects([&] { neo_mv::extend_border(src, dst, 4, 3, -1, 1); });
  rejects([&] { neo_mv::extend_border(src, dst, INT32_MAX, 3, 1, 1); });
  rejects([&] { neo_mv::extend_border(src, dst, 5, 3, 1, 1); });
  auto alias = neo_mv::checked_plane(source.data() + 1, 3, 2, 5 * sizeof(T), 8 * sizeof(T));
  rejects([&] { neo_mv::extend_border(src, alias, 3, 2, 0, 0); });
  require(source == before);

  // Row-gap sharing is legal when active sample regions do not overlap.
  std::array<T, 8> interleaved{T(7), T(0), T(8), T(0)};
  auto a = neo_mv::checked_plane<const T>(interleaved.data(), 1, 2, 2 * sizeof(T), sizeof(interleaved));
  auto b = neo_mv::checked_plane(interleaved.data() + 1, 1, 2, 2 * sizeof(T), 7 * sizeof(T));
  neo_mv::extend_border(a, b, 1, 2, 0, 0);
  require(interleaved[1] == T(7) && interleaved[3] == T(8));

  T one = T(7);
  std::array<T, 12> expanded{};
  neo_mv::extend_border(neo_mv::checked_plane<const T>(&one, 1, 1, sizeof(T), sizeof(T)),
                        neo_mv::checked_plane(expanded.data(), 4, 3, 4 * sizeof(T), sizeof(expanded)), 2, 1, 1, 1);
  for (auto value : expanded)
    require(value == T(7));
}

void floating_copy() {
  const float negative_zero = -0.0f;
  float output = 1.0f;
  auto dst = neo_mv::checked_plane(&output, 1, 1, sizeof(float), sizeof(float));
  neo_mv::extend_border(neo_mv::checked_plane(&negative_zero, 1, 1, sizeof(float), sizeof(float)), dst, 1, 1, 0, 0);
  require(output == 0.0f && std::signbit(output));
  for (const float bad : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    rejects([&] {
      neo_mv::extend_border(neo_mv::checked_plane(&bad, 1, 1, sizeof(float), sizeof(float)), dst, 1, 1, 0, 0);
    });
    require(std::signbit(output));
  }
}
} // namespace

int main() {
  try {
    admission();
    borders<std::uint8_t>();
    borders<std::uint16_t>();
    borders<float>();
    floating_copy();
    std::cout << "View admission and scalar border extension checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
