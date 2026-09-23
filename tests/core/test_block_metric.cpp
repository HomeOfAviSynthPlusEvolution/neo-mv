#include "core/motion/block_metric.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("block metric assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class F>
void rejects(F call) {
  bool rejected = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  CHECK(rejected);
}

template <class T>
struct Block {
  int width, height, stride;
  std::vector<T> data;
  Block(int w, int h, T value) : width(w), height(h), stride(w + 3), data(stride * h, T(77)) {
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        at(x, y) = value;
  }
  T& at(int x, int y) { return data[y * stride + x + 1]; }
  span2d::Plane<const T> view() const {
    return neo_mv::checked_plane(data.data() + 1, width, height, stride * sizeof(T), (data.size() - 1) * sizeof(T));
  }
};
using neo_mv::BlockMetric;
using neo_mv::block_metric;

template <class T>
void integer_examples() {
  Block<T> s(8, 8, T(3)), r(8, 8, T(1));
  const auto before = s.data, other = r.data;
  CHECK(block_metric(s.view(), r.view(), BlockMetric::sad) == 128);
  CHECK(block_metric(r.view(), s.view(), BlockMetric::sad) == 128);
  CHECK(block_metric(s.view(), r.view(), BlockMetric::satd) == 64);
  CHECK(s.data == before && r.data == other);
  Block<T> one(4, 4, T(1)), zero(4, 4, T(0));
  CHECK(block_metric(one.view(), zero.view(), BlockMetric::sad) == 16);
  CHECK(block_metric(one.view(), zero.view(), BlockMetric::satd) == 8);
  one = zero;
  one.at(2, 1) = T(1);
  CHECK(block_metric(one.view(), zero.view(), BlockMetric::sad) == 1);
  CHECK(block_metric(one.view(), zero.view(), BlockMetric::satd) == 8);
  Block<T> maximum(64, 64, std::numeric_limits<T>::max()), large_zero(64, 64, T(0));
  CHECK(block_metric(maximum.view(), large_zero.view(), BlockMetric::sad) == 4096LL * std::numeric_limits<T>::max());
  CHECK(block_metric(maximum.view(), large_zero.view(), BlockMetric::satd) == 2048LL * std::numeric_limits<T>::max());
}

void matrix_oracle() {
  // Independent direct H E H^T evaluation, across multiple nonsymmetric cells.
  constexpr int H[4][4] = {{1, 1, 1, 1}, {1, -1, 1, -1}, {1, 1, -1, -1}, {1, -1, -1, 1}};
  Block<std::uint16_t> s(12, 8, 0), r(12, 8, 0);
  std::uint32_t state = 0x12345678U;
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 12; ++x) {
      state = state * 1664525U + 1013904223U;
      s.at(x, y) = static_cast<std::uint16_t>(state >> 16);
      state = state * 1664525U + 1013904223U;
      r.at(x, y) = static_cast<std::uint16_t>(state >> 16);
    }
  std::int64_t expected = 0;
  for (int y = 0; y < 8; y += 4)
    for (int x = 0; x < 12; x += 4) {
      std::int64_t cell = 0;
      for (int a = 0; a < 4; ++a)
        for (int b = 0; b < 4; ++b) {
          std::int64_t coefficient = 0;
          for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 4; ++i)
              coefficient += H[a][j] * (std::int64_t(s.at(x + i, y + j)) - r.at(x + i, y + j)) * H[b][i];
          cell += coefficient < 0 ? -coefficient : coefficient;
        }
      expected += cell / 2;
    }
  CHECK(block_metric(s.view(), r.view(), BlockMetric::satd) == expected);
}

void floats() {
  Block<float> s(4, 4, 1.0f / 65536.0f), r(4, 4, 0.0f);
  CHECK(block_metric(s.view(), r.view(), BlockMetric::sad) == 16);
  CHECK(block_metric(s.view(), r.view(), BlockMetric::satd) == 8);
  Block<float> two(8, 4, 1.0f / 1048576.0f), z(8, 4, 0.0f);
  CHECK(block_metric(two.view(), z.view(), BlockMetric::satd) == 1); // encoding each cell would produce zero
  Block<float> outside(4, 4, -2.0f), positive(4, 4, 3.0f);
  CHECK(block_metric(outside.view(), positive.view(), BlockMetric::sad) == 5242800);
  CHECK(block_metric(outside.view(), positive.view(), BlockMetric::satd) == 2621400);
  CHECK(neo_mv::encode_float_error(-0.0f) == 0);
  CHECK(neo_mv::encode_float_error(0.5f) == 32768);
  CHECK(neo_mv::encode_float_error(65536.0f) == 4294901760LL);
  CHECK(neo_mv::encode_float_error(65537.0f) == 4294967295LL);
  CHECK(neo_mv::encode_float_error(std::nextafter(65537.0f, 0.0f)) == 4294966784LL);
  // Row-major accumulation loses each small term after the leading value.
  Block<float> order(4, 4, 0x1p-24f);
  order.at(0, 0) = 2.0f;
  CHECK(block_metric(order.view(), r.view(), BlockMetric::sad) == 131070);
  s.at(3, 3) = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { block_metric(s.view(), r.view(), BlockMetric::sad); });
  rejects([&] { block_metric(s.view(), r.view(), BlockMetric::satd); });
  Block<float> huge(4, 4, std::numeric_limits<float>::max());
  rejects([&] { block_metric(huge.view(), outside.view(), BlockMetric::sad); });
  rejects([&] { block_metric(huge.view(), r.view(), BlockMetric::satd); });
  rejects([&] { neo_mv::encode_float_error(std::numeric_limits<float>::max()); });
  // Row gaps are never sampled, including their nonfinite values.
  Block<float> gaps(4, 4, 0.0f);
  for (int y = 0; y < 4; ++y)
    gaps.data[std::size_t(y) * gaps.stride] = std::numeric_limits<float>::infinity();
  CHECK(block_metric(gaps.view(), r.view(), BlockMetric::satd) == 0);
}

void invalid_rectangles() {
  Block<std::uint8_t> s(16, 2, 1), r(16, 2, 0), other(4, 4, 0);
  CHECK(block_metric(s.view(), r.view(), BlockMetric::sad) == 32);
  rejects([&] { block_metric(s.view(), r.view(), BlockMetric::satd); });
  rejects([&] { block_metric(s.view(), other.view(), BlockMetric::sad); });
  rejects([&] { block_metric(s.view(), r.view(), static_cast<BlockMetric>(99)); });
  rejects([&] { block_metric(s.view(), span2d::Plane<const std::uint8_t>{}, BlockMetric::sad); });
}
} // namespace

int main() {
  try {
    integer_examples<std::uint8_t>();
    integer_examples<std::uint16_t>();
    matrix_oracle();
    floats();
    invalid_rectangles();
    std::cout << "Scalar block metric checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
