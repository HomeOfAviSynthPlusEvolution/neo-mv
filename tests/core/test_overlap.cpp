#include "core/render/overlap.hpp"

#include <iostream>
#include <string>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("overlap assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E = std::invalid_argument, class F>
bool rejects(F call) {
  try {
    call();
  } catch (const E&) {
    return true;
  }
  return false;
}

std::array<float, 3> permitted_axis(int b, int o, int n, int k, int i) {
  if (o == 0 || (i < o && k == 0) || (i >= b - o && k == n - 1) || (i >= o && i < b - o))
    return {1, 1, 1};
  const float pi = static_cast<float>(std::acos(-1.0L));
  const int z = i < o ? i - o : i - b + o;
  const float offset = static_cast<float>(double(z) + 0.5);
  const float product = pi * offset;
  const float angle = product / static_cast<float>(2 * o);
  // Higher precision reference followed by the explicit +/- one float-ULP
  // cosine tolerance. The allowed square is propagated, never guessed.
  const float c = static_cast<float>(std::cos(static_cast<long double>(angle)));
  std::array<float, 3> values{std::nextafter(c, -INFINITY), c, std::nextafter(c, INFINITY)};
  for (auto& v : values) {
    v = std::clamp(v, -1.0f, 1.0f);
    v = v * v;
  }
  return values;
}
void windows() {
  for (int b : {2, 3, 4, 6, 8, 12, 16, 24, 48, 128})
    for (int o : {0, 1, b / 2})
      for (int n : {1, 2, 3}) {
        const int cover = n * (b - o) + o;
        const OverlapCompositionPlan plan({b, b, o, o, n, n, cover, cover, cover, cover});
        for (int k = 0; k < n; ++k) {
          const auto axis = overlap_axis_window(b, o, n, k);
          for (int i = 0; i < b; ++i) {
            const auto allowed = permitted_axis(b, o, n, k, i);
            CHECK(std::find(allowed.begin(), allowed.end(), axis[i]) != allowed.end());
          }
        }
        for (int by = 0; by < n; ++by)
          for (int bx = 0; bx < n; ++bx)
            for (int y = 0; y < b; ++y)
              for (int x = 0; x < b; ++x) {
                bool allowed = false;
                for (float wy : permitted_axis(b, o, n, by, y))
                  for (float wx : permitted_axis(b, o, n, bx, x)) {
                    const float product = wy * wx;
                    const float scaled = product * 2048.0f;
                    const auto quantized = static_cast<int>(scaled + 0.5f);
                    allowed = allowed || plan.coefficient(bx, by, x, y) == quantized;
                  }
                CHECK(allowed);
              }
      }
  const OverlapCompositionPlan p({4, 4, 1, 1, 2, 2, 7, 7, 7, 7});
  CHECK(p.coefficient(0, 0, 3, 0) == 1024);
  CHECK(p.coefficient(0, 0, 3, 3) == 512);
  CHECK(p.coefficient(0, 0, 0, 0) == 2048);
  CHECK(rejects([&] { p.coefficient(2, 0, 0, 0); }));
  CHECK(rejects([] { overlap_axis_window(4, 3, 2, 0); }));
  CHECK(rejects([] { OverlapCompositionPlan p({8, 8, 4, 0, 3, 1, 18, 8, 24, 8}); }));
  CHECK(rejects([] { OverlapCompositionPlan p({8, 8, 0, 0, INT32_MAX, 1, 8, 8, INT32_MAX, 8}); }));
}

template <class T>
struct Blocks {
  std::vector<std::vector<T>> data;
  std::vector<span2d::Plane<const T>> views;
  Blocks(int count, int w, int h) : data(count) {
    for (int i = 0; i < count; ++i) {
      data[i].assign((w + 3) * h, T(241));
      views.push_back(checked_plane<const T>(data[i].data(), w, h, (w + 3) * sizeof(T), data[i].size() * sizeof(T)));
    }
  }
};
template <class T>
void composition(int bits) {
  const T maximum = std::is_same_v<T, float> ? T(1) : T((1 << bits) - 1);
  for (int ox : {0, 1, 3, 4})
    for (int oy : {0, 1, 3, 4}) {
      constexpr int b = 8, nx = 3, ny = 3;
      const int width = nx * (b - ox) + ox - 2, height = ny * (b - oy) + oy - 1;
      const OverlapCompositionPlan plan({b, b, ox, oy, nx, ny, width, height, 32, 32});
      Blocks<T> blocks(nx * ny, b, b);
      for (int k = 0; k < nx * ny; ++k)
        for (int y = 0; y < b; ++y)
          for (int x = 0; x < b; ++x) {
            if constexpr (std::is_same_v<T, float>)
              blocks.data[k][y * (b + 3) + x] = (float(k * 73 + y * 11 + x * 7) - 350.0f) / 13.0f;
            else
              blocks.data[k][y * (b + 3) + x] = T((k * 73 + y * 11 + x * 7) % (std::int64_t(maximum) + 1));
          }
      std::vector<T> output((width + 5) * height, T(240));
      auto view = checked_plane(output.data(), width, height, (width + 5) * sizeof(T), output.size() * sizeof(T));
      compose_render_blocks(plan, blocks.views, view, bits);
      // Independently enumerate the entire grid in row-major order.
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          std::int64_t integer = 0;
          float floating = 0;
          T copy = 0;
          for (int by = 0; by < ny; ++by)
            for (int bx = 0; bx < nx; ++bx) {
              const int lx = x - bx * (b - ox), ly = y - by * (b - oy);
              if (lx < 0 || ly < 0 || lx >= b || ly >= b)
                continue;
              const auto q = blocks.views[by * nx + bx].row(ly)[lx];
              copy = q;
              const auto w = plan.coefficient(bx, by, lx, ly);
              if constexpr (std::is_same_v<T, float>) {
                const float product = q * float(w);
                const float contribution = product / 64.0f;
                floating = floating + contribution;
              } else
                integer += std::int64_t(q) * w / 64;
            }
          if (ox == 0 && oy == 0)
            CHECK(view.row(y)[x] == copy);
          else if constexpr (std::is_same_v<T, float>)
            CHECK(view.row(y)[x] == floating / 32.0f);
          else
            CHECK(view.row(y)[x] == T(std::min<std::int64_t>((integer + 16) / 32, maximum)));
        }
        for (int x = width; x < width + 5; ++x)
          CHECK(output[y * (width + 5) + x] == T(240));
      }
    }
}

void examples_and_errors() {
  OverlapCompositionPlan plan({4, 4, 1, 0, 2, 1, 7, 4, 7, 4});
  Blocks<std::uint8_t> blocks(2, 4, 4);
  std::fill(blocks.data[0].begin(), blocks.data[0].end(), 10);
  std::fill(blocks.data[1].begin(), blocks.data[1].end(), 21);
  super_detail::PlaneBuffer<std::uint8_t> output(7, 4);
  compose_render_blocks(plan, blocks.views, output.view(), 8);
  CHECK(output.view().row(0)[3] == 16);
  Blocks<float> floats(2, 4, 4);
  std::fill(floats.data[0].begin(), floats.data[0].end(), 10);
  std::fill(floats.data[1].begin(), floats.data[1].end(), 21);
  super_detail::PlaneBuffer<float> out_float(7, 4);
  compose_render_blocks(plan, floats.views, out_float.view(), 32);
  CHECK(out_float.view().row(0)[3] == 15.5f);
  floats.data[1][0] = std::numeric_limits<float>::max();
  CHECK(rejects<std::overflow_error>([&] { compose_render_blocks(plan, floats.views, out_float.view(), 32); }));
  floats.data[1][0] = INFINITY;
  CHECK(rejects([&] { compose_render_blocks(plan, floats.views, out_float.view(), 32); }));
  const OverlapCompositionPlan cropped({8, 4, 0, 0, 3, 1, 18, 4, 32, 4});
  Blocks<std::uint8_t> crop_blocks(3, 8, 4);
  for (int k = 0; k < 3; ++k)
    std::fill(crop_blocks.data[k].begin(), crop_blocks.data[k].end(), std::uint8_t(10 + k));
  super_detail::PlaneBuffer<std::uint8_t> crop_output(18, 4);
  compose_render_blocks(cropped, crop_blocks.views, crop_output.view(), 8);
  for (int x = 0; x < 18; ++x)
    CHECK(crop_output.view().row(0)[x] == 10 + x / 8);
  Blocks<float> invalid_crop(3, 8, 4);
  invalid_crop.data[2][3 * 11 + 7] = std::numeric_limits<float>::quiet_NaN();
  super_detail::PlaneBuffer<float> unchanged(18, 4);
  CHECK(rejects([&] { compose_render_blocks(cropped, invalid_crop.views, unchanged.view(), 32); }));
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 18; ++x)
      CHECK(unchanged.view().row(y)[x] == 0);
  const OverlapCompositionPlan single({4, 4, 2, 2, 1, 1, 4, 4, 4, 4});
  Blocks<float> zero(1, 4, 4);
  std::fill(zero.data[0].begin(), zero.data[0].end(), -0.0f);
  super_detail::PlaneBuffer<float> z(4, 4);
  compose_render_blocks(single, zero.views, z.view(), 32);
  CHECK(!std::signbit(z.view().row(0)[0])); // accumulation starts with +0
  const OverlapCompositionPlan copy({4, 4, 0, 0, 1, 1, 4, 4, 4, 4});
  compose_render_blocks(copy, zero.views, z.view(), 32);
  CHECK(std::signbit(z.view().row(0)[0]));
  auto alias = checked_plane(zero.data[0].data(), 4, 4, 7 * sizeof(float), zero.data[0].size() * sizeof(float));
  CHECK(rejects([&] { compose_render_blocks(single, zero.views, alias, 32); }));
  zero.views.clear();
  CHECK(rejects([&] { compose_render_blocks(single, zero.views, z.view(), 32); }));
}
} // namespace
int main() {
  try {
    windows();
    composition<std::uint8_t>(8);
    composition<std::uint16_t>(10);
    composition<std::uint16_t>(16);
    composition<float>(32);
    examples_and_errors();
    std::cout << "Overlap windows and scalar block composition checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
