#include "highway/render.hpp"
#include "highway/render_rows.hpp"
#include <cstring>
#include <iostream>
#include <vector>

namespace {
void check(bool ok) {
  if (!ok)
    throw std::runtime_error("render SIMD assertion failed");
}
template <class T>
struct Buffer {
  int w, h, stride;
  std::vector<T> samples;
  Buffer(int width, int height, int gap = 3)
      : w(width), h(height), stride(width + gap), samples(std::size_t(stride) * h + 1, T(19)) {}
  span2d::Plane<T> view() {
    return neo_mv::checked_plane(samples.data() + 1, w, h, std::ptrdiff_t(stride) * sizeof(T),
                                 (samples.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
};
template <class T, class F>
void rejected_without_write(Buffer<T>& output, F f) {
  auto before = output.samples;
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  check(caught);
  check(std::memcmp(before.data(), output.samples.data(), before.size() * sizeof(T)) == 0);
}
template <class T>
void run(int bits) {
  // Natural alignment only, distinct row pitches, full vectors and short tails.
  for (int width : {1, 7, 16, 33, 128})
    for (int pel : {1, 2, 4})
      for (int ratio : {1, 2}) {
        constexpr int height = 5;
        neo_mv::RenderPhaseGeometry g{pel, ratio, ratio, 4, 4, {}};
        neo_mv::SubpixelPhases<T> phases{pel, {}};
        std::vector<Buffer<T>> storage;
        storage.reserve(16);
        for (int i = 0; i < pel * pel; ++i) {
          int w = width + 8 - (pel == 4 && i % pel == 3), h = height + 8 - (pel == 4 && i / pel == 3);
          storage.emplace_back(w, h, 5 + i % 3);
          g.phases[i] = {w, h};
          phases.planes[i] = storage.back().read();
          for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
              if constexpr (std::is_same_v<T, float>)
                storage.back().view().row(y)[x] = float(x - 2 * y + i) / 13.0f;
              else
                storage.back().view().row(y)[x] = T((x + 3 * y + 7 * i) % 256);
            }
          if constexpr (std::is_same_v<T, float>)
            storage.back().view().row(3)[3] = -0.0f;
        }
        neo_mv::BlockRegion block{0, 0, width * ratio, height * ratio};
        Buffer<T> expected(width, height, 7), actual(width, height, 7);
        for (auto d : {neo_mv::RenderDisplacement{0, 0}, neo_mv::RenderDisplacement{-1, -1},
                       neo_mv::RenderDisplacement{-3, 1}}) {
          neo_mv::sample_render_block(g, block, d, phases, expected.view(), bits);
          neo_mv::simd::sample_render_block(g, block, d, phases, actual.view(), bits);
          check(std::memcmp(expected.samples.data(), actual.samples.data(), expected.samples.size() * sizeof(T)) == 0);
        }
        rejected_without_write(
            actual, [&] { neo_mv::simd::sample_render_block(g, block, {-10000, 0}, phases, actual.view(), bits); });
        auto wrong = phases;
        wrong.pel = pel == 1 ? 2 : 1;
        rejected_without_write(
            actual, [&] { neo_mv::simd::sample_render_block(g, block, {0, 0}, wrong, actual.view(), bits); });
        if (pel > 1) {
          auto invalid = phases;
          invalid.planes[pel * pel - 1] = {};
          rejected_without_write(
              actual, [&] { neo_mv::simd::sample_render_block(g, block, {0, 0}, invalid, actual.view(), bits); });
        }
        if (pel == 4) {
          // Phase 3 has width+7 columns: start 7 fits, start 8 is one past its domain.
          neo_mv::simd::sample_render_block(g, block, {15 * ratio, 0}, phases, actual.view(), bits);
          rejected_without_write(actual, [&] {
            neo_mv::simd::sample_render_block(g, block, {19 * ratio, 0}, phases, actual.view(), bits);
          });
          rejected_without_write(actual, [&] {
            neo_mv::simd::sample_render_block(g, block, {0, 19 * ratio}, phases, actual.view(), bits);
          });
        }
        bool caught = false;
        try {
          neo_mv::simd::sample_render_block(g, block, {0, 0}, phases, storage[0].view().subplane(0, 0, width, height),
                                            bits);
        } catch (const std::invalid_argument&) {
          caught = true;
        }
        check(caught);

        if constexpr (std::is_same_v<T, float> || std::is_same_v<T, std::uint16_t>) {
          // Bad last consumed row must be rejected before any output write.
          auto& row = storage[0];
          const auto saved = row.view().row(4 + height - 1)[4 + width - 1];
          if constexpr (std::is_same_v<T, float>)
            row.view().row(4 + height - 1)[4 + width - 1] = std::numeric_limits<float>::quiet_NaN();
          else
            row.view().row(4 + height - 1)[4 + width - 1] = T(1 << bits);
          rejected_without_write(
              actual, [&] { neo_mv::simd::sample_render_block(g, block, {0, 0}, phases, actual.view(), bits); });
          row.view().row(4 + height - 1)[4 + width - 1] = saved;
          // Invalid values outside the selected footprint are not consumed.
          if constexpr (std::is_same_v<T, float>)
            row.view().row(0)[0] = std::numeric_limits<float>::infinity();
          else
            row.view().row(0)[0] = T(1 << bits);
          neo_mv::simd::sample_render_block(g, block, {0, 0}, phases, actual.view(), bits);
        }
      }
}
template <class T>
void fused_composition(int bits) {
  for (int width : {1, 4, 7, 8, 16, 17, 32, 33})
    for (bool overlap : {false, true}) {
      const int ox = overlap ? width / 2 : 0, oy = overlap ? 2 : 0;
      const int total_width = 2 * width - ox, total_height = 10 - oy;
      neo_mv::OverlapCompositionPlan plan(
          {width, 5, ox, oy, 2, 2, total_width - 1, total_height - 1, total_width, total_height});
      std::vector<Buffer<T>> storage;
      storage.reserve(4);
      std::vector<span2d::Plane<const T>> views;
      std::vector<neo_mv::simd::detail::SampledRenderBlock<T>> blocks;
      for (int i = 0; i < 4; ++i) {
        storage.emplace_back(width, 5, 3 + i);
        auto v = storage.back().view();
        for (int y = 0; y < 5; ++y)
          for (int x = 0; x < width; ++x) {
            if constexpr (std::is_same_v<T, float>)
              v.row(y)[x] = float(x * 17 - y * 31 + i * 19) / 13;
            else
              v.row(y)[x] = T((x * 17 + y * 31 + i * 19) % (1u << bits));
          }
        views.push_back(v);
        blocks.push_back({v.data(), v.stride(), plan.has_overlap() ? plan.coefficient_row(i % 2, i / 2, 0) : nullptr});
      }
      Buffer<T> expected(total_width - 1, total_height - 1), actual(total_width - 1, total_height - 1);
      neo_mv::compose_render_blocks(plan, views, expected.view(), bits);
      auto execute = [&] {
        neo_mv::simd::detail::compose_sampled(plan.geometry(), blocks.data(), actual.view(),
                                              neo_mv::subpixel_detail::sample_max<T>(bits));
      };
      execute();
      check(std::memcmp(expected.samples.data(), actual.samples.data(), expected.samples.size() * sizeof(T)) == 0);
      if constexpr (!std::is_same_v<T, std::uint8_t>) {
        // Bottom-right sample is cropped out of output, but must still be admitted.
        auto last = storage.back().view();
        if constexpr (std::is_same_v<T, float>)
          last.row(4)[width - 1] = std::numeric_limits<float>::quiet_NaN();
        else if (bits < 16)
          last.row(4)[width - 1] = T(1u << bits);
        if (bits != 16)
          rejected_without_write(actual, execute);
      }
    }
}
} // namespace
int main() {
  try {
    run<std::uint8_t>(8);
    run<std::uint16_t>(10);
    run<float>(32);
    fused_composition<std::uint8_t>(8);
    fused_composition<std::uint16_t>(10);
    fused_composition<std::uint16_t>(16);
    fused_composition<float>(32);
    std::cout << "Highway target: " << neo_mv::simd::detail::target_name() << '\n';
    std::cout << "Phase2 render sampling typical-path checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
