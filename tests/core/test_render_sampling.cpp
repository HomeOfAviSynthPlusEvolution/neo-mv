#include "core/render/block_sampling.hpp"
#include "core/motion/composition.hpp"
#include "core/motion/super_input.hpp"

#include <iostream>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("render sampling assertion failed at line " + std::to_string(line));
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
RenderPhaseGeometry geometry(int pel, int rx, int ry, int pad, bool restricted = true) {
  RenderPhaseGeometry g{pel, rx, ry, pad / rx, pad / ry, {}};
  for (int ay = 0; ay < pel; ++ay)
    for (int ax = 0; ax < pel; ++ax)
      g.phases[ay * pel + ax] = {16 / rx + 2 * g.pad_x - (restricted && pel == 4 && ax == 3),
                                 16 / ry + 2 * g.pad_y - (restricted && pel == 4 && ay == 3)};
  return g;
}
bool safe(const RenderPhaseGeometry& g, BlockRegion b, int vx, int vy, int t) {
  // Literal independent formula, bounded small inputs, no production helpers.
  const int dx = vx * t / 256, dy = vy * t / 256;
  const int ax = int(std::floor(double(g.pel * b.x + dx) / g.ratio_x));
  const int ay = int(std::floor(double(g.pel * b.y + dy) / g.ratio_y));
  const int qx = int(std::floor(double(ax) / g.pel)), qy = int(std::floor(double(ay) / g.pel));
  const auto e = g.phases[(ay - g.pel * qy) * g.pel + ax - g.pel * qx];
  return g.pad_x + qx >= 0 && g.pad_y + qy >= 0 && g.pad_x + qx + b.width / g.ratio_x <= e.width &&
         g.pad_y + qy + b.height / g.ratio_y <= e.height;
}
void domain_proof() {
  for (int pel : {1, 2, 4})
    for (int rx : {1, 2})
      for (int ry : {1, 2})
        for (int pad : {0, 3, 4})
          for (int t : {0, 1, 63, 128, 255, 256})
            for (bool restricted : {false, true})
              for (int trial = 0; trial < 40; ++trial) {
                const auto g = geometry(pel, rx, ry, pad, restricted);
                const BlockRegion b{2 * (trial % 5), 2 * ((trial / 5) % 5), 4, 4};
                const int left = trial * 7 % 47 - 19, top = trial * 11 % 43 - 17;
                const int right = left + trial % 9 + 1, bottom = top + trial % 7 + 1;
                bool expected = true;
                for (int y = top; y < bottom; ++y)
                  for (int x = left; x < right; ++x)
                    expected = safe(g, b, x, y, t) && expected;
                CHECK(!rejects([&] { validate_render_domain(g, b, {left, top, right, bottom}, t); }) == expected);
              }
  for (int edge : {-257, -256, -255, 255, 256, 257}) {
    const auto g = geometry(2, 2, 2, 0);
    const BlockRegion b{0, 0, 4, 4};
    bool expected = true;
    for (int v = edge - 2; v <= edge + 2; ++v)
      expected = safe(g, b, v, 0, 1) && expected;
    CHECK(!rejects([&] { validate_render_domain(g, b, {edge - 2, 0, edge + 3, 1}, 1); }) == expected);
  }
  auto wide = geometry(1, 1, 1, 0);
  wide.phases[0] = {INT32_MAX, INT32_MAX};
  validate_render_domain(wide, {0, 0, 1, 1}, {0, 0, INT32_MAX, INT32_MAX});
  CHECK(rejects<std::overflow_error>([&] { validate_render_domain(wide, {0, 0, 1, 1}, {0, 0, INT64_MAX, 1}); }));
  CHECK(rejects([&] { validate_render_domain(wide, {0, 0, 1, 1}, {0, 0, 0, 1}); }));
  CHECK(rejects([&] { validate_render_domain(wide, {0, 0, 1, 1}, {0, 0, 1, 1}, 257); }));
}
void spec_examples() {
  for (int pel : {1, 2, 4})
    for (int ratio : {1, 2}) {
      const auto gg = geometry(pel, ratio, ratio, 4);
      const BlockRegion block{4, 4, 4, 4};
      for (int dy = -20; dy <= 20; ++dy)
        for (int dx = -20; dx <= 20; ++dx) {
          const RenderDisplacement d{dx, dy};
          const bool accepted = !rejects([&] { render_footprint(gg, block, d); });
          CHECK(!rejects([&] { render_footprint_admitted(gg, block, d); }) == accepted);
          if (accepted) {
            const auto a = render_footprint(gg, block, d), b = render_footprint_admitted(gg, block, d);
            CHECK(a.phase == b.phase && a.x == b.x && a.y == b.y && a.width == b.width && a.height == b.height);
          }
        }
    }
  auto g = geometry(2, 2, 2, 4);
  auto f = render_footprint(g, {0, 0, 8, 8}, {-1, 0});
  CHECK(f.phase == 1 && f.x == 1 && f.y == 2 && f.width == 4 && f.height == 4);
  f = render_footprint(g, {8, 0, 8, 8}, {-3, 0});
  CHECK(f.phase == 0 && f.x == 5);
  for (int pad : {3, 4}) {
    SuperPlan<std::uint8_t> plan({16, 16, 8, 8, 0, 0, pad, pad, 2, 2, 2, true}, 8);
    const auto m = super_analysis_metadata(plan, 1);
    const auto sampling = super_sampling_geometry(plan)[0];
    for (int by = 0; by < 2; ++by)
      for (int bx = 0; bx < 2; ++bx)
        for (int plane = 0; plane < 3; ++plane) {
          const auto pg = render_phase_geometry(sampling, plane);
          const auto block = analysis_block(m, bx, by);
          const auto domain = analysis_domain(m, block);
          CHECK(rejects([&] { validate_render_domain(pg, block, domain); }) == (pad == 3 && plane != 0));
          validate_render_domain(pg, block, domain, 0);
          for (int shift : {-1, 0, 1})
            render_footprint(pg, block, {0, shift});
        }
  }
  auto luma = geometry(2, 1, 1, 4);
  CHECK(rejects([&] { render_footprint(luma, {0, 0, 8, 8}, {0, -9}); }));
  RenderPhaseGeometry quarter{4, 1, 1, 0, 0, {}};
  for (int a = 0; a < 16; ++a)
    quarter.phases[a] = {12 - (a % 4 == 3), 12 - (a / 4 == 3)};
  CHECK(rejects([&] { render_footprint(quarter, {0, 0, 4, 4}, {35, 0}); }));
  CHECK(render_footprint(quarter, {0, 0, 4, 4}, {31, 0}).x == 7);
  CHECK(rejects<std::overflow_error>([&] { render_footprint(luma, {2, 0, 4, 4}, {INT64_MAX, 0}); }));
  CHECK(rejects([&] { render_footprint(luma, {0, 0, 4, 4}, {INT64_MIN, 0}); }));
}
template <class T>
void storage(int bits) {
  const auto g = geometry(2, 2, 2, 4);
  std::array<std::vector<T>, 16> buffers;
  SubpixelPhases<T> phases{2};
  for (int a = 0; a < 4; ++a) {
    auto& data = buffers[a];
    data.assign(15 * 12, T(250));
    for (int y = 0; y < 12; ++y)
      for (int x = 0; x < 12; ++x)
        data[y * 15 + x] = T(a * 40 + y * 2 + x);
    phases.planes[a] = checked_plane<const T>(data.data(), 12, 12, 15 * sizeof(T), data.size() * sizeof(T));
  }
  std::vector<T> output(7 * 4, T(249));
  const auto out = checked_plane(output.data(), 4, 4, 7 * sizeof(T), output.size() * sizeof(T));
  sample_render_block(g, {0, 0, 8, 8}, {-1, 0}, phases, out, bits);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x)
      CHECK(output[y * 7 + x] == T(45 + y * 2 + x));
    for (int x = 4; x < 7; ++x)
      CHECK(output[y * 7 + x] == T(249));
  }
  const auto saved = output;
  CHECK(rejects([&] { sample_render_block(g, {0, 0, 8, 8}, {-99, 0}, phases, out, bits); }));
  CHECK(output == saved);
  auto missing = phases;
  missing.planes[3] = {}; // unused declared phase must still conform
  CHECK(rejects([&] { sample_render_block(g, {0, 0, 8, 8}, {0, 0}, missing, out, bits); }));
  const auto alias = checked_plane(buffers[3].data(), 4, 4, 15 * sizeof(T), buffers[3].size() * sizeof(T));
  CHECK(rejects([&] { sample_render_block(g, {0, 0, 8, 8}, {-1, 0}, phases, alias, bits); }));
  if constexpr (std::is_same_v<T, float>) {
    buffers[1][2 * 15 + 1] = -0.0f;
    sample_render_block(g, {0, 0, 8, 8}, {-1, 0}, phases, out, bits);
    CHECK(std::signbit(out.row(0)[0]));
    const auto unchanged = output;
    buffers[1][5 * 15 + 4] = std::numeric_limits<float>::infinity();
    CHECK(rejects([&] { sample_render_block(g, {0, 0, 8, 8}, {-1, 0}, phases, out, bits); }));
    CHECK(output == unchanged && std::signbit(out.row(0)[0]));
  } else if constexpr (sizeof(T) == 2) {
    buffers[1][5 * 15 + 4] = T(1024);
    CHECK(rejects([&] { sample_render_block(g, {0, 0, 8, 8}, {-1, 0}, phases, out, 10); }));
    CHECK(output == saved);
  }
}
} // namespace
int main() {
  try {
    domain_proof();
    spec_examples();
    storage<std::uint8_t>(8);
    storage<std::uint16_t>(10);
    storage<float>(32);
    std::cout << "Render sampling and whole-domain admission checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
