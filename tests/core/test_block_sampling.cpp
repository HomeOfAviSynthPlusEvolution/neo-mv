#include "core/motion/block_sampling.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("block sampling assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class F>
bool rejects(F call) {
  try {
    call();
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

neo_mv::SamplingGeometry geometry(int padding, int pel, bool chroma, bool restricted = true) {
  neo_mv::SamplingGeometry g;
  g.pel = pel;
  g.ratio_x = g.ratio_y = chroma ? 2 : 1;
  g.chroma = chroma;
  for (int k = 0; k < (chroma ? 3 : 1); ++k) {
    const int ratio = k == 0 ? 1 : 2;
    auto& p = g.planes[k];
    p.pad_x = p.pad_y = padding / ratio;
    p.current = {16 / ratio + 2 * p.pad_x, 16 / ratio + 2 * p.pad_y};
    for (int ay = 0; ay < pel; ++ay)
      for (int ax = 0; ax < pel; ++ax)
        p.reference[ay * pel + ax] = {p.current.width - (restricted && pel == 4 && ax == 3),
                                      p.current.height - (restricted && pel == 4 && ay == 3)};
  }
  return g;
}

template <class T>
struct Frames {
  std::array<std::vector<T>, 3> source;
  std::array<std::array<std::vector<T>, 16>, 3> reference;
  neo_mv::SamplingFrames<T> views;
  Frames(const neo_mv::SamplingGeometry& g, T s, T r) {
    const auto fill = [](std::vector<T>& buffer, neo_mv::PhaseExtent e, T value) {
      buffer.assign((e.width + 1) * e.height, value);
      return neo_mv::checked_plane<const T>(buffer.data(), e.width, e.height, (e.width + 1) * sizeof(T),
                                            buffer.size() * sizeof(T));
    };
    for (int k = 0; k < (g.chroma ? 3 : 1); ++k) {
      views.current[k] = fill(source[k], g.planes[k].current, s);
      for (int a = 0; a < g.pel * g.pel; ++a)
        views.reference[k][a] = fill(reference[k][a], g.planes[k].reference[a], r);
    }
  }
};

void specification_geometry() {
  using namespace neo_mv;
  auto bad = geometry(3, 2, true);
  CHECK(rejects([&] { validate_sampling_domain(bad, {0, 0, 8, 8}, {-6, -6, 22, 22}); }));
  // Luma is safe; only enabled chroma makes this geometry unsupported.
  bad.chroma = false;
  validate_sampling_domain(bad, {0, 0, 8, 8}, {-6, -6, 22, 22});
  auto good = geometry(4, 2, true);
  for (int y : {0, 8})
    for (int x : {0, 8})
      validate_sampling_domain(good, {x, y, 8, 8}, {-2 * (x + 4), -2 * (y + 4), 2 * (12 - x), 2 * (12 - y)},
                               {{0, 0}, {0, -1}, {0, 1}});
  Frames<std::uint16_t> f(good, 7, 7);
  const auto e = block_error(good, {0, 0, 8, 8}, f.views, {-6, 0}, BlockMetric::sad);
  CHECK(e.luma == 0 && e.chroma == 0 && e.raw == 0);

  SamplingGeometry quarter;
  quarter.pel = 4;
  quarter.planes[0].current = {12, 12};
  for (int a = 0; a < 16; ++a)
    quarter.planes[0].reference[a] = {12 - (a % 4 == 3), 12 - (a / 4 == 3)};
  CHECK(rejects([&] { validate_sampling_domain(quarter, {0, 0, 4, 4}, {35, 0, 36, 1}); }));
  validate_sampling_domain(quarter, {0, 0, 4, 4}, {31, 0, 32, 1});
  CHECK(rejects([&] { validate_sampling_domain(quarter, {0, 0, 4, 4}, {0, 35, 1, 36}); }));
  quarter.planes[0].reference[3].width = 12; // external pel full declared domain
  validate_sampling_domain(quarter, {0, 0, 4, 4}, {35, 0, 36, 1});
  // A safe Omega cannot excuse an unsafe separately admitted zero-field seed.
  CHECK(rejects([&] { validate_sampling_domain(quarter, {0, 0, 4, 4}, {0, 0, 1, 1}, {{0, -2}}); }));
  CHECK(rejects([&] { validate_sampling_domain(quarter, {0, 0, 4, 4}, {0, 0, 0, 1}); }));
}

// An independent literal Safe(v) oracle, enumerating small candidate domains.
bool safe(const neo_mv::SamplingGeometry& g, neo_mv::BlockRegion b, int vx, int vy) {
  for (int k = 0; k < (g.chroma ? 3 : 1); ++k) {
    const auto& p = g.planes[k];
    const int rx = k == 0 ? 1 : g.ratio_x, ry = k == 0 ? 1 : g.ratio_y;
    const int tx = vx / rx, ty = vy / ry;
    const int qx = static_cast<int>(std::floor(double(tx) / g.pel));
    const int qy = static_cast<int>(std::floor(double(ty) / g.pel));
    const auto e = p.reference[(ty - g.pel * qy) * g.pel + tx - g.pel * qx];
    const int x = p.pad_x + b.x / rx + qx, y = p.pad_y + b.y / ry + qy;
    if (x < 0 || y < 0 || x + b.width / rx > e.width || y + b.height / ry > e.height)
      return false;
  }
  return true;
}

void bounds_against_enumeration() {
  using namespace neo_mv;
  for (int pel : {1, 2, 4})
    for (int pad : {0, 1, 3, 4})
      for (bool chroma : {false, true})
        for (bool restricted : {false, true}) {
          auto g = geometry(pad, pel, chroma, restricted);
          for (int trial = 0; trial < 150; ++trial) {
            const int left = trial * 7 % 47 - 19, top = trial * 11 % 43 - 17;
            const int right = left + trial % 9 + 1, bottom = top + trial % 7 + 1;
            const BlockRegion b{2 * (trial % 5), 2 * ((trial / 5) % 5), 4, 4};
            bool expected = true;
            for (int y = top; y < bottom; ++y)
              for (int x = left; x < right; ++x)
                expected = safe(g, b, x, y) && expected;
            const bool accepted = !rejects([&] { validate_sampling_domain(g, b, {left, top, right, bottom}); });
            CHECK(accepted == expected);
          }
        }
  auto wide = geometry(0, 1, false);
  wide.planes[0].current = wide.planes[0].reference[0] = {INT32_MAX, INT32_MAX};
  validate_sampling_domain(wide, {0, 0, 1, 1}, {0, 0, INT32_MAX, INT32_MAX}); // constant-time, enormous Omega
  CHECK(rejects([&] { validate_sampling_domain(wide, {0, 0, 1, 1}, {INT32_MAX, 0, std::int64_t(INT32_MAX) + 1, 1}); }));
  CHECK(rejects([&] { validate_sampling_domain(wide, {0, 0, 1, 1}, {INT32_MIN, 0, std::int64_t(INT32_MIN) + 1, 1}); }));
}

void signed_sampling() {
  using namespace neo_mv;
  const auto g = geometry(4, 2, true);
  Frames<std::uint8_t> f(g, 0, 0);
  for (int k = 0; k < 3; ++k)
    for (int a = 0; a < 4; ++a) {
      const auto e = g.planes[k].reference[a];
      for (int y = 0; y < e.height; ++y)
        for (int x = 0; x < e.width; ++x)
          f.reference[k][a][y * (e.width + 1) + x] = static_cast<std::uint8_t>(10 * a + x + 2 * y);
    }
  const auto e1 = block_error(g, {0, 0, 4, 4}, f.views, {-1, 0}, BlockMetric::sad);
  CHECK(e1.luma == 408 && e1.chroma == 60 && e1.raw == 468);
  const auto e3 = block_error(g, {0, 0, 4, 4}, f.views, {-3, 0}, BlockMetric::sad);
  CHECK(e3.luma == 392 && e3.chroma == 132 && e3.raw == 524);
  CHECK(rejects([&] { block_error(g, {1, 0, 4, 4}, f.views, {0, 0}, BlockMetric::sad); }));
  CHECK(rejects([&] { block_error(g, {0, 0, 4, 4}, f.views, {INT32_MIN, 0}, BlockMetric::sad); }));
  auto mismatch = f.views;
  mismatch.reference[2][3] = {}; // even an unused declared phase must conform
  CHECK(rejects([&] { block_error(g, {0, 0, 4, 4}, mismatch, {0, 0}, BlockMetric::sad); }));
}

void assembled_metrics() {
  using namespace neo_mv;
  const auto g = geometry(4, 1, true);
  Frames<std::uint8_t> integer(g, 1, 0);
  const auto sad = block_error(g, {0, 0, 8, 8}, integer.views, {0, 0}, BlockMetric::sad);
  const auto satd = block_error(g, {0, 0, 8, 8}, integer.views, {0, 0}, BlockMetric::satd);
  CHECK(sad.luma == 64 && sad.chroma == 32 && sad.raw == 96);
  CHECK(satd.luma == 32 && satd.chroma == 32 && satd.raw == 64);
  Frames<float> floating(g, 1.0f / 65536.0f, 0);
  const auto e = block_error(g, {0, 0, 4, 4}, floating.views, {0, 0}, BlockMetric::satd);
  CHECK(e.luma == 8 && e.chroma == 8 && e.raw == 16);
  Frames<float> saturated(g, 32768, 0);
  const auto full = block_error(g, {0, 0, 4, 4}, saturated.views, {0, 0}, BlockMetric::sad);
  CHECK(full.luma == 4294967295LL && full.chroma == 8589934590LL && full.raw == 12884901885LL);
  // Q is per plane: sub-half chroma metrics individually encode to zero.
  Frames<float> rounded(g, 1.0f / 1048576.0f, 0);
  const auto separate = block_error(g, {0, 0, 4, 4}, rounded.views, {0, 0}, BlockMetric::sad);
  CHECK(separate.luma == 1 && separate.chroma == 0 && separate.raw == 1);
}
} // namespace

int main() {
  try {
    specification_geometry();
    bounds_against_enumeration();
    signed_sampling();
    assembled_metrics();
    std::cout << "Block sampling and whole-domain admission checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
