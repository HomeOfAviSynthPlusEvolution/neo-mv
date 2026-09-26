#include "core/motion/dct.hpp"
#include "core/motion/dct_fft.hpp"
#include <cfenv>
#include "core/motion/analyse.hpp"
#include "core/motion/recalculate.hpp"
#include "motion_fixture.hpp"
#if NEO_MV_DCT_TEST_HIGHWAY
#include "highway/kernels.hpp"
#include "hwy/targets.h"
#endif
#include <future>
#include <iostream>
#include <random>

using namespace neo_mv;
// Standalone adapter for direct sampling tests; production uses the same evaluator.
template <class T>
struct DctBlockError {
  MetricScratch scratch;
  PreparedMetricEvaluator<T, ScalarKernels<T>> evaluator;
  DctBlockError(const SamplingGeometry& geometry, BlockRegion block, const SamplingFrames<T>& frames, int bits)
      : evaluator(metric_layer_plan(MotionMetric::dct, {}), geometry, block, frames, scratch, bits) {}
  BlockError operator()(MotionVector vector) { return evaluator(vector); }
};

static void check(bool ok, const char* message) {
  if (!ok)
    throw std::runtime_error(message);
}
template <class F>
void rejects(F f) {
  try {
    f();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error("missing DCT rejection");
}
// Independent direct cosine summation in double: never calls the production
// FFT, generated constants, or an integer transform. No high-precision library.
std::vector<double> oracle(const std::vector<std::uint16_t>& input, int w, int h) {
  const double pi = std::acos(-1.0);
  std::vector<double> cx(w * w), cy(h * h), rows(w * h), result(w * h);
  for (int k = 0; k < w; ++k)
    for (int x = 0; x < w; ++x)
      cx[k * w + x] = std::cos(pi * (x + 0.5) * k / w);
  for (int k = 0; k < h; ++k)
    for (int y = 0; y < h; ++y)
      cy[k * h + y] = std::cos(pi * (y + 0.5) * k / h);
  for (int y = 0; y < h; ++y)
    for (int u = 0; u < w; ++u)
      for (int x = 0; x < w; ++x)
        rows[y * w + u] += input[y * w + x] * cx[u * w + x];
  for (int v = 0; v < h; ++v)
    for (int u = 0; u < w; ++u) {
      double sum = 0;
      for (int y = 0; y < h; ++y)
        sum += rows[y * w + u] * cy[v * h + y];
      result[v * w + u] = sum * (2 * std::sqrt(2.0) / (w * h));
    }
  return result;
}
const std::vector<std::pair<int, int>> shapes = {{4, 4},   {6, 6},   {8, 4},    {8, 8},    {12, 12}, {16, 2},
                                                 {16, 8},  {16, 16}, {24, 24},  {32, 16},  {32, 32}, {48, 48},
                                                 {64, 32}, {64, 64}, {128, 64}, {128, 128}};
struct DctCase {
  int w, h, bits, pattern;
  std::vector<std::uint16_t> pixels;
  std::vector<double> reference;
};
std::vector<DctCase> make_cases() {
  std::mt19937 rng(729);
  std::vector<DctCase> cases;
  for (auto [w, h] : shapes)
    for (int bits : {8, 10, 12, 14, 16})
      for (int pattern = 0; pattern < 16; ++pattern) {
        DctCase c{w, h, bits, pattern, std::vector<std::uint16_t>(w * h), {}};
        const int max = (1 << bits) - 1, mid = 1 << (bits - 1);
        for (int y = 0; y < h; ++y)
          for (int x = 0; x < w; ++x) {
            int v;
            switch (pattern) {
              case 0:
                v = 0;
                break;
              case 1:
                v = max;
                break;
              case 2:
                v = mid;
                break;
              case 3:
                v = ((x + y) & 1) ? max : 0;
                break;
              case 4:
                v = (x & 1) ? max : 0;
                break;
              case 5:
                v = (y & 1) ? max : 0;
                break;
              case 6:
                v = (x == 0 && y == 0) ? max : 0;
                break;
              case 7:
                v = (x == w / 2 && y == h / 2) ? max : 0;
                break;
              case 8:
                v = x < w / 2 ? max : 0;
                break;
              case 9:
                v = int((x + y) * max / (w + h - 2));
                break;
              case 10:
                v = mid + int(rng() % 3) - 1;
                break;
              case 11:
                v = (x == 0 && y == 0) ? w * h / 4 : 0;
                v = std::min(v, max);
                break;
              default:
                v = int(rng() & max);
                break;
            }
            c.pixels[y * w + x] = std::uint16_t(v);
          }
        c.reference = oracle(c.pixels, w, h);
        cases.push_back(std::move(c));
      }
  return cases;
}
void coefficients(const std::vector<DctCase>& cases, bool simd) {
  double worst = 0;
  std::size_t different = 0, total = 0;
  for (const auto& c : cases) {
    const int w = c.w, h = c.h, stride = dct_detail::padded_stride(w);
    const int size = stride * dct_detail::padded_stride(h), max = (1 << c.bits) - 1, mid = 1 << (c.bits - 1);
    std::vector<float> input(size + 2, 123456.0f), rows = input, output = input;
    // Unused padding must not leak into valid transform coefficients.
    std::fill(input.begin() + 1, input.end() - 1, std::numeric_limits<float>::quiet_NaN());
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        input[1 + y * stride + x] = float(int(c.pixels[y * w + x]) - mid);
    dct_detail::transform_block(w, h, input.data() + 1, rows.data() + 1, output.data() + 1, simd);
    check(rows.front() == 123456 && rows.back() == 123456 && output.front() == 123456 && output.back() == 123456,
          "DCT scratch overrun");
    std::vector<int> q(w * h + 2, -12345);
    dct_detail::quantize_ac(w, h, output.data() + 1, q.data() + 1, max, simd);
    check(q.front() == -12345 && q[1] == -12345 && q.back() == -12345, "DCT quantization guard/DC overrun");
    // A quality regression threshold, not a proven bound for every input.
    // 16-bit permits ~0.013 coefficient units; smaller depths scale with range.
    const double tolerance = 2e-7 * max + 1e-7;
    for (int i = 1; i < w * h; ++i) {
      const double actual = output[1 + (i / w) * stride + i % w], expected = c.reference[i];
      const double error = std::abs(actual - expected);
      worst = std::max(worst, error);
      if (!(error <= tolerance)) {
        std::cerr << "float DCT error " << w << 'x' << h << " bits=" << c.bits << " pattern=" << c.pattern
                  << " coefficient=" << i << " actual=" << actual << " expected=" << expected << " error=" << error
                  << " tolerance=" << tolerance << '\n';
        throw std::runtime_error("float32 DCT exceeds double-reference tolerance");
      }
      const int reference = std::clamp(int(std::nearbyint(expected)) + mid, 0, max);
      check(std::abs(q[i + 1] - reference) <= 1, "DCT integer coefficient differs by more than one");
      different += q[i + 1] != reference;
      ++total;
    }
  }
  std::cout << "double reference: max error=" << worst << ", differing coefficients=" << different << '/' << total
            << '\n';
}
auto view(const std::vector<std::uint16_t>& x, int w, int h) {
  return checked_plane<const std::uint16_t>(x.data(), w, h, w * sizeof(std::uint16_t),
                                            x.size() * sizeof(std::uint16_t));
}
// A one-level impulse has |AC| <= 2*sqrt(2)/area < 0.5 here.
// Consequently this pair differs only in DC, whose exact mean lies immediately
// below an integer boundary. Large 16-bit blocks lose that impulse in float.
void dc_boundary() {
  for (auto size : {std::pair{4, 4},
                    {6, 6},
                    {8, 4},
                    {8, 8},
                    {12, 12},
                    {16, 2},
                    {16, 8},
                    {16, 16},
                    {24, 24},
                    {32, 16},
                    {32, 32},
                    {48, 48},
                    {64, 32},
                    {64, 64},
                    {128, 64},
                    {128, 128}}) {
    const auto [w, h] = size;
    for (int bits : {8, 10, 12, 14, 16}) {
      std::vector<std::uint16_t> a(w * h, (1 << bits) - 2), b = a;
      --a.back();
      DctWorkspace dct(w, h, bits, true);
      dct.set_source(view(a, w, h));
      auto expected = 2LL * int(std::sqrt(float(w * h)) + 0.5f);
      check(dct.compare(view(b, w, h)) == expected, "DC lost a one-level impulse below integer boundary");
      dct.set_source(view(b, w, h));
      check(dct.compare(view(a, w, h)) == expected, "DC boundary symmetry");
    }
  }
}
void numeric() {
  std::mt19937 random(1931);
  for (auto [w, h] : shapes)
    for (int bits : {8, 10, 12, 14, 16})
      for (bool simd : {false, true}) {
        std::vector<std::uint16_t> a(w * h, 32), b(w * h, 96);
        DctWorkspace dct(w, h, bits, simd);
        dct.set_source(view(a, w, h));
        check(dct.compare(view(a, w, h)) == 0, "DCT identity");
        check(dct.compare(view(b, w, h)) == 64LL * int(std::sqrt(double(w * h)) + 0.5), "constant DC normalization");
        for (auto& x : a)
          x = std::uint16_t(random() & ((1 << bits) - 1));
        for (auto& x : b)
          x = std::uint16_t(random() & ((1 << bits) - 1));
        const auto before = a, other = b;
        dct.set_source(view(a, w, h));
        const auto cost = dct.compare(view(b, w, h));
        dct.set_source(view(b, w, h));
        check(dct.compare(view(a, w, h)) == cost, "DCT symmetry");
        check(a == before && b == other, "DCT changed input");
      }
  rejects([] { DctWorkspace d(8, 8, 32); });
  rejects([] { DctWorkspace d(0, 8, 8); });
  rejects([] { DctWorkspace d(10, 8, 8); });
  rejects([] { DctWorkspace d(129, 8, 8); });
  DctWorkspace d(8, 8, 10);
  std::vector<std::uint16_t> outside(64, 1024);
  rejects([&] { d.set_source(view(outside, 8, 8)); });
  rejects([&] { d.set_source(view(outside, 16, 4)); });
  std::vector<float> bad(64, std::numeric_limits<float>::quiet_NaN());
  auto fp = [&] {
    return checked_plane<const float>(bad.data(), 8, 8, 8 * sizeof(float), bad.size() * sizeof(float));
  };
  rejects([&] { d.set_source(fp()); });
  bad[0] = 0.5f;
  rejects([&] { d.set_source(fp()); });
  bad[0] = -1;
  rejects([&] { d.set_source(fp()); });
}
void sampling_and_search() {
  for (int pel : {1, 2, 4})
    for (int bits : {10, 16})
      for (int block : {6, 8, 12}) {
        MotionFixture<std::uint16_t> f(24, 24, block, block, 8, pel, true);
        f.metadata.bits = bits;
        std::fill(f.source[0].begin(), f.source[0].end(), 32);
        for (int k = 0; k < 3; ++k)
          for (int a = 0; a < pel * pel; ++a)
            std::fill(f.reference[k][a].begin(), f.reference[k][a].end(), k == 0 ? 96 : a + 1);
        DctBlockError<std::uint16_t> evaluator(f.geometry, {0, 0, block, block}, f.frames, bits);
        for (auto v : {MotionVector{0, 0}, {-1, -1}, {1, 1}, {-3, 2}}) {
          const auto e = evaluator(v);
          check(e.luma == 64LL * block, "sampled DCT DC");
          const auto sad = block_error(f.geometry, {0, 0, block, block}, f.frames, v, BlockMetric::sad);
          check(e.chroma == sad.chroma && e.raw == e.luma + e.chroma, "DCT chroma SAD");
        }
        AnalyseControls controls;
        controls.metric = MotionMetric::dct;
        controls.badrange = 0;
        controls.mvlambda = 0;
        const auto grid = analyse_vectors<std::uint16_t>(f.metadata, {f.geometry}, {f.frames}, controls);
        check(!grid.values.empty(), "DCT Analyse empty");
        auto old = f.old_field();
        old.grid = grid;
        RecalculateControls r;
        r.metric = MotionMetric::dct;
        r.thsad = 0;
        const auto refined = recalculate_vectors(old, f.metadata, f.geometry, f.frames, r);
        for (int y = 0; y < f.metadata.blocks_y; ++y)
          for (int x = 0; x < f.metadata.blocks_x; ++x) {
            DctBlockError<std::uint16_t> direct(f.geometry, analysis_block(f.metadata, x, y), f.frames, bits);
            const auto index = std::size_t(y) * f.metadata.blocks_x + x;
            check(grid.values[index].error == direct(grid.values[index].vector).raw, "Analyse exported DCT error");
            check(refined.values[index].error == direct(refined.values[index].vector).raw,
                  "Recalculate exported DCT error");
          }
      }
}
#if NEO_MV_DCT_TEST_HIGHWAY
template <class T>
void backend_equivalence() {
  std::mt19937 random(821);
  for (int block : {6, 8, 12})
    for (int pel : {1, 2, 4}) {
      MotionFixture<T> f(24, 24, block, block, 8, pel, true);
      for (int k = 0; k < 3; ++k) {
        for (auto& value : f.source[k])
          value = T(random() & std::numeric_limits<T>::max());
        for (int phase = 0; phase < pel * pel; ++phase)
          for (auto& value : f.reference[k][phase])
            value = T(random() & std::numeric_limits<T>::max());
      }
      const auto same = [](const MotionGrid& a, const MotionGrid& b) {
        check(a.values.size() == b.values.size(), "DCT backend grid size");
        for (std::size_t i = 0; i < a.values.size(); ++i)
          check(a.values[i].vector.x == b.values[i].vector.x && a.values[i].vector.y == b.values[i].vector.y &&
                    a.values[i].error == b.values[i].error,
                "DCT scalar/Highway mismatch");
      };
      for (int search = 0; search <= 5; ++search) {
        AnalyseControls a;
        a.metric = MotionMetric::dct;
        a.search = search;
        a.trymany = search % 3;
        a.badrange = search % 2 ? -2 : 2;
        a.fields = pel > 1;
        a.pelsearch = pel;
        a.meander = search % 2 == 0;
        a.globalmv = search % 2 != 0;
        const int shift = pel > 1 ? (search % 2 ? -pel / 2 : pel / 2) : 0;
        same(analyse_vectors<T>(f.metadata, {f.geometry}, {f.frames}, a, shift),
             analyse_vectors<T, HighwayKernels<T>>(f.metadata, {f.geometry}, {f.frames}, a, shift));
        RecalculateControls r;
        r.metric = MotionMetric::dct;
        r.search = search;
        r.thsad = 0;
        r.smooth = search % 2 == 0;
        same(recalculate_vectors<T>(f.old_field(), f.metadata, f.geometry, f.frames, r),
             recalculate_vectors<T, HighwayKernels<T>>(f.old_field(), f.metadata, f.geometry, f.frames, r));
      }
    }
}
#endif
void quantization() {
  const int saved = std::fegetround();
  // Feed exact represented ties and their immediate neighbors directly to
  // isolate quantization from the float32 transform's approximation error.
  for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    check(std::fesetround(mode) == 0, "set rounding mode");
    for (auto [w, h] : shapes) {
      const int stride = dct_detail::padded_stride(w);
      std::vector<float> v(stride * dct_detail::padded_stride(h));
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
          const int i = y * w + x;
          const float tie = float(((i * 7919) % 131072) - 65536) + 0.5f;
          v[y * stride + x] = i % 3 == 0 ? tie : std::nextafter(tie, i % 3 == 1 ? -INFINITY : INFINITY);
        }
      std::vector<int> a(w * h + 2, -12345), b = a;
      dct_detail::quantize_ac(w, h, v.data(), a.data() + 1, 65535, false);
      dct_detail::quantize_ac(w, h, v.data(), b.data() + 1, 65535, true);
      check(a == b, "scalar/SIMD quantization mismatch");
      check(a.front() == -12345 && a[1] == -12345 && a.back() == -12345, "quantization guard");
      for (int i = 1; i < w * h; ++i) {
        const double value = v[(i / w) * stride + i % w];
        const double low = std::floor(value), fraction = value - low;
        const int q = int(low) + int(fraction > 0.5 || (fraction == 0.5 && int(low) % 2 != 0));
        check(a[i + 1] == std::clamp(q + 32768, 0, 65535), "ties-to-even quantization");
      }
    }
  }
  check(std::fesetround(saved) == 0, "restore rounding mode");
}
int main() {
  try {
    check(parse_motion_metric("dct") == MotionMetric::dct, "DCT parameter parse");
#if NEO_MV_DCT_TEST_HIGHWAY
    backend_equivalence<std::uint8_t>();
    backend_equivalence<std::uint16_t>();
#endif
    const auto cases = make_cases();
    coefficients(cases, false);
    dc_boundary();
#if NEO_MV_DCT_TEST_HIGHWAY
    for (auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      numeric();
      quantization();
      coefficients(cases, true);
      std::cout << "DCT target " << hwy::TargetName(target) << " passed\n";
    }
    hwy::SetSupportedTargetsForTest(0);
#else
    numeric();
    quantization();
    coefficients(cases, true);
#endif
    sampling_and_search();
    std::vector<std::future<void>> workers;
    for (int i = 0; i < 4; ++i)
      workers.push_back(std::async(std::launch::async, sampling_and_search));
    for (auto& worker : workers)
      worker.get();
    std::cout << "DCT numeric, sampling, search and concurrent workspace checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
