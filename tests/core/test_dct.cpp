#include "core/motion/dct.hpp"
#include "core/motion/dct_rounding.hpp"
#include "core/motion/dct_fft.hpp"
#include "core/motion/dct_refine.hpp"
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
// Independent, separable cosine matrix oracle, in double precision. No FFT
// routines or production quantization code are used here.
std::vector<int> oracle(const std::vector<std::uint16_t>& input, int w, int h, int bits) {
  const double pi = std::acos(-1.0);
  std::vector<double> cx(w * w), cy(h * h), rows(w * h);
  std::vector<int> result(w * h);
  for (int k = 0; k < w; ++k)
    for (int x = 0; x < w; ++x)
      cx[k * w + x] = std::cos(pi * (x + 0.5) * k / w);
  for (int k = 0; k < h; ++k)
    for (int y = 0; y < h; ++y)
      cy[k * h + y] = std::cos(pi * (y + 0.5) * k / h);
  for (int y = 0; y < h; ++y)
    for (int u = 0; u < w; ++u)
      for (int x = 0; x < w; ++x)
        rows[y * w + u] += 2 * input[y * w + x] * cx[u * w + x];
  for (int v = 0; v < h; ++v)
    for (int u = 0; u < w; ++u) {
      double value = 0;
      for (int y = 0; y < h; ++y)
        value += 2 * rows[y * w + u] * cy[v * h + y];
      value *= (u == 0 && v == 0 ? 0.125 : 1 / std::sqrt(2.0)) / (w * h);
      int q;
      if (u == 0 && v == 0) {
        std::int64_t sum = 0;
        for (auto sample : input)
          sum += sample;
        q = int(sum / (2 * w * h));
      } else {
        const auto k = std::int64_t(std::floor(value));
        if (std::abs(value - (double(k) + 0.5)) < 1e-8 && dct_detail::exact_half(input.data(), w, h, u, v, k))
          q = int(k + (k % 2 != 0));
        else
          q = int(std::nearbyint(value));
      }
      result[v * w + u] = std::clamp(q + (1 << (bits - 1)), 0, (1 << bits) - 1);
    }
  return result;
}
auto view(const std::vector<std::uint16_t>& x, int w, int h) {
  return checked_plane<const std::uint16_t>(x.data(), w, h, w * sizeof(std::uint16_t),
                                            x.size() * sizeof(std::uint16_t));
}
void bounded_fft() {
  struct RestoreRounding {
    int mode = std::fegetround();
    ~RestoreRounding() { std::fesetround(mode); }
  } restore;
  std::mt19937 random(625);
  for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    check(std::fesetround(mode) == 0, "cannot set rounding mode for DCT test");
    for (int n : {2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 128}) {
      std::vector<double> input(3 * n, -17), output(3 * n, -19);
      for (int pattern = 0; pattern < 4; ++pattern) {
        for (int x = 0; x < n; ++x)
          input[3 * x] = pattern == 0   ? 65535
                         : pattern == 1 ? (x == 0 ? 65535 : 0)
                         : pattern == 2 ? ((x & 1) ? 65535 : 0)
                                        : double(random() & 65535);
        const auto before = input;
        dct_detail::dct_line(n, input.data(), 3, output.data(), 3);
        int ell = 0;
        for (int k = 2 * n; k > 1; k = (k + 1) / 2)
          ++ell;
        const double bound = 2 * n * 65535.0 * (64 * (ell + 1)) * 0x1p-52;
        for (int k = 0; k < n; ++k) {
          check(std::fesetround(FE_TONEAREST) == 0, "cannot set oracle rounding mode");
          double expected = 0;
          // Evaluate the independent cosine oracle in round-to-nearest: some
          // libm implementations lose accuracy in directed rounding modes.
          // The FFT under test was evaluated in the selected mode above.
          // Independent direct cosine sum is a numerical cross-check; the
          // constant generator and analytical bound supply the enclosure proof.
          for (int x = 0; x < n; ++x)
            expected += 2 * input[3 * x] * std::cos(std::acos(-1.0) * (2 * x + 1) * k / (2 * n));
          check(std::fesetround(mode) == 0, "cannot restore FFT rounding mode");
          if (!(std::abs(output[3 * k] - expected) < 4 * bound)) {
            std::cerr << "FFT mismatch mode=" << mode << " n=" << n << " pattern=" << pattern << " k=" << k
                      << " got=" << output[3 * k] << " expected=" << expected << " bound=" << bound << '\n';
            throw std::runtime_error("bounded FFT cosine cross-check");
          }
          if (pattern == 0 && k > 0)
            check(std::abs(output[3 * k]) <= bound, "bounded FFT constant has nonzero AC");
          check(output[3 * k + 1] == -19 && output[3 * k + 2] == -19, "FFT output stride overrun");
        }
        check(input == before, "FFT modified input");
      }
    }
  }
  rejects([] { dct_detail::ac_error_bound(10, 10, 8); });
  rejects([] { dct_detail::ac_error_bound(8, 8, 32); });
  for (int w : {4, 6, 12, 24, 48, 128}) {
    const auto bound = dct_detail::ac_error_bound(w, w, 16);
    check(bound > 0 && bound < 1e-6, "unexpected DCT AC error enclosure");
  }
}
void refinement() {
  for (int sign : {-1, 1})
    for (int odd : {1, 3, 5, 7}) {
      std::array<std::uint16_t, 16> pixels{};
      pixels[sign == 1 ? 0 : 1] = std::uint16_t(4 * odd);
      const int expected = sign * (2 * ((odd + 1) / 4));
      check(dct_detail::interval_round(pixels.data(), 4, 4, 2, 0) == expected,
            "adaptive DCT ties must round to even for both signs");
    }
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
    for (int sign : {-1, 1})
      for (int delta : {-1, 0, 1}) {
        std::vector<std::uint16_t> pixels(w * h);
        pixels[sign == 1 ? 0 : 1] = std::uint16_t(w * h / 4 + delta);
        const int expected = delta > 0 ? sign : 0;
        check(dct_detail::interval_round(pixels.data(), w, h, w / 2, 0) == expected, "adaptive DCT interval");
        check(dct_detail::fixed_round(pixels.data(), w, h, w / 2, 0, expected) == (delta != 0),
              "fixed DCT interval must exclude exact ties");
        check(!dct_detail::fixed_round(pixels.data(), w, h, w / 2, 0, expected + 1), "wrong upper cell");
        check(!dct_detail::fixed_round(pixels.data(), w, h, w / 2, 0, expected - 1), "wrong lower cell");
        check(dct_detail::refine_ac(pixels.data(), w, h, w / 2, 0, sign * 0.5000000001) == expected,
              "DCT estimate dictated refinement result");
      }
  }
  std::mt19937 random(936);
  for (int n : {4, 6, 12, 24, 48, 128}) {
    std::vector<std::uint16_t> pixels(n * n);
    for (auto& p : pixels)
      p = std::uint16_t(random());
    const int u = n / 2 - 1, v = n / 2 + 1;
    const int expected = dct_detail::interval_round(pixels.data(), n, n, u, v);
    check(dct_detail::fixed_round(pixels.data(), n, n, u, v, expected), "random fixed interval");
    check(dct_detail::refine_ac(pixels.data(), n, n, u, v, 0) == expected, "wrong estimate must refine");
  }
}
void exact_boundaries() {
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
    // u=w/2, v=0 has AC=2*sum(sign[x]*pixel)/area, sign=+,-,-,+.
    // Construct exact +/-1/2 and perturb by one unit on either side.
    for (int sign : {-1, 1})
      for (int delta : {-1, 0, 1}) {
        std::vector<std::uint16_t> samples(w * h);
        samples[sign == 1 ? 0 : 1] = std::uint16_t(w * h / 4 + delta);
        const int k = sign == 1 ? 0 : -1;
        check(dct_detail::exact_half(samples.data(), w, h, w / 2, 0, k) == (delta == 0),
              "exact DCT boundary predicate");
        check(!dct_detail::exact_half(samples.data(), w, h, w / 2, 0, k + 1), "wrong neighboring DCT boundary");
        check(!dct_detail::exact_half(samples.data(), w, h, w / 2, 0, k - 1), "wrong neighboring DCT boundary");
      }
  }
  // Mixed frequency: on 12x12, (u=6,v=8) at (x=0,y=1) has
  // cos(x)=sqrt(2)/2 and cos(y)=-1. Pixel324 therefore yields -4.5.
  std::array<std::uint16_t, 144> mixed{};
  mixed[12] = 324;
  check(dct_detail::exact_half(mixed.data(), 12, 12, 6, 8, -5), "mixed DCT frequency tie");
  ++mixed[12];
  check(!dct_detail::exact_half(mixed.data(), 12, 12, 6, 8, -5), "mixed DCT frequency non-tie");
  std::array<std::uint8_t, 16> small{};
  small[0] = 4;
  check(dct_detail::exact_half(small.data(), 4, 4, 2, 0, 0), "8-bit exact DCT tie");
  check(!dct_detail::exact_half(small.data(), 4, 4, 2, 0, INT64_MIN), "DCT lower boundary overflow");
  check(!dct_detail::exact_half(small.data(), 4, 4, 2, 0, INT64_MAX), "DCT upper boundary overflow");
  rejects([&] { dct_detail::exact_half(small.data(), 0, 4, 1, 0, 0); });
  rejects([&] { dct_detail::exact_half(small.data(), 4, 4, 0, 0, 0); });
  rejects([&] { dct_detail::exact_half(small.data(), 4, 4, 4, 0, 0); });
  rejects([&] { dct_detail::exact_half(mixed.data(), 10, 10, 1, 0, 0); });
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
      DctWorkspace dct(w, h, bits);
      dct.set_source(view(a, w, h));
      const auto expected = 2LL * int(std::sqrt(float(w * h)) + 0.5f);
      check(dct.compare(view(b, w, h)) == expected, "DC lost a one-level impulse below integer boundary");
      dct.set_source(view(b, w, h));
      check(dct.compare(view(a, w, h)) == expected, "DC boundary symmetry");
    }
  }
}
void numeric() {
  std::mt19937 random(729);
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
    const int w = size.first, h = size.second, scale = int(std::sqrt(float(w * h)) + 0.5f);
    for (int bits : {8, 10, 12, 14, 16}) {
      std::vector<std::uint16_t> a(w * h, 32), b(w * h, 96);
      DctWorkspace dct(w, h, bits);
      dct.set_source(view(a, w, h));
      check(dct.compare(view(a, w, h)) == 0, "identity DCT");
      check(dct.compare(view(b, w, h)) == 64LL * scale, "constant DC normalization");
      for (auto& x : a)
        x = random() & ((1 << bits) - 1);
      for (auto& x : b)
        x = random() & ((1 << bits) - 1);
      const auto before = a, other = b;
      dct.set_source(view(a, w, h));
      const auto actual = dct.compare(view(b, w, h));
      const auto qa = oracle(a, w, h, bits), qb = oracle(b, w, h, bits);
      std::int64_t expected = 3LL * std::abs(qa[0] - qb[0]);
      for (std::size_t i = 0; i < qa.size(); ++i)
        expected += std::abs(qa[i] - qb[i]);
      expected = expected * scale / 2;
      check(expected == actual, "exact cosine matrix oracle mismatch");
      const int saved_mode = std::fegetround();
      for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
        check(std::fesetround(mode) == 0, "cannot set native DCT rounding mode");
        DctWorkspace native(w, h, bits, true);
        native.set_source(view(a, w, h));
        check(native.compare(view(b, w, h)) == expected, "native DCT quantization mismatch");
      }
      check(std::fesetround(saved_mode) == 0, "cannot restore DCT rounding mode");
      dct.set_source(view(b, w, h));
      check(dct.compare(view(a, w, h)) == actual, "DCT symmetry");
      check(a == before && b == other, "DCT changed input");
      std::fill(a.begin(), a.end(), 0);
      std::fill(b.begin(), b.end(), (1 << bits) - 1);
      dct.set_source(view(a, w, h));
      check(dct.compare(view(b, w, h)) > 0, "DCT extreme pixels");
      b = a;
      b[w * h / 2] = 1;
      dct.set_source(view(b, w, h));
      check(dct.compare(view(b, w, h)) == 0, "DCT impulse identity");
    }
  }
  rejects([] { DctWorkspace d(8, 8, 32); });
  rejects([] { DctWorkspace d(0, 8, 8); });
  rejects([] { DctWorkspace d(129, 8, 8); });
  DctWorkspace d(8, 8, 10);
  std::vector<std::uint16_t> outside(64, 1024);
  rejects([&] { d.set_source(view(outside, 8, 8)); });
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
        controls.metric = BlockMetric::dct;
        controls.badrange = 0;
        controls.mvlambda = 0;
        const auto grid = analyse_vectors<std::uint16_t>(f.metadata, {f.geometry}, {f.frames}, controls);
        check(!grid.values.empty(), "DCT Analyse empty");
        auto old = f.old_field();
        old.grid = grid;
        RecalculateControls r;
        r.metric = BlockMetric::dct;
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
        a.metric = BlockMetric::dct;
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
        r.metric = BlockMetric::dct;
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
  const int saved_mode = std::fegetround();
  std::mt19937 random(319);
  for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    check(std::fesetround(mode) == 0, "cannot set quantization rounding mode");
    for (auto size : {std::pair{4, 4}, {6, 6}, {8, 4}, {16, 2}, {12, 12}, {48, 48}, {128, 128}}) {
      const auto [w, h] = size;
      std::vector<std::uint16_t> samples(w * h);
      std::vector<double> input(w * h), rows(w * h), transformed(w * h);
      for (int pattern = 0; pattern < 4; ++pattern) {
        for (int i = 0; i < w * h; ++i) {
          samples[i] = pattern == 0   ? 65535
                       : pattern == 1 ? (i == 0 ? w * h / 4 : 0)
                       : pattern == 2 ? ((i & 1) ? 65535 : 0)
                                      : std::uint16_t(random());
          input[i] = samples[i];
        }
        dct_detail::transform_block(w, h, input.data(), rows.data(), transformed.data(), false);
        // DC is left untouched. Extra sentinels guard the unaligned AC start
        // and the scalar tail after the final full vector.
        std::vector<int> scalar(w * h + 2, -12345), native = scalar;
        const double error = dct_detail::ac_error_bound(w, h, 16);
        dct_detail::quantize_ac(samples.data(), w, h, transformed.data(), scalar.data() + 1, 65535, error, false);
        dct_detail::quantize_ac(samples.data(), w, h, transformed.data(), native.data() + 1, 65535, error, true);
        check(native == scalar, "DCT per-coefficient SIMD quantization mismatch");
        check(native.front() == -12345 && native[1] == -12345 && native.back() == -12345,
              "DCT quantization overwrote DC or guard");
      }
    }
  }
  check(std::fesetround(saved_mode) == 0, "cannot restore quantization rounding mode");
}
int main() {
  try {
    check(parse_block_metric("dct") == BlockMetric::dct, "DCT parameter parse");
#if NEO_MV_DCT_TEST_HIGHWAY
    backend_equivalence<std::uint8_t>();
    backend_equivalence<std::uint16_t>();
#endif
    refinement();
    bounded_fft();
    exact_boundaries();
    dc_boundary();
#if NEO_MV_DCT_TEST_HIGHWAY
    for (auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      numeric();
      quantization();
      std::cout << "DCT target " << hwy::TargetName(target) << " passed\n";
    }
    hwy::SetSupportedTargetsForTest(0);
#else
    numeric();
    quantization();
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
