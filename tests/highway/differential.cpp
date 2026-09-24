#include "../core/motion_fixture.hpp"
#include "core/motion/analyse.hpp"
#include "core/motion/recalculate.hpp"
#include "highway/kernels.hpp"
#include "hwy/targets.h"
#include "hwy/highway.h"
#include <cstring>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
template <class T> struct Buffer {
  int w, h, stride;
  std::vector<T> data;
  Buffer(int width, int height) : w(width), h(height), stride(width + 7), data(std::size_t(stride) * h + 2, T(11)) {}
  span2d::Plane<T> view() {
    return neo_mv::checked_plane(data.data() + 1, w, h, stride * sizeof(T), (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
};
void check(bool ok, const char *what) {
  if (!ok)
    throw std::runtime_error(what);
}
template <class T> void equal(Buffer<T> &a, Buffer<T> &b) {
  check(a.data.size() == b.data.size(), "size");
  for (std::size_t i = 0; i < a.data.size(); ++i) {
    if constexpr (std::is_same_v<T, float>)
      check(std::memcmp(&a.data[i], &b.data[i], sizeof(T)) == 0, "float plane bits mismatch");
    else
      check(a.data[i] == b.data[i], "integer plane mismatch");
  }
}
template <class T> void run() {
  std::mt19937 rng(75142);
  for (int width : {1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129})
    for (int height : {1, 2, 3, 4, 7, 8, 13}) {
      Buffer<T> src(width, height), ref(width, height);
      for (auto *buf : {&src, &ref})
        for (int y = 0; y < height; ++y)
          for (int x = 0; x < width; ++x) {
            if constexpr (std::is_same_v<T, float>)
              buf->view().row(y)[x] = float(int(rng() % 8193) - 4096) / 65535;
            else
              buf->view().row(y)[x] = T(rng());
          }
      for (auto op : {neo_mv::BlockMetric::sad, neo_mv::BlockMetric::satd}) {
        if (op == neo_mv::BlockMetric::satd && (width % 4 || height % 4))
          continue;
        const auto expected = neo_mv::block_metric(src.read(), ref.read(), op);
        const auto actual = neo_mv::simd::block_metric(src.read(), ref.read(), op);
        if (expected != actual) {
          std::cerr << "metric mismatch: type="
                    << (std::is_same_v<T, float> ? "float" : sizeof(T) == 1 ? "uint8" : "uint16")
                    << " width=" << width << " height=" << height
                    << " metric=" << (op == neo_mv::BlockMetric::sad ? "SAD" : "SATD")
                    << " scalar=" << expected << " simd=" << actual << '\n';
#if defined(_M_ARM64)
          if constexpr (std::is_same_v<T, std::uint16_t>) {
            namespace hn = hwy::HWY_NAMESPACE;
            const hn::CappedTag<std::int32_t, 2> d;
            const hn::Rebind<T, decltype(d)> narrow;
            for (auto* buffer : {&src, &ref}) {
              std::cerr << (buffer == &src ? "source\n" : "reference\n");
              for (int y = 0; y < height; ++y) {
                const auto* p = &buffer->read().row(y)[0];
                for (int x = 0; x < width; ++x) std::cerr << p[x] << ' ';
                std::cerr << "\nloaded: ";
                auto a = hn::Zero(narrow), b = a, c = a, e = a;
                hn::LoadInterleaved4(narrow, p, a, b, c, e);
                for (auto v : {a, b, c, e}) {
                  std::int32_t lanes[2];
                  hn::StoreU(hn::PromoteTo(d, v), d, lanes);
                  std::cerr << '[' << lanes[0] << ',' << lanes[1] << "] ";
                }
                std::cerr << '\n';
              }
            }
            for (int x = 0; x < width; x += 4) {
              const auto a = src.read().subplane(x, 0, 4, 4);
              const auto b = ref.read().subplane(x, 0, 4, 4);
              std::cerr << "cell " << x << " scalar=" << neo_mv::block_metric(a, b, op)
                        << " simd=" << neo_mv::simd::block_metric(a, b, op) << '\n';
            }
          }
#endif
          throw std::runtime_error("metric mismatch");
        }
      }
      Buffer<T> a(width + 7, height + 5), b(width + 7, height + 5);
      neo_mv::extend_border(src.read(), a.view(), width + 3, height + 1, 2, 2);
      neo_mv::simd::extend_border(src.read(), b.view(), width + 3, height + 1, 2, 2);
      equal(a, b);
      if (width >= 2 && height >= 2)
        for (int filter = 0; filter < 3; ++filter) {
          Buffer<T> ra(width / 2, height / 2), rb(width / 2, height / 2), ta(2 * (width / 2), height / 2),
              tb(2 * (width / 2), height / 2);
          neo_mv::reduce_pyramid(src.read(), 0, 0, ra.view(), filter, ta.view());
          neo_mv::simd::reduce_pyramid(src.read(), 0, 0, rb.view(), filter, tb.view());
          equal(ra, rb);
          equal(ta, tb);
        }
      if (width >= 2 && height >= 2)
        for (int sharp = 0; sharp < 3; ++sharp) {
          if (width < 2 * (sharp + 1) || height < 2 * (sharp + 1))
            continue;
          for (int pel : {2, 4}) {
            std::vector<Buffer<T>> aa, bb;
            aa.reserve(16);
            bb.reserve(16);
            std::array<span2d::Plane<T>, 16> av{}, bv{};
            for (int i = 0; i < pel * pel; ++i) {
              aa.emplace_back(width, height);
              bb.emplace_back(width, height);
              av[i] = aa.back().view();
              bv[i] = bb.back().view();
            }
            const int bits = std::is_same_v<T, float> ? 32 : int(sizeof(T) * 8);
            const auto ap = neo_mv::interpolate_subpixels(src.read(), pel, sharp, bits, av);
            const auto bp = neo_mv::simd::interpolate_subpixels(src.read(), pel, sharp, bits, bv);
            for (int i = 0; i < pel * pel; ++i) {
              equal(aa[i], bb[i]);
              check(ap.planes[i].width() == bp.planes[i].width() && ap.planes[i].height() == bp.planes[i].height(),
                    "phase extent");
            }
            Buffer<T> external(pel * width, pel * height);
            for (int y = 0; y < external.h; ++y)
              for (int x = 0; x < external.w; ++x)
                external.view().row(y)[x] = T((x + y) % 13);
            neo_mv::extract_external_subpixels(src.read(), external.read(), width, height, 0, 0, pel, bits, av);
            neo_mv::simd::extract_external_subpixels(src.read(), external.read(), width, height, 0, 0, pel, bits, bv);
            for (int i = 0; i < pel * pel; ++i)
              equal(aa[i], bb[i]);
          }
        }
    }
}
#include "boundaries.hpp"

void sharp6_integer_boundaries() {
  for (int width : {1, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65})
    for (int bits : {9, 10, 12, 16})
      for (int pattern = 0; pattern < 4; ++pattern) {
        const int maximum = (1 << bits) - 1;
        std::array<std::vector<std::uint16_t>, 6> rows;
        const std::uint16_t* taps[6];
        std::vector<std::uint16_t> output(width + 2, 123);
        constexpr int coefficients[] = {1, -5, 20, 20, -5, 1};
        for (int tap = 0; tap < 6; ++tap) {
          rows[tap].resize(width + 1);
          taps[tap] = rows[tap].data() + 1;
          for (int x = 0; x < width; ++x)
            rows[tap][x + 1] = std::uint16_t(pattern == 0 ? 0 : pattern == 1 ? maximum :
                                            ((coefficients[tap] > 0) == ((x + pattern) % 2 == 0) ? maximum : 0));
        }
        neo_mv::simd::detail::formula(taps, 1, output.data() + 1, width, neo_mv::simd::detail::Formula::sharp6, maximum);
        for (int x = 0; x < width; ++x) {
          int sum = 16;
          for (int tap = 0; tap < 6; ++tap)
            sum += coefficients[tap] * taps[tap][x];
          const int expected = std::min(maximum, std::max(0, sum) / 32);
          check(output[x + 1] == expected, "sharp6 integer clipping mismatch");
        }
        check(output.front() == 123 && output.back() == 123, "sharp6 output canary");
      }
}

template <class T> void sampled_motion(int block_width = 8) {
  using namespace neo_mv;
  for (int pel : {1, 2, 4}) {
    MotionFixture<T> f(2 * block_width, 2 * block_width, block_width, block_width, 4, pel, true);
    for (int k = 0; k < 3; ++k) {
      for (std::size_t i = 0; i < f.source[k].size(); ++i)
        f.source[k][i] = T((i * 13 + k * 7) % 233);
      for (int a = 0; a < pel * pel; ++a) {
        const auto extent = f.geometry.planes[k].reference[a];
        const int stride = extent.width + 1 + a;
        auto& storage = f.reference[k][a];
        storage.resize(std::size_t(stride) * extent.height);
        f.frames.reference[k][a] = checked_plane<const T>(storage.data(), extent.width, extent.height,
                                                          stride * sizeof(T), storage.size() * sizeof(T));
        for (std::size_t i = 0; i < storage.size(); ++i)
          storage[i] = T((i * 17 + a * 11 + k * 5) % 241);
      }
    }
    if constexpr (std::is_same_v<T, float>) {
      for (auto &plane : f.source)
        for (auto &v : plane)
          v = (v - 127) / 65535;
      for (auto &plane : f.reference)
        for (auto &phase : plane)
          for (auto &v : phase)
            v = (v - 127) / 65535;
    }
    const auto same_error = [&](MotionVector v) {
      for (auto metric : {BlockMetric::sad, BlockMetric::satd}) {
        const auto a = block_error(f.geometry, {0, 0, block_width, block_width}, f.frames, v, metric);
        const auto b = simd::block_error(f.geometry, {0, 0, block_width, block_width}, f.frames, v, metric);
        auto prepared = simd::PreparedBlockError<T>(f.geometry, {0, 0, block_width, block_width}, f.frames, metric);
        const auto c = prepared(v);
        decltype(auto) prepared_frames = HighwayKernels<T>::prepare_frames(f.geometry, f.frames);
        auto shared = HighwayKernels<T>::prepare_block_error(f.geometry, {0, 0, block_width, block_width}, prepared_frames, metric);
        const auto d = shared(v);
        check(a.luma == b.luma && a.chroma == b.chroma && a.raw == b.raw && a.luma == c.luma &&
                  a.chroma == c.chroma && a.raw == c.raw && a.luma == d.luma && a.chroma == d.chroma &&
                  a.raw == d.raw,
              "sampled error mismatch");
        const auto admitted = shared.bounded(v, a.raw + 1, {0, 0}, 0, 0);
        check(admitted && admitted->luma == a.luma && admitted->chroma == a.chroma && admitted->raw == a.raw,
              "bounded metric changed an admitted candidate");
        if constexpr (std::is_integral_v<T>)
          if (metric == BlockMetric::sad && a.raw > 0)
            check(!shared.bounded(v, a.raw, {0, 0}, 0, 0), "bounded metric admitted a non-improving candidate");
        // Bounded rejection must not leave stale references in full evaluation.
        auto copied = prepared;
        auto borrowed_copy = shared;
        (void)copied.bounded(v, a.raw, v, 0, 0);
        (void)borrowed_copy.bounded(v, a.raw, v, 0, 0);
        const auto again = copied(v), borrowed_again = borrowed_copy(v);
        check(again.raw == a.raw && again.luma == a.luma && again.chroma == a.chroma &&
                  borrowed_again.raw == a.raw && borrowed_again.luma == a.luma &&
                  borrowed_again.chroma == a.chroma, "interleaved copied evaluator mismatch");
        const auto copied_bound = copied.bounded(v, a.raw + 1, v, 0, 0);
        check(copied_bound && copied_bound->raw == a.raw && copied_bound->luma == a.luma &&
                  copied_bound->chroma == a.chroma, "copied bounded evaluator mismatch");
      }
    };
    for (int y : {-3, -1, 0, 1, 3})
      for (int x : {-3, -1, 0, 1, 3})
        same_error({x, y});
    if constexpr (std::is_integral_v<T>)
      if (pel == 2) {
        decltype(auto) prepared_frames = HighwayKernels<T>::prepare_frames(f.geometry, f.frames);
        auto shared = HighwayKernels<T>::prepare_block_error(f.geometry, {0, 0, block_width, block_width}, prepared_frames,
                                                              BlockMetric::sad);
        bool overflow = false;
        try {
          refine_motion({{0, 0}, 0, 0}, {{INT32_MIN, 0}, INT64_MAX, 0, {-1, -1, 2, 2}, 4, 1, {}}, shared);
        } catch (const std::overflow_error&) {
          overflow = true;
        }
        check(overflow, "bounded search hid a candidate cost overflow");
      }

    const auto same_grid = [](const MotionGrid &a, const MotionGrid &b) {
      check(a.width == b.width && a.height == b.height && a.values.size() == b.values.size(), "grid geometry");
      for (std::size_t i = 0; i < a.values.size(); ++i)
        check(a.values[i].vector.x == b.values[i].vector.x && a.values[i].vector.y == b.values[i].vector.y &&
                  a.values[i].error == b.values[i].error,
              "motion grid mismatch");
    };
    for (int search = 0; search <= 5; ++search)
      for (bool satd : {false, true})
        for (int trymany : {0, 1, 2}) {
        AnalyseControls a;
        a.search = search;
        a.satd = satd;
        a.trymany = trymany;
        same_grid(analyse_vectors<T>(f.metadata, {f.geometry}, {f.frames}, a),
                  analyse_vectors<T, HighwayKernels<T>>(f.metadata, {f.geometry}, {f.frames}, a));
        RecalculateControls r;
        r.search = search;
        r.satd = satd;
        r.thsad = 0;
        same_grid(recalculate_vectors(f.old_field(), f.metadata, f.geometry, f.frames, r),
                  recalculate_vectors<T, HighwayKernels<T>>(f.old_field(), f.metadata, f.geometry, f.frames, r));
      }
    if constexpr (std::is_integral_v<T>) {
      for (int k = 0; k < 3; ++k) {
        for (std::size_t i = 0; i < f.source[k].size(); ++i)
          f.source[k][i] = i % 2 ? std::numeric_limits<T>::max() : T(0);
        for (int a = 0; a < pel * pel; ++a)
          for (std::size_t i = 0; i < f.reference[k][a].size(); ++i)
            f.reference[k][a][i] = i % 2 ? T(0) : std::numeric_limits<T>::max();
      }
      same_error({0, 0});
      same_error({-3, 3});
    }
    if (pel == 4 && block_width == 8) {
      for (int k = 0; k < 3; ++k)
        for (int a = 0; a < 16; ++a) {
          auto &extent = f.geometry.planes[k].reference[a];
          extent.width -= a % 4 == 3;
          extent.height -= a / 4 == 3;
          f.frames.reference[k][a] = f.frames.reference[k][a].subplane(0, 0, extent.width, extent.height);
        }
      same_error({-1, -3});
      rejects([&] { simd::block_error(f.geometry, {8, 8, 8, 8}, f.frames, {19, 0}, BlockMetric::sad); });
      same_error({15, 15});
    }
  }
}
template <class T> void fused_analyse_block() {
  using namespace neo_mv;
  for (int pel : {1, 2, 4}) {
    MotionFixture<T> f(32, 32, 16, 16, 16, pel, true);
    const BlockRegion region{0, 0, 16, 16};
    const CandidateDomain omega{-3, -3, 4, 4};
    const MotionTriple predictor{{1, -1}, 0};
    const MotionVector zero{-4, 0}; // Safe seed outside Omega remains eligible.
    const SpatialPredictors spatial{{{{{-1, 0}, 0}, {{1, 0}, 0}, {{0, 1}, 0}, {{0, -1}, 0}}}, {2, -2}};
    validate_sampling_domain(f.geometry, region, omega, {zero});
    for (int pattern = 0; pattern < 3; ++pattern) {
      for (int k = 0; k < 3; ++k) {
        for (std::size_t i = 0; i < f.source[k].size(); ++i)
          f.source[k][i] = pattern == 0 ? T(0) : T((i * 997 + k * 113) & std::numeric_limits<T>::max());
        for (int a = 0; a < pel * pel; ++a) {
          const auto extent = f.geometry.planes[k].reference[a];
          const int stride = extent.width + a + 3;
          auto& storage = f.reference[k][a];
          storage.resize(std::size_t(stride) * extent.height);
          f.frames.reference[k][a] = checked_plane<const T>(storage.data(), extent.width, extent.height,
              stride * sizeof(T), storage.size() * sizeof(T));
          for (std::size_t i = 0; i < storage.size(); ++i)
            storage[i] = pattern == 0 ? T(0) : pattern == 1 ? std::numeric_limits<T>::max()
                : T((i * 313 + a * 251 + k * 17) & std::numeric_limits<T>::max());
        }
      }
      validate_sampling_frames(f.geometry, f.frames);
      auto owned = simd::PreparedBlockError<T>(f.geometry, region, f.frames, BlockMetric::sad);
      auto frames = simd::PreparedSamplingFrames<T>(f.geometry, f.frames);
      auto borrowed = simd::PreparedBlockError<T, false>(f.geometry, region, frames, BlockMetric::sad);
      auto copied = borrowed;
      const auto scalar = [&](MotionVector v) {
        return block_error<T, true>(f.geometry, region, f.frames, v, BlockMetric::sad);
      };
      for (int search = 0; search <= 5; ++search)
        for (int scenario = 0; scenario < 5; ++scenario) {
          const std::int64_t lambdas[] = {0, 17, INT32_MAX, std::int64_t(INT32_MAX) + 1, INT64_MAX};
          const auto lambda = lambdas[scenario];
          AnalyseControls c;
          c.search = search;
          c.searchparam = c.pelsearch = 3;
          c.trymany = scenario % 3;
          c.badrange = scenario % 2 ? -3 : 3;
          c.pnew = scenario % 2 ? 256 : 0;
          c.pzero = pattern == 0 ? 0 : 25;
          c.pglobal = 127;
          const int layer = scenario == 2 ? 1 : 0;
          const auto threshold = scenario == 0 ? INT64_MAX : std::int64_t{-1};
          SearchResult expected{};
          bool overflow = false;
          try {
            expected = analyse_detail::block(predictor, spatial, zero, omega, layer, pel, lambda, threshold, c, scalar);
          } catch (const std::overflow_error&) {
            overflow = true;
          }
          const auto verify = [&](auto& evaluate) {
            bool actual_overflow = false;
            try {
              const auto actual = evaluate.analyse(predictor, spatial, zero, omega, layer, pel, lambda, threshold, c);
              check(!overflow && actual.vector.x == expected.vector.x && actual.vector.y == expected.vector.y &&
                        actual.cost == expected.cost && actual.raw == expected.raw, "fused analysis result");
            } catch (const std::overflow_error&) {
              actual_overflow = true;
            }
            check(actual_overflow == overflow, "fused analysis changed overflow behavior");
          };
          verify(owned);
          verify(copied);
          check(owned(zero).raw == scalar(zero).raw, "fused/full interleaving");
        }
    }
  }
}

template <class T> void narrow_reference_motion() {
  using namespace neo_mv;
  Buffer<T> current(100, 1), reference(4, 1);
  SamplingGeometry geometry;
  geometry.planes[0].current = {100, 1};
  geometry.planes[0].reference[0] = {4, 1};
  SamplingFrames<T> frames;
  frames.current[0] = current.read();
  frames.reference[0][0] = reference.read();
  const BlockRegion block{96, 0, 4, 1};
  const MotionVector vector{-96, 0};
  validate_sampling_domain(geometry, block, sampling_detail::singleton(vector));
  validate_sampling_frames(geometry, frames);
  const auto scalar = block_error<T, true>(geometry, block, frames, vector, BlockMetric::sad);
  const auto highway = simd::block_error<T, true>(geometry, block, frames, vector, BlockMetric::sad);
  auto prepared = simd::PreparedBlockError<T>(geometry, block, frames, BlockMetric::sad);
  const auto fast = prepared(vector);
  check(scalar.luma == highway.luma && scalar.raw == highway.raw && scalar.luma == fast.luma &&
            scalar.raw == fast.raw, "narrow reference mismatch");
}

template <class T> void bounded_narrow_reference_motion() {
  using namespace neo_mv;
  for (int width : {8, 16})
    for (int pel : {1, 2, 4}) {
      MotionFixture<T> f(96 + width, width, width, width, 0, pel, true);
      std::array<std::array<std::unique_ptr<GuardBuffer<T>>, 16>, 3> guarded;
      for (int k = 0; k < 3; ++k)
        for (int a = 0; a < pel * pel; ++a) {
          const int w = (k == 0 ? width : width / 2) + 4;
          guarded[k][a] = std::make_unique<GuardBuffer<T>>(w, w);
          for (int y = 0; y < w; ++y)
            for (int x = 0; x < w; ++x)
              guarded[k][a]->view().row(y)[x] = T((x + y + a + k) * 7);
          f.frames.reference[k][a] = guarded[k][a]->read();
          f.geometry.planes[k].reference[a] = {w, w};
        }
      const BlockRegion block{96, 0, width, width};
      auto prepared = simd::PreparedBlockError<T>(f.geometry, block, f.frames, BlockMetric::sad);
      for (int y = 0; y < 2 * pel; ++y)
        for (int x = 0; x < 2 * pel; ++x) {
          const MotionVector vector{-96 * pel + x, y};
          const auto expected = block_error<T, true>(f.geometry, block, f.frames, vector, BlockMetric::sad);
          const auto actual = prepared.bounded(vector, expected.raw + 1, vector, 0, 0);
          check(actual && actual->luma == expected.luma && actual->chroma == expected.chroma &&
                    actual->raw == expected.raw, "bounded narrow reference mismatch");
          check(!prepared.bounded(vector, expected.raw, vector, 0, 0), "bounded narrow reference threshold");
        }
      if (width == 16) {
        const MotionVector zero{-96 * pel, 0};
        const CandidateDomain omega{zero.x, 0, zero.x + 2 * pel, 2 * pel};
        const SpatialPredictors spatial{{{{zero, 0}, {zero, 0}, {zero, 0}, {zero, 0}}}, zero};
        AnalyseControls c;
        c.trymany = 2;
        const auto scalar = [&](MotionVector v) {
          return block_error<T, true>(f.geometry, block, f.frames, v, BlockMetric::sad);
        };
        const auto expected = analyse_detail::block({zero, 0}, spatial, zero, omega, 0, pel, 17, -1, c, scalar);
        const auto actual = prepared.analyse({zero, 0}, spatial, zero, omega, 0, pel, 17, -1, c);
        check(actual.vector.x == expected.vector.x && actual.vector.y == expected.vector.y &&
                  actual.cost == expected.cost && actual.raw == expected.raw, "fused narrow reference");
      }
    }
}

template <class T> void integer_metric_extremes() {
  // Exercise bounded SAD accumulation, both fallback dimensions, and SATD
  // reductions with full-range differences (not only random small blocks).
  for (int w : {4, 8, 16, 32, 64, 128, 129, 132})
    for (int h : {4, 128, 129, 512}) {
      Buffer<T> a(w, h), b(w, h);
      for (int pattern = 0; pattern < 3; ++pattern) {
        for (int y = 0; y < h; ++y)
          for (int x = 0; x < w; ++x) {
            const auto high = std::numeric_limits<T>::max();
            a.view().row(y)[x] = pattern == 0 ? T(0) : high;
            b.view().row(y)[x] = pattern == 0 || (pattern == 2 && (x + y) % 2) ? high : T(0);
          }
        for (auto metric : {neo_mv::BlockMetric::sad, neo_mv::BlockMetric::satd}) {
          if (metric == neo_mv::BlockMetric::satd && (w % 4 || h % 4))
            continue;
          check(neo_mv::block_metric(a.read(), b.read(), metric) ==
                    neo_mv::simd::block_metric(a.read(), b.read(), metric),
                "full-range integer metric mismatch");
        }
      }
    }
}

template <class T> void bounded_sad_thresholds() {
  using namespace neo_mv::simd::detail;
  for (int width : {8, 16}) {
    std::array<std::unique_ptr<Buffer<T>>, 3> source;
    std::array<std::unique_ptr<GuardBuffer<T>>, 3> reference;
    std::array<MetricRequest<T>, 3> requests{};
    for (int k = 0; k < 3; ++k) {
      const int w = k == 0 ? width : width / 2;
      source[k] = std::make_unique<Buffer<T>>(w, w);
      reference[k] = std::make_unique<GuardBuffer<T>>(w, w);
      const auto a = source[k]->read(), b = reference[k]->read();
      requests[k] = {a.data(), a.stride(), b.data(), b.stride(), w, w, false};
    }
    const auto bounded = width == 16 ? metric_batch_420_bounded_function(static_cast<T*>(nullptr))
                                     : metric_batch_420_small_bounded_function(static_cast<T*>(nullptr));
    const auto full = width == 16 ? metric_batch_420_function(static_cast<T*>(nullptr))
                                  : metric_batch_420_small_function(static_cast<T*>(nullptr));
    const auto verify = [&] {
      std::array<std::int64_t, 3> expected{}, complete{};
      std::int64_t total = 0;
      for (int k = 0; k < 3; ++k) {
        expected[k] = neo_mv::block_metric(source[k]->read(), reference[k]->read(), neo_mv::BlockMetric::sad);
        total += expected[k];
      }
      full(requests.data(), 3, complete.data());
      check(complete == expected, "complete fixed SAD mismatch");
      for (auto limit : {std::int64_t{-1}, std::int64_t{0}, total / 2, total - 1, total, total + 1, INT64_MAX}) {
        std::array<std::int64_t, 3> errors{};
        const bool accepted = bounded(requests.data(), limit, errors.data());
        check(accepted == (total < limit), "bounded SAD threshold mismatch");
        if (accepted)
          check(errors == expected, "bounded SAD complete error mismatch");
      }
    };
    const auto clear = [&] {
      for (int k = 0; k < 3; ++k)
        for (int y = 0; y < requests[k].height; ++y)
          for (int x = 0; x < requests[k].width; ++x) {
            source[k]->view().row(y)[x] = T(0);
            reference[k]->view().row(y)[x] = T(0);
          }
    };
    const int maximum = std::numeric_limits<T>::max();
    for (int value : {0, maximum / 2, maximum / 2 + 1, maximum}) {
      clear();
      for (int k = 0; k < 3; ++k)
        for (int y = 0; y < requests[k].height; ++y)
          for (int x = 0; x < requests[k].width; ++x) {
            source[k]->view().row(y)[x] = (x + y) % 2 ? T(value) : T(0);
            reference[k]->view().row(y)[x] = (x + y) % 2 ? T(0) : T(value);
          }
      verify();
    }
    // Pulses on both sides of every prefix boundary, including U/V-only errors.
    for (int plane = 0; plane < 3; ++plane)
      for (int row = 0; row < requests[plane].height; ++row) {
        clear();
        for (int x = 0; x < requests[plane].width; ++x)
          reference[plane]->view().row(row)[x] = T(maximum);
        verify();
      }
  }
}

int main() {
  try {
    for (auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << "Testing " << hwy::TargetName(target) << std::endl;
      check(std::strcmp(neo_mv::simd::detail::target_name(), hwy::TargetName(target)) == 0, "dispatch target mismatch");
      run<std::uint8_t>();
      run<std::uint16_t>();
      run<float>();
      sharp6_integer_boundaries();
      bounded_sad_thresholds<std::uint8_t>();
      bounded_sad_thresholds<std::uint16_t>();
      integer_metric_extremes<std::uint8_t>();
      integer_metric_extremes<std::uint16_t>();
      boundaries<std::uint8_t>();
      boundaries<std::uint16_t>();
      boundaries<float>();
      sampled_motion<std::uint8_t>();
      sampled_motion<std::uint16_t>();
      sampled_motion<std::uint8_t>(16);
      sampled_motion<std::uint16_t>(16);
      sampled_motion<float>();
      fused_analyse_block<std::uint8_t>();
      fused_analyse_block<std::uint16_t>();
      narrow_reference_motion<std::uint8_t>();
      narrow_reference_motion<std::uint16_t>();
      narrow_reference_motion<float>();
      bounded_narrow_reference_motion<std::uint8_t>();
      bounded_narrow_reference_motion<std::uint16_t>();
      extra_cases();
      external_base_validation();
      external_float_contract();
    }
    hwy::SetSupportedTargetsForTest(0);
    std::cout << "All differential checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
