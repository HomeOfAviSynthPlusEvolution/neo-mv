#include "../core/motion_fixture.hpp"
#include "core/motion/analyse.hpp"
#include "core/motion/recalculate.hpp"
#include "highway/kernels.hpp"
#include "hwy/targets.h"
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
        check(neo_mv::block_metric(src.read(), ref.read(), op) ==
                  neo_mv::simd::block_metric(src.read(), ref.read(), op),
              "metric mismatch");
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

template <class T> void sampled_motion() {
  using namespace neo_mv;
  for (int pel : {1, 2, 4}) {
    MotionFixture<T> f(16, 16, 8, 8, 4, pel, true);
    for (int k = 0; k < 3; ++k) {
      for (std::size_t i = 0; i < f.source[k].size(); ++i)
        f.source[k][i] = T((i * 13 + k * 7) % 233);
      for (int a = 0; a < pel * pel; ++a)
        for (std::size_t i = 0; i < f.reference[k][a].size(); ++i)
          f.reference[k][a][i] = T((i * 17 + a * 11 + k * 5) % 241);
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
        const auto a = block_error(f.geometry, {0, 0, 8, 8}, f.frames, v, metric);
        const auto b = simd::block_error(f.geometry, {0, 0, 8, 8}, f.frames, v, metric);
        check(a.luma == b.luma && a.chroma == b.chroma && a.raw == b.raw, "sampled error mismatch");
      }
    };
    for (int y : {-3, -1, 0, 1, 3})
      for (int x : {-3, -1, 0, 1, 3})
        same_error({x, y});

    const auto same_grid = [](const MotionGrid &a, const MotionGrid &b) {
      check(a.width == b.width && a.height == b.height && a.values.size() == b.values.size(), "grid geometry");
      for (std::size_t i = 0; i < a.values.size(); ++i)
        check(a.values[i].vector.x == b.values[i].vector.x && a.values[i].vector.y == b.values[i].vector.y &&
                  a.values[i].error == b.values[i].error,
              "motion grid mismatch");
    };
    for (int search = 0; search <= 5; ++search)
      for (bool satd : {false, true}) {
        AnalyseControls a;
        a.search = search;
        a.satd = satd;
        same_grid(analyse_vectors<T>(f.metadata, {f.geometry}, {f.frames}, a),
                  analyse_vectors<T, HighwayKernels<T>>(f.metadata, {f.geometry}, {f.frames}, a));
        RecalculateControls r;
        r.search = search;
        r.satd = satd;
        r.thsad = 0;
        same_grid(recalculate_vectors(f.old_field(), f.metadata, f.geometry, f.frames, r),
                  recalculate_vectors<T, HighwayKernels<T>>(f.old_field(), f.metadata, f.geometry, f.frames, r));
      }
    if (pel == 4) {
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

int main() {
  try {
    for (auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << "Testing " << hwy::TargetName(target) << std::endl;
      check(std::strcmp(neo_mv::simd::detail::target_name(), hwy::TargetName(target)) == 0, "dispatch target mismatch");
      run<std::uint8_t>();
      run<std::uint16_t>();
      run<float>();
      boundaries<std::uint8_t>();
      boundaries<std::uint16_t>();
      boundaries<float>();
      sampled_motion<std::uint8_t>();
      sampled_motion<std::uint16_t>();
      sampled_motion<float>();
      extra_cases();
      external_float_contract();
    }
    hwy::SetSupportedTargetsForTest(0);
    std::cout << "All differential checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
