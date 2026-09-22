#pragma once
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif
#include <new>
// Each active row ends directly at an inaccessible page.
template <class T> struct GuardBuffer {
  unsigned char *allocation;
  std::size_t page;
  int w, h;
  GuardBuffer(int width, int height) : w(width), h(height) {
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    page = info.dwPageSize;
#else
    const auto size = sysconf(_SC_PAGESIZE);
    check(size > 0, "cannot query page size");
    page = static_cast<std::size_t>(size);
#endif
    check(std::size_t(w) * sizeof(T) <= page, "guard row too large");
#if defined(_WIN32)
    allocation = static_cast<unsigned char *>(VirtualAlloc(nullptr, 2 * page * h, MEM_RESERVE, PAGE_NOACCESS));
    if (!allocation)
#else
    allocation =
        static_cast<unsigned char *>(mmap(nullptr, 2 * page * h, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if (allocation == MAP_FAILED)
#endif
      throw std::bad_alloc();
    for (int y = 0; y < h; ++y) {
#if defined(_WIN32)
      if (!VirtualAlloc(allocation + 2 * page * y, page, MEM_COMMIT, PAGE_READWRITE)) {
        VirtualFree(allocation, 0, MEM_RELEASE);
#else
      if (mprotect(allocation + 2 * page * y, page, PROT_READ | PROT_WRITE) != 0) {
        munmap(allocation, 2 * page * h);
#endif
        throw std::bad_alloc();
      }
      auto *p = reinterpret_cast<T *>(allocation + 2 * page * y + page - w * sizeof(T));
      for (int x = 0; x < w; ++x)
        ::new (p + x) T(0);
    }
  }
  ~GuardBuffer() {
#if defined(_WIN32)
    VirtualFree(allocation, 0, MEM_RELEASE);
#else
    munmap(allocation, 2 * page * h);
#endif
  }
  GuardBuffer(const GuardBuffer &) = delete;
  span2d::Plane<T> view() {
    return neo_mv::checked_plane(reinterpret_cast<T *>(allocation + page - w * sizeof(T)), w, h, 2 * page,
                                 2 * page * h - page + w * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
};
template <class F> void rejects(F f) {
  bool threw = false;
  try {
    f();
  } catch (const std::exception &) {
    threw = true;
  }
  check(threw, "invalid input accepted");
}
template <class T> void boundaries() {
  for (int w : {4, 8, 12, 16, 20, 28, 32, 64, 68, 128, 132}) {
    GuardBuffer<T> a(w, 8), b(w, 8), dst(w / 2, 4), tmp(w, 4);
    for (int y = 0; y < 8; ++y)
      for (int x = 0; x < w; ++x)
        a.view().row(y)[x] = T((x + y) % 17);
    for (auto op : {neo_mv::BlockMetric::sad, neo_mv::BlockMetric::satd})
      check(neo_mv::block_metric(a.read(), b.read(), op) == neo_mv::simd::block_metric(a.read(), b.read(), op),
            "guard metric");
    for (int f = 0; f < 3; ++f)
      neo_mv::simd::reduce_pyramid(a.read(), 0, 0, dst.view(), f, tmp.view());
    std::array<span2d::Plane<T>, 16> views{};
    std::vector<std::unique_ptr<GuardBuffer<T>>> phases;
    for (int i = 0; i < 16; ++i) {
      phases.emplace_back(new GuardBuffer<T>(w, 8));
      views[i] = phases.back()->view();
    }
    for (int sharp = 0; sharp < 3 && 2 * (sharp + 1) <= w; ++sharp)
      neo_mv::simd::interpolate_subpixels(a.read(), 4, sharp, std::is_same_v<T, float> ? 32 : sizeof(T) * 8, views);
  }
  Buffer<T> src(68, 8), other(68, 8);
  rejects([&] { neo_mv::simd::extend_border(src.read(), src.view(), 68, 8, 0, 0); });
  rejects([&] { neo_mv::simd::block_metric(span2d::Plane<const T>{}, other.read(), neo_mv::BlockMetric::sad); });
  if constexpr (std::is_same_v<T, float>) {
    for (float bad : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
      for (int x : {0, 67}) {
        src.view().row(0)[x] = bad;
        rejects([&] { neo_mv::simd::block_metric(src.read(), other.read(), neo_mv::BlockMetric::sad); });
        src.view().row(0)[x] = 0;
      }
    src.view().row(0)[0] = std::numeric_limits<float>::max();
    other.view().row(0)[0] = -std::numeric_limits<float>::max();
    rejects([&] { neo_mv::simd::block_metric(src.read(), other.read(), neo_mv::BlockMetric::sad); });
  }
  if constexpr (std::is_same_v<T, std::uint16_t>)
    for (int bits = 9; bits <= 15; ++bits) {
      std::vector<Buffer<T>> a, b;
      a.reserve(16);
      b.reserve(16);
      std::array<span2d::Plane<T>, 16> av{}, bv{};
      for (int i = 0; i < 16; ++i) {
        a.emplace_back(68, 8);
        b.emplace_back(68, 8);
        av[i] = a.back().view();
        bv[i] = b.back().view();
      }
      for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 68; ++x)
          src.view().row(y)[x] = ((x + y) % 3) ? 0 : T((1 << bits) - 1);
      for (int sharp = 0; sharp < 3; ++sharp) {
        neo_mv::interpolate_subpixels(src.read(), 4, sharp, bits, av);
        neo_mv::simd::interpolate_subpixels(src.read(), 4, sharp, bits, bv);
        for (int i = 0; i < 16; ++i)
          equal(a[i], b[i]);
      }
      src.view().row(0)[67] = T(1 << bits);
      rejects([&] { neo_mv::simd::interpolate_subpixels(src.read(), 4, 2, bits, bv); });
    }
}
void extra_cases() {
  // Non-dyadic samples and crop origins; NaNs outside the consumed crop are legal.
  Buffer<float> src(141, 19), ref(141, 19), a(65, 8), b(65, 8), ta(130, 8), tb(130, 8);
  std::mt19937 rng(51);
  for (int y = 0; y < 19; ++y)
    for (int x = 0; x < 141; ++x) {
      src.view().row(y)[x] = float(int(rng() % 10001) - 5000) / 12345.67f;
      ref.view().row(y)[x] = float(int(rng() % 10001) - 5000) / 12345.67f;
    }
  for (auto op : {neo_mv::BlockMetric::sad, neo_mv::BlockMetric::satd}) {
    const auto s = src.read().subplane(3, 2, 128, 16), r = ref.read().subplane(3, 2, 128, 16);
    check(neo_mv::block_metric(s, r, op) == neo_mv::simd::block_metric(s, r, op), "non-dyadic metric");
  }
  src.view().row(0)[0] = std::numeric_limits<float>::quiet_NaN();
  for (int f = 0; f < 3; ++f) {
    neo_mv::reduce_pyramid(src.read(), 3, 2, a.view(), f, ta.view());
    neo_mv::simd::reduce_pyramid(src.read(), 3, 2, b.view(), f, tb.view());
    equal(a, b);
    equal(ta, tb);
  }
  Buffer<std::uint8_t> base(36, 10), ref8(36, 10);
  neo_mv::SamplingGeometry g{};
  g.planes[0].current = {36, 10};
  g.planes[0].reference[0] = {36, 10};
  neo_mv::SamplingFrames<std::uint8_t> frames{};
  frames.current[0] = base.read();
  frames.reference[0][0] = ref8.read();
  const neo_mv::BlockRegion region{0, 0, 32, 8};
  const auto expected = neo_mv::block_error(g, region, frames, {0, 0}, neo_mv::BlockMetric::satd);
  const auto actual = neo_mv::simd::block_error(g, region, frames, {0, 0}, neo_mv::BlockMetric::satd);
  check(expected.raw == actual.raw && expected.luma == actual.luma && expected.chroma == actual.chroma, "block_error");
  std::array<span2d::Plane<std::uint8_t>, 16> av{}, bv{};
  std::vector<Buffer<std::uint8_t>> aa, bb;
  aa.reserve(16);
  bb.reserve(16);
  for (int i = 0; i < 16; ++i) {
    aa.emplace_back(36, 10);
    bb.emplace_back(36, 10);
    av[i] = aa.back().view();
    bv[i] = bb.back().view();
  }
  for (int pel : {2, 4}) {
    GuardBuffer<std::uint8_t> external(32 * pel, 6 * pel);
    for (int y = 0; y < external.h; ++y)
      for (int x = 0; x < external.w; ++x)
        external.view().row(y)[x] = std::uint8_t(x + y);
    neo_mv::extract_external_subpixels(base.read(), external.read(), 32, 6, 2, 2, pel, 8, av);
    neo_mv::simd::extract_external_subpixels(base.read(), external.read(), 32, 6, 2, 2, pel, 8, bv);
    for (int i = 0; i < pel * pel; ++i)
      equal(aa[i], bb[i]);
  }
  // pel=1 ignores all provided output views and the unused external view.
  auto p =
      neo_mv::simd::extract_external_subpixels(base.read(), span2d::Plane<const std::uint8_t>{}, 32, 6, 2, 2, 1, 8, bv);
  check(p.planes[0].data() == base.read().data(), "pel=1 must borrow base");
  auto bad = bv;
  bad[2] = bad[1];
  rejects([&] { neo_mv::simd::interpolate_subpixels(base.read(), 2, 0, 8, bad); });
  rejects([&] { neo_mv::checked_plane(reinterpret_cast<std::uint16_t *>(base.data.data() + 1), 4, 1, 9, 20); });
}

void external_base_validation() {
  for (int width : {1, 7, 8, 9, 15, 16, 17, 31, 32, 33, 65, 129}) {
    Buffer<float> floats(width, 2);
    Buffer<std::uint16_t> integers(width, 2);
    for (int x = 0; x < width; ++x) {
      floats.view().row(1)[x] = std::numeric_limits<float>::quiet_NaN();
      integers.view().row(1)[x] = 1024;
      // Even pel=1, which ignores the external clip, validates the base image.
      rejects([&] { neo_mv::simd::extract_external_subpixels(floats.read(), {}, width, 2, 0, 0, 1, 32, {}); });
      rejects([&] { neo_mv::simd::extract_external_subpixels(integers.read(), {}, width, 2, 0, 0, 1, 10, {}); });
      floats.view().row(1)[x] = 0;
      integers.view().row(1)[x] = 0;
    }
  }
}

void external_float_contract() {
  Buffer<float> base(32, 8), external(64, 16);
  std::vector<Buffer<float>> dst;
  dst.reserve(4);
  std::array<span2d::Plane<float>, 16> views{};
  for (int i = 0; i < 4; ++i) {
    dst.emplace_back(32, 8);
    views[i] = dst.back().view();
  }
  external.view().row(0)[0] = std::numeric_limits<float>::quiet_NaN();
  neo_mv::simd::extract_external_subpixels(base.read(), external.read(), 32, 8, 0, 0, 2, 32, views);
  external.view().row(0)[1] = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { neo_mv::simd::extract_external_subpixels(base.read(), external.read(), 32, 8, 0, 0, 2, 32, views); });
}
