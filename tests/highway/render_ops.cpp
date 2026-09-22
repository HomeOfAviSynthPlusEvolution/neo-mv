#include "highway/render_ops.hpp"
#include "kernels/render_scalar.hpp"
#include "hwy/targets.h"
#include <iostream>
#include <cstring>
#include <random>
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
namespace {
void check(bool v) {
  if (!v)
    throw std::runtime_error("render operation mismatch");
}
template <class T>
struct Buffer {
  int w, h, stride;
  std::vector<T> data;
  Buffer(int x, int y) : w(x), h(y), stride(x + 5), data(std::size_t(stride) * h + 1, T(17)) {}
  span2d::Plane<T> view() {
    return neo_mv::checked_plane(data.data() + 1, w, h, std::ptrdiff_t(stride) * sizeof(T),
                                 (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
  void random(std::mt19937& rng, int bits) {
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        if constexpr (std::is_same_v<T, float>)
          view().row(y)[x] = float(int(rng() % 20001) - 10000) / 987.0f;
        else
          view().row(y)[x] = T(rng() & ((1u << bits) - 1));
      }
  }
};
template <class T>
void equal(const Buffer<T>& a, const Buffer<T>& b) {
  check(a.data.size() == b.data.size());
  check(std::memcmp(a.data.data(), b.data.data(), a.data.size() * sizeof(T)) == 0);
}
template <class E = std::invalid_argument, class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const E&) {
    caught = true;
  }
  check(caught);
}
template <class T>
void run(int bits) {
  std::mt19937 rng(935);
  for (int w : {1, 7, 16, 33, 129}) {
    Buffer<T> c(w, 5), r(w, 5), a(w, 5), b(w, 5);
    c.random(rng, bits);
    r.random(rng, bits);
    for (int nr : {2, 6, 50}) {
      std::vector<neo_mv::WeightedReferenceBlock<T>> refs(std::size_t(nr), {true, r.read()});
      refs.back() = {false, {}};
      neo_mv::DegrainWeights weights{256, std::vector<int>(std::size_t(nr))};
      for (int i = 0; i < nr - 1; ++i) {
        weights.reference[i] = 256 / nr;
        weights.centre -= weights.reference[i];
      }
      neo_mv::weighted_render_block(c.read(), refs, weights, a.view(), bits);
      neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), bits);
      equal(a, b);
      weights.reference[0]++;
      rejects([&] { neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), bits); });
    }
    for (double v :
         {0.1, 2.2, 65535.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
      const neo_mv::ChangeLimit<T> limit(v, bits);
      neo_mv::limit_render_plane(limit, r.read(), c.read(), a.view());
      neo_mv::simd::limit_render_plane(limit, r.read(), c.read(), b.view());
      equal(a, b);
    }
  }
  // Four contributors, asymmetric overlap, one-axis overlap, single block, visible crop.
  for (int w : {1, 8, 33})
    for (int mode : {0, 1, 2, 3})
      for (int count : {1, 3}) {
        const int h = 8, ox = (mode & 1) ? w / 2 : 0, oy = (mode & 2) ? 3 : 0;
        const int vw = (count - 1) * (w - ox) + w, vh = (count - 1) * (h - oy) + h;
        const int cw = std::max(1, vw - 2), ch = std::max(1, vh - 1);
        neo_mv::OverlapCompositionPlan plan({w, h, ox, oy, count, count, cw, ch, vw + 7, vh + 3});
        std::vector<Buffer<T>> storage;
        storage.reserve(std::size_t(count) * count);
        std::vector<span2d::Plane<const T>> blocks;
        for (int i = 0; i < count * count; ++i) {
          storage.emplace_back(w, h);
          storage.back().random(rng, bits);
          blocks.push_back(storage.back().read());
        }
        Buffer<T> a(cw, ch), b(cw, ch);
        neo_mv::compose_render_blocks(plan, blocks, a.view(), bits);
        neo_mv::simd::compose_render_blocks(plan, blocks, b.view(), bits);
        equal(a, b);
      }
}
template <class T>
struct EndRow {
  void* memory;
  T* data;
  std::size_t page;
  int width;
  explicit EndRow(int width) : width(width) {
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    page = info.dwPageSize;
#else
    const auto size = sysconf(_SC_PAGESIZE);
    if (size <= 0)
      throw std::runtime_error("cannot query page size");
    page = static_cast<std::size_t>(size);
#endif
    check(width > 0 && std::size_t(width) <= page / sizeof(T));
#if defined(_WIN32)
    memory = VirtualAlloc(nullptr, 2 * page, MEM_RESERVE, PAGE_NOACCESS);
    if (!memory)
      throw std::bad_alloc();
    if (!VirtualAlloc(memory, page, MEM_COMMIT, PAGE_READWRITE)) {
      VirtualFree(memory, 0, MEM_RELEASE);
      throw std::bad_alloc();
    }
#else
    memory = mmap(nullptr, 2 * page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (memory == MAP_FAILED)
      throw std::bad_alloc();
    if (mprotect(memory, page, PROT_READ | PROT_WRITE) != 0) {
      munmap(memory, 2 * page);
      throw std::bad_alloc();
    }
#endif
    data = reinterpret_cast<T*>(static_cast<unsigned char*>(memory) + page - width * sizeof(T));
    for (int x = 0; x < width; ++x)
      ::new (data + x) T(1);
  }
  ~EndRow() {
#if defined(_WIN32)
    VirtualFree(memory, 0, MEM_RELEASE);
#else
    munmap(memory, 2 * page);
#endif
  }
  EndRow(const EndRow&) = delete;
  EndRow& operator=(const EndRow&) = delete;
  span2d::Plane<T> view() { return neo_mv::checked_plane(data, width, 1, width * sizeof(T), width * sizeof(T)); }
  span2d::Plane<const T> read() { return view(); }
};
template <class T>
void guarded_rows(int bits) {
  EndRow<T> c(33), r(33), out(33);
  std::vector<neo_mv::WeightedReferenceBlock<T>> refs{{true, r.read()}, {false, {}}};
  neo_mv::simd::weighted_render_block(c.read(), refs, {128, {128, 0}}, out.view(), bits);
  neo_mv::simd::limit_render_plane(neo_mv::ChangeLimit<T>(1, bits), r.read(), c.read(), out.view());
  neo_mv::OverlapCompositionPlan plan({33, 1, 16, 0, 1, 1, 33, 1, 33, 1});
  std::vector<span2d::Plane<const T>> blocks{c.read()};
  neo_mv::simd::compose_render_blocks(plan, blocks, out.view(), bits);
  for (int x = 0; x < 33; ++x)
    check(out.data[x] == 1.0f);
}
void ordered_and_accumulation() {
  Buffer<float> c(33, 1), p(33, 1), tiny(33, 1), neg(33, 1), a(33, 1), b(33, 1);
  for (int x = 0; x < 33; ++x) {
    c.view().row(0)[x] = 0;
    p.view().row(0)[x] = 0x1p60f;
    tiny.view().row(0)[x] = 1;
    neg.view().row(0)[x] = -0x1p60f;
  }
  std::vector<neo_mv::WeightedReferenceBlock<float>> refs{
      {true, p.read()}, {true, tiny.read()}, {true, neg.read()}, {true, tiny.read()}};
  neo_mv::DegrainWeights weights{0, {64, 64, 64, 64}};
  neo_mv::weighted_render_block(c.read(), refs, weights, a.view(), 32);
  neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), 32);
  equal(a, b);
  check(b.view().row(0)[0] == 0.25f);
  for (int x = 0; x < 33; ++x)
    p.view().row(0)[x] = std::numeric_limits<float>::max() / 128.0f;
  refs = {{true, p.read()}, {true, p.read()}};
  weights = {0, {128, 128}};
  rejects<std::overflow_error>([&] { neo_mv::weighted_render_block(c.read(), refs, weights, a.view(), 32); });
  rejects<std::overflow_error>([&] { neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), 32); });
}
void exceptional() {
  Buffer<float> c(33, 2), r(33, 2), a(33, 2), b(33, 2);
  for (auto* v : {&c, &r})
    for (int y = 0; y < 2; ++y)
      for (int x = 0; x < 33; ++x)
        v->view().row(y)[x] = -0.0f;
  std::vector<neo_mv::WeightedReferenceBlock<float>> refs{{true, r.read()}, {false, {}}};
  const neo_mv::DegrainWeights weights{256, {0, 0}};
  r.view().row(0)[0] = 1;
  neo_mv::weighted_render_block(c.read(), refs, weights, a.view(), 32);
  neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), 32);
  equal(a, b);
  r.view().row(1)[32] = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), 32); });
  r.view().row(1)[32] = 0;
  c.view().row(0)[0] = std::numeric_limits<float>::max();
  rejects<std::overflow_error>([&] { neo_mv::simd::weighted_render_block(c.read(), refs, weights, b.view(), 32); });
  rejects<std::overflow_error>([&] {
    neo_mv::simd::limit_render_plane(neo_mv::ChangeLimit<float>(std::numeric_limits<float>::max(), 32), r.read(),
                                     c.read(), b.view());
  });
  rejects([&] { neo_mv::simd::limit_render_plane(neo_mv::ChangeLimit<float>(1, 32), r.read(), c.read(), c.view()); });
  c.view().row(0)[0] = -0.0f;
  neo_mv::limit_render_plane(neo_mv::ChangeLimit<float>(1, 32), c.read(), r.read(), a.view());
  neo_mv::simd::limit_render_plane(neo_mv::ChangeLimit<float>(1, 32), c.read(), r.read(), b.view());
  equal(a, b);
  neo_mv::OverlapCompositionPlan plan({33, 2, 16, 1, 1, 1, 1, 1, 33, 2});
  c.view().row(1)[32] = std::numeric_limits<float>::quiet_NaN();
  std::vector<span2d::Plane<const float>> blocks{c.read()};
  Buffer<float> out(1, 1);
  rejects([&] { neo_mv::simd::compose_render_blocks(plan, blocks, out.view(), 32); });
  c.view().row(1)[32] = 0;
  c.view().row(0)[0] = std::numeric_limits<float>::max();
  rejects<std::overflow_error>([&] { neo_mv::simd::compose_render_blocks(plan, blocks, out.view(), 32); });
}
template <class T>
void fused_degrain(int bits) {
  using namespace neo_mv;
  std::mt19937 rng(12039);
  for (int w : {1, 4, 7, 8, 16, 17, 32, 33})
    for (int overlap : {0, 1, 2, 3})
      for (int nr : {2, 6, 50}) {
        const int ox = (overlap & 1) ? w / 2 : 0, oy = (overlap & 2) ? 2 : 0;
        const int width = 2 * w - ox, height = 10 - oy;
        OverlapCompositionPlan plan({w, 5, ox, oy, 2, 2, width - 1, height - 1, width, height});
        DegrainPlane<T> p;
        p.references = nr;
        std::vector<Buffer<T>> inputs, generated;
        inputs.reserve(8);
        generated.reserve(4);
        std::vector<span2d::Plane<const T>> views;
        for (int i = 0; i < 4; ++i) {
          inputs.emplace_back(w, 5);
          inputs.back().random(rng, bits);
          inputs.emplace_back(w, 5);
          inputs.back().random(rng, bits);
          auto c = inputs[2 * i].read(), r = inputs[2 * i + 1].read();
          const auto* coeff = plan.has_overlap() ? plan.coefficient_row(i % 2, i / 2, 0) : nullptr;
          p.sources.push_back({c.row(0).data(), c.stride(), coeff});
          DegrainWeights weights{256, std::vector<int>(nr)};
          std::vector<WeightedReferenceBlock<T>> refs(nr, {true, r});
          refs.back() = {false, {}};
          for (int j = 0; j < nr; ++j) {
            weights.reference[j] = (j == 0 || j == nr - 1) ? 0 : 256 / nr;
            weights.centre -= weights.reference[j];
            p.sources.push_back({j == nr - 1 ? nullptr : r.row(0).data(), r.stride(), coeff});
          }
          p.weights.push_back(weights.centre);
          p.weights.insert(p.weights.end(), weights.reference.begin(), weights.reference.end());
          generated.emplace_back(w, 5);
          weighted_render_block(c, refs, weights, generated.back().view(), bits);
          views.push_back(generated.back().read());
        }
        Buffer<T> composed(width - 1, height - 1), centre(width - 1, height - 1), expected(width - 1, height - 1),
            scalar(width - 1, height - 1), highway(width - 1, height - 1);
        centre.random(rng, bits);
        compose_render_blocks(plan, views, composed.view(), bits);
        for (double amount : {2.2, std::numeric_limits<double>::infinity()}) {
          ChangeLimit<T> limit(amount, bits);
          limit_render_plane(limit, composed.read(), centre.read(), expected.view());
          ScalarRenderKernels<T>::compose_degrain(plan, p, centre.read(), scalar.view(), limit);
          HighwayRenderKernels<T>::compose_degrain(plan, p, centre.read(), highway.view(), limit);
          equal(expected, scalar);
          equal(expected, highway);
        }
        // Invalid cropped sample in an available reference with zero weight.
        if constexpr (std::is_same_v<T, float> || std::is_same_v<T, std::uint16_t>) {
          if (bits != 16) {
            if constexpr (std::is_same_v<T, float>)
              inputs.back().view().row(4)[w - 1] = std::numeric_limits<float>::quiet_NaN();
            else
              inputs.back().view().row(4)[w - 1] = T(1u << bits);
            const ChangeLimit<T> limit(std::numeric_limits<double>::infinity(), bits);
            rejects([&] { ScalarRenderKernels<T>::compose_degrain(plan, p, centre.read(), scalar.view(), limit); });
            rejects([&] { HighwayRenderKernels<T>::compose_degrain(plan, p, centre.read(), highway.view(), limit); });
          }
        }
      }
}
} // namespace
int main() {
  try {
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      fused_degrain<std::uint8_t>(8);
      fused_degrain<std::uint16_t>(10);
      fused_degrain<std::uint16_t>(16);
      fused_degrain<float>(32);
    }
    hwy::SetSupportedTargetsForTest(0);
    run<std::uint8_t>(8);
    run<std::uint16_t>(10);
    run<std::uint16_t>(16);
    run<float>(32);
    exceptional();
    ordered_and_accumulation();
    guarded_rows<std::uint8_t>(8);
    guarded_rows<std::uint16_t>(16);
    guarded_rows<float>(32);
    std::cout << "Highway target: " << neo_mv::simd::detail::target_name() << '\n';
    std::cout << "Render composition, weighted samples and limits passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
