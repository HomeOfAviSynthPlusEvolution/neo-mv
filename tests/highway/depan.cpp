#include "highway/depan.hpp"
#include "highway/rows.hpp"
#include "hwy/targets.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <random>
#include <string>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace {
using namespace neo_mv;
using namespace neo_mv::depan;
constexpr int widths[] = {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129};
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Depan differential assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
void same_float(float a, float b) {
  CHECK(std::memcmp(&a, &b, sizeof(float)) == 0);
}
void same_map(Transform a, Transform b) {
  same_float(a.tx, b.tx);
  same_float(a.ty, b.ty);
  same_float(a.u, b.u);
  same_float(a.v, b.v);
  same_float(a.w, b.w);
  same_float(a.h, b.h);
}
template <class T>
void same_storage(const std::vector<T>& a, const std::vector<T>& b) {
  CHECK(a.size() == b.size());
  CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}
template <class F>
bool rejected(F&& call) {
  try {
    call();
    return false;
  } catch (const std::invalid_argument&) {
    return true;
  } catch (const std::overflow_error&) {
    return true;
  }
}
template <class T>
struct Buffer {
  int width, height, stride;
  std::vector<T> storage;
  Buffer(int w, int h) : width(w), height(h), stride(w + 5), storage(std::size_t(stride) * h + 2, T(173)) {}
  span2d::Plane<T> view() {
    return checked_plane(storage.data() + 1, width, height, std::ptrdiff_t(stride) * sizeof(T),
                         (storage.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
};

template <class T>
void sampling(int bits) {
  std::mt19937 random(58139);
  const auto maximum = (1u << bits) - 1;
  const std::array<Transform, 7> maps{{{},
                                       {-0.5001f, 0.99999994f, 1, 0, 0, 1},
                                       {0.99609375f, 0.03125f, 1, 0, 0, 1},
                                       {-2.25f, -1.25f, 0.875f, 0, 0, 1.125f},
                                       {1.001f, -0.001f, 1.03125f, -0.125f, 0.0625f, 0.96875f},
                                       {-12.5f, 0.5f, 1, 0, 0, 1},
                                       {12.5f, 0.5f, 1, 0, 0, 1}}};
  for (const int width : widths) {
    Buffer<T> input(width, 7), scalar(width, 7), highway(width, 7);
    for (int pattern = 0; pattern < 2; ++pattern) {
      for (int y = 0; y < 7; ++y)
        for (int x = 0; x < width; ++x)
          input.view().row(y)[x] = T(pattern ? ((x + y) % 3 == 0 ? maximum : 0) : random() & maximum);
      const auto before = input.storage;
      for (const int mode : {0, 1, 2})
        for (const auto map : maps)
          for (const int mirror : {0, 1, 2, 4, 8, 15})
            for (const int blur : {0, 3}) {
              SamplingPlan(width, 7, bits, mode, mirror, blur, int(maximum / 2), map)
                  .render(input.read(), scalar.view());
              HighwaySamplingPlan(width, 7, bits, mode, mirror, blur, int(maximum / 2), map)
                  .render(input.read(), highway.view());
              same_storage(scalar.storage, highway.storage);
            }
      same_storage(before, input.storage);
    }
  }
  // Large finite scale/shear coordinates remain outside, without unsafe integer casts.
  for (const auto map : {Transform{-1e30f, 1e30f, 1, 0, 0, 1}, Transform{0, 0, -1e30f, 0, 0, 1e30f},
                         Transform{0, 0, 1, 1e30f, -1e30f, 1}})
    for (const int mode : {0, 1, 2}) {
      Buffer<T> input(17, 7), scalar(17, 7), highway(17, 7);
      for (int y = 0; y < 7; ++y)
        for (int x = 0; x < 17; ++x)
          input.view().row(y)[x] = T((x * 137 + y * 73) & maximum);
      SamplingPlan(17, 7, bits, mode, 15, INT32_MAX, 0, map).render(input.read(), scalar.view());
      HighwaySamplingPlan(17, 7, bits, mode, 15, INT32_MAX, 0, map).render(input.read(), highway.view());
      same_storage(scalar.storage, highway.storage);
    }
  for (int width = 1; width <= 4; ++width)
    for (int height = 1; height <= 4; ++height)
      for (const int mode : {0, 1, 2}) {
        Buffer<T> input(width, height), scalar(width, height), highway(width, height);
        const auto a = rejected([&] {
          SamplingPlan(width, height, bits, mode, 15, 3, 0, {-1.5f, -0.5f}).render(input.read(), scalar.view());
        });
        const auto b = rejected([&] {
          HighwaySamplingPlan(width, height, bits, mode, 15, 3, 0, {-1.5f, -0.5f}).render(input.read(), highway.view());
        });
        CHECK(a == b);
        same_storage(scalar.storage, highway.storage);
      }
  if constexpr (std::is_same_v<T, std::uint16_t>) {
    if (bits < 16) {
      Buffer<T> input(17, 7), output(17, 7);
      input.view().row(6)[16] = T(1u << bits);
      const auto before = output.storage;
      CHECK(rejected([&] { HighwaySamplingPlan(17, 7, bits, 2, 0, 0, 0, {}).render(input.read(), output.view()); }));
      same_storage(before, output.storage);
    }
  }
}

void coordinates() {
  const Transform maps[] = {{},
                            {-0.5f, 0.5f},
                            {0.99999994f, -0.0000001f, 0.875f, 0, 0, 1.125f},
                            {1.001f, -0.001f, 1.03125f, -0.125f, 0.0625f, 0.96875f},
                            {0, 0, -0.0f, 0, 0, -0.0f},
                            {0, 0, -1e30f, 0, 0, 1e30f},
                            {0, 0, 1, 1e30f, -1e30f, 1},
                            {0, 0, std::numeric_limits<float>::max(), 0, 0, 1}};
  for (int width : widths)
    for (int mode : {0, 1, 2})
      for (auto map : maps) {
        SamplingPlan plan(width, INT32_MAX, 16, mode, 0, 0, 0, map);
        for (int y : {0, 1, 6, 16777217, INT32_MAX - 1}) {
          std::vector<SamplingCoordinates> a(width), b(width);
          const bool ea = rejected([&] { plan.row_coordinates(y, [&](int x, SamplingCoordinates q) { a[x] = q; }); });
          const bool eb = rejected([&] { simd::depan_rows::coordinates(plan, y, b.data()); });
          CHECK(ea == eb);
          if (ea)
            continue;
          for (int x = 0; x < width; ++x) {
            CHECK(std::memcmp(&a[x].i, &b[x].i, sizeof(double)) == 0);
            CHECK(std::memcmp(&a[x].j, &b[x].j, sizeof(double)) == 0);
            same_float(a[x].fx, b[x].fx);
            same_float(a[x].fy, b[x].fy);
          }
        }
      }
}

Observations observations(int nx, int ny) {
  Observations result{nx, ny, true, true, 400, {}};
  for (int y = 0; y < ny; ++y)
    for (int x = 0; x < nx; ++x)
      result.values.push_back({8 * x + 4, 8 * y + 4, 0, 0, 0, 1});
  return result;
}
// Independent correctly-rounded std::fma oracle for the permitted fusion sites.
// All other arithmetic, accumulation order and integer sampling stay unchanged.
float fused_add(float a, float b, float c) {
  (void)mul(a, b);
  return finite(std::fma(a, b, finite(c)));
}
// Reference reduction spells out all ten sums independently of accumulate_fit.
template <class Residuals>
FitSums oracle_sums(const Observations& o, const std::vector<float>& weights, Residuals&& errors, bool zoom,
                    bool rotation) {
  FitSums sums;
  const auto madd = [](float a, float b, float c) {
    return simd::depan_rows::native_fma() ? fused_add(a, b, c) : add(c, mul(a, b));
  };
  for (std::size_t i = 0; i < o.values.size(); ++i) {
    const auto e = errors(i);
    const float w = f32(weights[i]);
    const auto x = static_cast<std::uint64_t>(o.values[i].x);
    const auto y = static_cast<std::uint64_t>(o.values[i].y);
    sums.n = add(sums.n, w);
    sums.x2 = madd(analysis_detail::square32(x), w, sums.x2);
    sums.y2 = madd(analysis_detail::square32(y), w, sums.y2);
    sums.residual = madd(add(mul(e[0], e[0]), mul(e[1], e[1])), w, sums.residual);
    sums.gx = madd(mul(2, e[0]), w, sums.gx);
    sums.gy = madd(mul(2, e[1]), w, sums.gy);
    if (zoom) {
      sums.gxx = madd(mul(analysis_detail::integer32(2 * x), e[0]), w, sums.gxx);
      sums.gyy = madd(mul(analysis_detail::integer32(2 * y), e[1]), w, sums.gyy);
    }
    if (rotation) {
      sums.gxy = madd(mul(analysis_detail::integer32(2 * y), e[0]), w, sums.gxy);
      sums.gyx = madd(mul(analysis_detail::integer32(2 * x), e[1]), w, sums.gyx);
    }
  }
  return sums;
}
struct FmaOracle : ScalarResiduals {
  static auto prepare(const Observations& o, Transform t) {
    if (!simd::depan_rows::native_fma()) {
      std::vector<std::array<float, 2>> result;
      for (const auto& v : o.values)
        result.push_back({analysis_detail::residual_x(v, t), analysis_detail::residual_y(v, t)});
      return result;
    }
    std::vector<std::array<float, 2>> result;
    for (const auto& v : o.values) {
      const float x = analysis_detail::integer32(static_cast<std::uint64_t>(v.x));
      const float y = analysis_detail::integer32(static_cast<std::uint64_t>(v.y));
      result.push_back({sub(sub(fused_add(t.v, y, fused_add(t.u, x, t.tx)), x), v.dx),
                        sub(sub(fused_add(t.h, y, fused_add(t.w, x, t.ty)), y), v.dy)});
    }
    return result;
  }
  static std::array<float, 4> adjust(std::array<float, 4> values, const std::array<float, 4>& scales,
                                     const std::array<float, 4>& gradients, std::size_t count) {
    if (!simd::depan_rows::native_fma())
      return ScalarResiduals::adjust(values, scales, gradients, count);
    for (std::size_t i = 0; i < count; ++i)
      values[i] = fused_add(-scales[i], gradients[i], values[i]);
    return values;
  }
};
struct OracleResiduals : FmaOracle {
  static FitSums accumulate(const Observations& o, const std::vector<float>& weights, Transform map, bool zoom,
                            bool rotation) {
    const auto rows = FmaOracle::prepare(o, map);
    return oracle_sums(o, weights, [&rows](std::size_t i) { return rows[i]; }, zoom, rotation);
  }
  static auto prepare(const Observations& o, Transform t) {
    return [rows = FmaOracle::prepare(o, t)](std::size_t i) {
      return rows[i];
    };
  }
};
void close_float(float a, float b) {
  CHECK(std::isfinite(a) && std::isfinite(b));
  CHECK(std::abs(double(a) - double(b)) <= 1e-4 + 1e-5 * (std::max)(std::abs(double(a)), std::abs(double(b))));
}
void update_pair(const Observations& o, const std::vector<float>& weights, Transform map, float aspect, float step,
                 bool zoom, bool rotation) {
  std::optional<FitUpdate> a, b;
  CHECK(rejected([&] { a = fit_update<OracleResiduals>(o, weights, map, aspect, step, zoom, rotation); }) ==
        rejected([&] { b = fit_update<HighwayResiduals>(o, weights, map, aspect, step, zoom, rotation); }));
  CHECK(a.has_value() == b.has_value());
  if (a) {
    same_map(a->map, b->map);
    same_float(a->error, b->error);
  }
}
void fit_pair(const Observations& o, FitParameters p = {}, bool bounded = false) {
  std::optional<FitResult> a, b;
  CHECK(rejected([&] { a = fit<OracleResiduals>(o, p); }) == rejected([&] { b = fit<HighwayResiduals>(o, p); }));
  CHECK(a.has_value() == b.has_value());
  if (a) {
    same_map(a->map, b->map);
    same_float(a->error, b->error);
    CHECK(a->iteration == b->iteration && a->good == b->good);
    if (bounded) {
      const auto scalar = fit(o, p);
      close_float(scalar.map.tx, b->map.tx);
      close_float(scalar.map.ty, b->map.ty);
      close_float(scalar.map.u, b->map.u);
      close_float(scalar.map.v, b->map.v);
      close_float(scalar.map.w, b->map.w);
      close_float(scalar.map.h, b->map.h);
      close_float(scalar.error, b->error);
      CHECK(scalar.good == b->good && scalar.iteration == b->iteration);
    }
  }
}
void fitting() {
  std::mt19937 random(9217);
  for (const int nx : {1, 3, 8, 9, 17, 33}) {
    auto o = observations(nx, 3);
    std::vector<float> weights(o.values.size());
    for (std::size_t i = 0; i < o.values.size(); ++i) {
      o.values[i].dx = float(int(random() % 65) - 32) * 0.125f;
      o.values[i].dy = float(int(random() % 65) - 32) * 0.125f;
      o.values[i].sad = random() % 500;
      o.values[i].base = float(random() % 256);
      weights[i] = float(int(random() % 17) - 4) * 0.03125f;
    }
    for (const bool zoom : {false, true})
      for (const bool rotation : {false, true}) {
        update_pair(o, weights, {0.5f, -0.25f, 1.001f, -0.0001f, 0.0002f, 0.999f}, 1.25f, 0.3f, zoom, rotation);
        FitParameters p;
        p.zoom = zoom;
        p.rotation = rotation;
        p.aspect = 1.25f;
        fit_pair(o, p, true);
      }
    o.masked = false;
    fit_pair(o);
  }
  auto golden = observations(2, 2);
  const std::array<std::array<float, 2>, 4> motion{{{0.5f, -1.5f}, {0, 0}, {2, 1}, {-1, 0.5f}}};
  for (std::size_t i = 0; i < motion.size(); ++i) {
    golden.values[i].dx = motion[i][0];
    golden.values[i].dy = motion[i][1];
  }
  FitParameters p;
  p.zoom = p.rotation = false;
  fit_pair(golden, p, true);
  const auto result = fit<HighwayResiduals>(golden, p);
  CHECK(result.good && result.iteration == 10);
  // The final good-motion comparison stays strict even next to its threshold.
  for (float threshold : {std::nextafter(result.error, 0.0f), result.error,
                          std::nextafter(result.error, std::numeric_limits<float>::infinity())}) {
    auto boundary = p;
    boundary.error = threshold;
    fit_pair(golden, boundary);
    const auto actual = fit<HighwayResiduals>(golden, boundary);
    CHECK(actual.good == (result.error < threshold));
    CHECK(actual.iteration == result.iteration);
  }
  if (!simd::depan_rows::native_fma()) {
    same_float(result.map.tx, 0.49161040782928467f);
    same_float(result.error, 1.6047950983047485f);
  }
  const float tiny = std::numeric_limits<float>::denorm_min();
  for (const float value : {0.0f, -0.0f, tiny, -tiny, 0x1p24f, -0x1p24f, std::numeric_limits<float>::max()}) {
    auto o = observations(17, 1);
    for (auto& v : o.values) {
      v.dx = value;
      v.dy = -value;
    }
    update_pair(o, std::vector<float>(17, 0), {-0.0f, -0.0f}, 1, 0.3f, true, true);
    fit_pair(o);
  }
  auto single = observations(1, 1);
  for (const float weight : {-1.0f, -0.1f, -0.0f, tiny, std::numeric_limits<float>::infinity()})
    update_pair(single, {weight}, {}, 1, 0.3f, false, false);
  single.values[0].x = INT64_MAX;
  single.values[0].y = (std::int64_t{1} << 54) + (std::int64_t{1} << 30) + 1;
  update_pair(single, {0}, {}, 1, 0.3f, true, true);
  p.zerow = -1;
  fit_pair(observations(1, 1), p);
  for (const float error : {-1.0f, 0.0f, 15.0f}) {
    p.error = error;
    fit_pair({}, p);
  }
}

// Place the final element against an inaccessible page; every row load/store,
// including a tail with fewer than one SIMD vector, must respect count.
template <class T>
struct EndRow {
  void* memory = nullptr;
  T* data;
  std::size_t page, committed;
  explicit EndRow(std::size_t count) {
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    page = info.dwPageSize;
#else
    const auto size = sysconf(_SC_PAGESIZE);
    CHECK(size > 0);
    page = static_cast<std::size_t>(size);
#endif
    committed = ((count * sizeof(T) + page - 1) / page) * page;
#if defined(_WIN32)
    memory = VirtualAlloc(nullptr, committed + page, MEM_RESERVE, PAGE_NOACCESS);
    if (!memory)
      throw std::bad_alloc();
    if (!VirtualAlloc(memory, committed, MEM_COMMIT, PAGE_READWRITE)) {
      VirtualFree(memory, 0, MEM_RELEASE);
      throw std::bad_alloc();
    }
#else
    memory = mmap(nullptr, committed + page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (memory == MAP_FAILED)
      throw std::bad_alloc();
    if (mprotect(memory, committed, PROT_READ | PROT_WRITE) != 0) {
      munmap(memory, committed + page);
      throw std::bad_alloc();
    }
#endif
    data = reinterpret_cast<T*>(static_cast<unsigned char*>(memory) + committed - count * sizeof(T));
    for (std::size_t i = 0; i < count; ++i)
      ::new (data + i) T{};
  }
  ~EndRow() {
#if defined(_WIN32)
    VirtualFree(memory, 0, MEM_RELEASE);
#else
    munmap(memory, committed + page);
#endif
  }
  EndRow(const EndRow&) = delete;
  EndRow& operator=(const EndRow&) = delete;
};
void guarded_weighted() {
  std::mt19937 random(94711);
  for (const int width : widths)
    for (const int taps : {4, 16})
      for (const int padding : {0, 3, 17}) {
        const auto stride = std::size_t(width + padding);
        EndRow<std::int64_t> samples(stride * (taps - 1) + width), weights(stride * (taps - 1) + width), out(width);
        std::fill_n(samples.data, stride * (taps - 1) + width, 65535);
        std::fill_n(weights.data, stride * (taps - 1) + width, 4194304);
        for (int k = 0; k < taps; ++k)
          for (int i = 0; i < width; ++i) {
            const auto pos = std::size_t(k) * stride + i;
            samples.data[pos] = i % 3 == 0 ? 65535 : random() % 65536;
            weights.data[pos] = i % 3 == 0 ? (i % 2 ? -4194304 : 4194304) : int(random() % 8388609) - 4194304;
          }
        for (const int shift : {10, 11, 22})
          for (const bool round : {false, true}) {
            if (round && shift != 11)
              continue;
            for (const std::int64_t maximum : {255, 1023, 65535}) {
              simd::depan_rows::weighted(samples.data, weights.data, width, taps, shift, round, maximum, out.data,
                                         padding ? stride : 0);
              for (int i = 0; i < width; ++i) {
                std::int64_t total = round ? 1024 : 0;
                for (int k = 0; k < taps; ++k)
                  total += samples.data[std::size_t(k) * stride + i] * weights.data[std::size_t(k) * stride + i];
                const auto denominator = std::int64_t{1} << shift;
                const auto divided = total >= 0 ? total / denominator : -1 - ((-1 - total) / denominator);
                CHECK(out.data[i] == std::clamp(divided, std::int64_t{0}, maximum));
              }
            }
          }
      }
  for (const int shift : {10, 11, 22}) {
    constexpr int count = 17;
    EndRow<std::int64_t> samples(count * 4), weights(count * 4), out(count);
    const auto denominator = std::int64_t{1} << shift;
    const std::array<std::int64_t, count> totals{-denominator - 1,
                                                 -denominator,
                                                 -denominator + 1,
                                                 -1025,
                                                 -1024,
                                                 -1023,
                                                 -1,
                                                 0,
                                                 1,
                                                 denominator / 2 - 1,
                                                 denominator / 2,
                                                 denominator / 2 + 1,
                                                 denominator - 1,
                                                 denominator,
                                                 denominator + 1,
                                                 65535 * denominator - 1,
                                                 65535 * denominator + 1};
    for (int i = 0; i < count; ++i) {
      samples.data[i] = 1;
      weights.data[i] = totals[i];
    }
    for (const bool round : {false, true}) {
      if (round && shift != 11)
        continue;
      simd::depan_rows::weighted(samples.data, weights.data, count, 4, shift, round, 65535, out.data);
      for (int i = 0; i < count; ++i) {
        const auto total = totals[i] + (round ? 1024 : 0);
        const auto divided = total >= 0 ? total / denominator : -1 - ((-1 - total) / denominator);
        CHECK(out.data[i] == std::clamp(divided, std::int64_t{0}, std::int64_t{65535}));
      }
    }
  }
}

template <class T>
void guarded_linear() {
  for (int width : widths) {
    EndRow<T> source(width * 2), output(width);
    EndRow<SamplingCoordinates> coords(width);
    auto input = checked_plane(static_cast<const T*>(source.data), width, 2, std::ptrdiff_t(width) * sizeof(T),
                               std::size_t(width) * 2 * sizeof(T));
    for (int i = 0; i < width * 2; ++i)
      source.data[i] = (i & 1) ? std::numeric_limits<T>::max() : T(0);
    for (int i = 0; i < width; ++i)
      coords.data[i] = {i % 3 ? double(width - 2) : -2.0, 0,
                        i % 2 ? std::nextafter(1.0f, 0.0f) : std::numeric_limits<float>::denorm_min(), 0.96875f};
    const SamplingPlan plan(width, 2, sizeof(T) * 8, 1, 0, 0, 7, {});
    for (bool preserve : {false, true}) {
      for (int i = 0; i < width; ++i)
        output.data[i] = T(173);
      std::vector<T> expected(width, T(173));
      for (int i = 0; i < width; ++i)
        plan.write_sample(input, expected[i], coords.data[i], preserve);
      simd::depan_rows::linear_row(plan, input, output.data, coords.data, preserve);
      CHECK(std::equal(expected.begin(), expected.end(), output.data));
    }
    EndRow<T> fused(width * 2);
    auto destination =
        checked_plane(fused.data, width, 2, std::ptrdiff_t(width) * sizeof(T), std::size_t(width) * 2 * sizeof(T));
    for (const auto map : {Transform{0.25f, 0.03125f, 1, 0.125f, -0.03125f, 1},
                           Transform{-1.5f, -0.96875f, 0.75f, 0.25f, 0.03125f, 0.75f}}) {
      const SamplingPlan reference(width, 2, sizeof(T) * 8, 1, 0, 0, 7, map);
      for (bool preserve : {false, true}) {
        std::fill_n(fused.data, width * 2, T(173));
        std::vector<T> expected(width * 2, T(173));
        for (int y = 0; y < 2; ++y)
          reference.row_coordinates(y, [&](int x, SamplingCoordinates q) {
            reference.write_sample(input, expected[std::size_t(y) * width + x], q, preserve);
          });
        simd::depan_rows::linear_render(reference, input, destination, preserve);
        CHECK(std::equal(expected.begin(), expected.end(), fused.data));
      }
    }
  }
}
void guarded_residuals() {
  const float tiny = std::numeric_limits<float>::denorm_min();
  const std::array<float, 8> values{0.0f, -0.0f, tiny, -tiny, 0.125f, -3.375f, 0x1p24f, -0x1p24f};
  for (const int width : widths) {
    EndRow<float> x(width), y(width), dx(width), dy(width), ex(width), ey(width);
    for (int i = 0; i < width; ++i) {
      x.data[i] = float(i * 8 + 4);
      y.data[i] = float(i % 7 * 8 + 4);
      dx.data[i] = values[std::size_t(i) % values.size()];
      dy.data[i] = values[(std::size_t(i) + 1) % values.size()];
    }
    for (const auto map :
         {Transform{}, Transform{-0.0f, -0.0f}, Transform{0.137f, -1.3f, 1.001f, 0.15f, -0.17f, 0.98f}}) {
      simd::depan_rows::residuals(x.data, y.data, dx.data, dy.data, width, map, ex.data, ey.data);
      for (int i = 0; i < width; ++i) {
        const Observation value{std::int64_t(x.data[i]), std::int64_t(y.data[i]), dx.data[i], dy.data[i], 0, 1};
        Observations single{1, 1, true, false, 400, {value}};
        const auto expected = FmaOracle::prepare(single, map)[0];
        same_float(ex.data[i], expected[0]);
        same_float(ey.data[i], expected[1]);
      }
    }
    for (const float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
      dx.data[width - 1] = invalid;
      CHECK(rejected(
          [&] { simd::depan_rows::residuals(x.data, y.data, dx.data, dy.data, width, {}, ex.data, ey.data); }));
    }
    dx.data[width - 1] = 0;
    Transform overflow;
    overflow.u = std::numeric_limits<float>::max();
    CHECK(rejected(
        [&] { simd::depan_rows::residuals(x.data, y.data, dx.data, dy.data, width, overflow, ex.data, ey.data); }));
  }
}
void guarded_accumulation() {
  std::mt19937 random(58401);
  for (int width : widths) {
    auto o = observations(width, 1);
    EndRow<float> ex(width), ey(width);
    std::vector<float> weights(width);
    for (int i = 0; i < width; ++i) {
      ex.data[i] = float(int(random() % 8193) - 4096) / 37.0f;
      ey.data[i] = float(int(random() % 8193) - 4096) / 53.0f;
      weights[i] = float(int(random() % 1025) - 512) / 17.0f;
    }
    for (bool zoom : {false, true})
      for (bool rotation : {false, true}) {
        const auto expected = oracle_sums(
            o, weights, [&](std::size_t i) { return std::array<float, 2>{ex.data[i], ey.data[i]}; }, zoom, rotation);
        const auto actual = simd::depan_rows::accumulate(o, weights, ex.data, ey.data, zoom, rotation);
        const std::array<float, 10> a{expected.n,  expected.x2,  expected.y2,  expected.residual, expected.gx,
                                      expected.gy, expected.gxx, expected.gyy, expected.gxy,      expected.gyx};
        const std::array<float, 10> b{actual.n,  actual.x2,  actual.y2,  actual.residual, actual.gx,
                                      actual.gy, actual.gxx, actual.gyy, actual.gxy,      actual.gyx};
        CHECK(std::memcmp(a.data(), b.data(), sizeof(a)) == 0);
      }
    for (float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
      ex.data[width - 1] = invalid;
      CHECK(rejected([&] { simd::depan_rows::accumulate(o, weights, ex.data, ey.data, true, true); }));
    }
  }
  auto o = observations(2, 1);
  const float ex[] = {-0.5f, 0x1.000002p-1f}, ey[] = {0, 0};
  const auto sums = simd::depan_rows::accumulate(o, {1, 0x1.fffffcp-1f}, ex, ey, true, true);
  same_float(sums.gx, simd::depan_rows::native_fma() ? -0x1p-46f : 0.0f);
  // A finite fused cancellation must not hide an overflowing weighted product.
  for (auto& value : o.values)
    value.x = INT64_MAX;
  const float zeros[] = {0, 0};
  CHECK(rejected([&] { simd::depan_rows::accumulate(o, {-3, 5}, zeros, zeros, false, false); }));
}
void guarded_adjustments() {
  for (const int width : widths) {
    EndRow<float> values(width), scales(width), gradients(width), output(width);
    for (int i = 0; i < width; ++i) {
      values.data[i] = i % 2 ? -0.0f : 1.0f;
      scales.data[i] = i % 2 ? 0.0f : 0x1.000002p0f;
      gradients.data[i] = i % 2 ? -0.0f : 0x1.fffffcp-1f;
    }
    simd::depan_rows::adjust(values.data, scales.data, gradients.data, width, output.data);
    for (int i = 0; i < width; ++i) {
      const float expected = simd::depan_rows::native_fma()
                                 ? fused_add(-scales.data[i], gradients.data[i], values.data[i])
                                 : sub(values.data[i], mul(scales.data[i], gradients.data[i]));
      same_float(output.data[i], expected);
    }
    CHECK(output.data[0] == (simd::depan_rows::native_fma() ? 0x1p-46f : 0.0f));
    simd::depan_rows::adjust(values.data, scales.data, gradients.data, width, values.data);
    CHECK(std::memcmp(values.data, output.data, std::size_t(width) * sizeof(float)) == 0);
    // A fused cancellation must not hide a non-finite separate product.
    values.data[width - 1] = std::numeric_limits<float>::max();
    scales.data[width - 1] = std::numeric_limits<float>::max();
    gradients.data[width - 1] = 2;
    CHECK(rejected([&] { simd::depan_rows::adjust(values.data, scales.data, gradients.data, width, output.data); }));
  }
}
} // namespace

int main() {
  try {
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << "Testing " << hwy::TargetName(target) << '\n';
      CHECK(std::strcmp(simd::detail::target_name(), hwy::TargetName(target)) == 0);
      coordinates();
      sampling<std::uint8_t>(8);
      sampling<std::uint16_t>(10);
      sampling<std::uint16_t>(16);
      fitting();
      guarded_weighted();
      guarded_linear<std::uint8_t>();
      guarded_linear<std::uint16_t>();
      guarded_residuals();
      guarded_adjustments();
      guarded_accumulation();
    }
    hwy::SetSupportedTargetsForTest(0);
    std::cout << "Depan exact differentials passed\n";
    return 0;
  } catch (const std::exception& error) {
    hwy::SetSupportedTargetsForTest(0);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
