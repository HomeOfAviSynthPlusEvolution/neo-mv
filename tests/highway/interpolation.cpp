#include "highway/interpolation.hpp"
#include "highway/rows.hpp"
#include "hwy/targets.h"

#include <cstring>
#include <iostream>
#include <new>
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

namespace {
using namespace neo_mv;
constexpr int widths[] = {1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129};
constexpr int times[] = {0, 1, 85, 128, 255, 256};
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("interpolation scalar/Highway mismatch at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class T>
void same(const std::vector<T>& first, const std::vector<T>& second) {
  CHECK(first.size() == second.size());
  CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(T)) == 0);
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
  std::vector<T> data;
  Buffer(int w, int h, int gap = 3) : width(w), height(h), stride(w + gap), data(std::size_t(stride) * h + 1, T(19)) {}
  span2d::Plane<T> view() {
    return checked_plane(data.data() + 1, width, height, std::ptrdiff_t(stride) * sizeof(T),
                         (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
};
template <class T>
T random_sample(std::mt19937& rng, int bits) {
  if constexpr (std::is_same_v<T, float>)
    return float(int(rng() % 200001) - 100000) / 719.37f;
  else
    return T(rng() & ((1u << bits) - 1));
}
template <class T>
void compositions(int bits) {
  std::mt19937 rng(0x1A43029);
  constexpr std::uint8_t masks[] = {0, 1, 127, 255};
  for (const int width : widths) {
    const std::size_t count = std::size_t(width) * 3;
    std::vector<InterpolationSamples<T>> samples(count);
    std::vector<std::uint8_t> mF(count), mB(count);
    for (int pattern = 0; pattern < 3; ++pattern) {
      for (std::size_t i = 0; i < count; ++i) {
        auto& s = samples[i];
        if (pattern == 0) {
          s = {random_sample<T>(rng, bits), random_sample<T>(rng, bits), random_sample<T>(rng, bits),
               random_sample<T>(rng, bits), random_sample<T>(rng, bits), random_sample<T>(rng, bits)};
          mF[i] = masks[i % 4];
          mB[i] = masks[(i / 4) % 4];
        } else {
          T maximum;
          if constexpr (std::is_same_v<T, float>)
            maximum = 0x1p100f;
          else
            maximum = T((1u << bits) - 1);
          const T value = pattern == 1 ? maximum : T(0);
          s = {value, value, value, value, value, value};
          mF[i] = mB[i] = 255; // Maximum 16-bit inner numerator, including H's bias.
        }
      }
      for (const int time : times)
        for (const bool extra : {false, true}) {
          Buffer<T> scalar(width, 3), highway(width, 3);
          ScalarInterpolationKernels<T>::compose(samples, mF, mB, extra, time, bits, scalar.view());
          HighwayInterpolationKernels<T>::compose(samples, mF, mB, extra, time, bits, highway.view());
          same(scalar.data, highway.data); // Include prefix and every row gap.
          if (pattern != 0 && !std::is_same_v<T, float>)
            for (int y = 0; y < 3; ++y)
              for (int x = 0; x < width; ++x)
                CHECK(highway.view().row(y)[x] == samples[0].A);
        }
    }
    Buffer<T> first(width, 3, 1), second(width, 3, 5);
    for (int y = 0; y < 3; ++y)
      for (int x = 0; x < width; ++x) {
        first.view().row(y)[x] = random_sample<T>(rng, bits);
        second.view().row(y)[x] = random_sample<T>(rng, bits);
      }
    for (const int time : times) {
      Buffer<T> scalar(width, 3), highway(width, 3);
      ScalarInterpolationKernels<T>::blend(first.read(), second.read(), scalar.view(), time, bits);
      HighwayInterpolationKernels<T>::blend(first.read(), second.read(), highway.view(), time, bits);
      same(scalar.data, highway.data);
    }
  }
}

void float_values() {
  constexpr int width = 65;
  const float tiny = std::numeric_limits<float>::denorm_min();
  const float safe = 0x1p100f;
  const float values[] = {0.0f, -0.0f, tiny, -tiny, 2 * tiny, -2 * tiny, 1.75f + 0x1p-23f, -1.0f, safe, -safe};
  std::vector<InterpolationSamples<float>> samples(width);
  std::vector<std::uint8_t> mF(width), mB(width);
  Buffer<float> first(width, 1), second(width, 1, 7);
  for (int pattern = 0; pattern < 3; ++pattern) {
    for (int i = 0; i < width; ++i) {
      if (pattern == 0) {
        const float a = values[i % 10], c = values[(i + 3) % 10];
        samples[i] = {a, c, values[(i + 2) % 10], values[(i + 5) % 10], values[(i + 7) % 10], values[(i + 9) % 10]};
      } else if (pattern == 1) {
        // Alternate A's zero sign to exercise first-operand min/max ties.
        const float a = i % 2 ? -0.0f : 0.0f, c = i % 2 ? 0.0f : -0.0f;
        samples[i] = {a, c, c, a, c, a};
      } else {
        const float value = values[i % 10];
        samples[i] = {value, value, value, value, value, value};
      }
      mF[i] = std::uint8_t(i % 2 ? 255 : 0);
      mB[i] = std::uint8_t(i % 3 ? 127 : 1);
      first.view().row(0)[i] = samples[i].A;
      second.view().row(0)[i] = samples[i].C;
    }
    for (const int time : times) {
      for (const bool extra : {false, true}) {
        Buffer<float> scalar(width, 1), highway(width, 1);
        ScalarInterpolationKernels<float>::compose(samples, mF, mB, extra, time, 32, scalar.view());
        HighwayInterpolationKernels<float>::compose(samples, mF, mB, extra, time, 32, highway.view());
        same(scalar.data, highway.data);
      }
      Buffer<float> scalar(width, 1), highway(width, 1);
      ScalarInterpolationKernels<float>::blend(first.read(), second.read(), scalar.view(), time, 32);
      HighwayInterpolationKernels<float>::blend(first.read(), second.read(), highway.view(), time, 32);
      same(scalar.data, highway.data);
    }
  }
}

void numeric_errors() {
  const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
  const float maximum = std::numeric_limits<float>::max();
  constexpr int width = 33;
  std::vector<InterpolationSamples<float>> samples(width, {0, 0, 0, 0, 0, 0});
  std::vector<std::uint8_t> masks(width, 0);
  for (const int index : {0, 15, 32})
    for (const bool extra : {false, true})
      for (const float bad : {nan, inf, -inf}) {
        samples[index] = {};
        if (extra)
          samples[index].K = bad;
        else
          samples[index].A0 = bad;
        Buffer<float> scalar(width, 1), highway(width, 1);
        CHECK(rejected(
            [&] { ScalarInterpolationKernels<float>::compose(samples, masks, masks, extra, 0, 32, scalar.view()); }));
        CHECK(rejected(
            [&] { HighwayInterpolationKernels<float>::compose(samples, masks, masks, extra, 0, 32, highway.view()); }));
        samples[index] = {};
      }
  // Finite required intermediates must overflow even if a later factor is zero.
  for (const bool extra : {false, true}) {
    samples.back().C = maximum;
    Buffer<float> scalar(width, 1), highway(width, 1);
    CHECK(rejected(
        [&] { ScalarInterpolationKernels<float>::compose(samples, masks, masks, extra, 0, 32, scalar.view()); }));
    CHECK(rejected(
        [&] { HighwayInterpolationKernels<float>::compose(samples, masks, masks, extra, 0, 32, highway.view()); }));
    samples.back() = {};
  }
  for (const float bad : {nan, inf, -inf, maximum, -maximum}) {
    Buffer<float> first(width, 2), second(width, 2), scalar(width, 2), highway(width, 2);
    std::fill(first.data.begin(), first.data.end(), 0.0f);
    std::fill(second.data.begin(), second.data.end(), 0.0f);
    // Nonfinite zero-weight operands still fail; finite maxima overflow at full weight.
    second.view().row(1)[32] = bad;
    const int time = std::isfinite(bad) ? 256 : 0;
    const auto before = highway.data;
    CHECK(rejected(
        [&] { ScalarInterpolationKernels<float>::blend(first.read(), second.read(), scalar.view(), time, 32); }));
    CHECK(rejected(
        [&] { HighwayInterpolationKernels<float>::blend(first.read(), second.read(), highway.view(), time, 32); }));
    same(before, scalar.data);
    same(before, highway.data);
  }
  // Fields not selected by the mode are not required arithmetic samples.
  for (const bool extra : {false, true}) {
    for (auto& sample : samples)
      sample =
          extra ? InterpolationSamples<float>{1, 2, nan, inf, 3, 4} : InterpolationSamples<float>{1, 2, 3, 4, nan, inf};
    Buffer<float> scalar(width, 1), highway(width, 1);
    ScalarInterpolationKernels<float>::compose(samples, masks, masks, extra, 128, 32, scalar.view());
    HighwayInterpolationKernels<float>::compose(samples, masks, masks, extra, 128, 32, highway.view());
    same(scalar.data, highway.data);
  }
}

template <class T>
void integer_blur(int bits) {
  std::mt19937 rng(27417);
  for (const std::size_t count : {1u, 2u, 3u, 7u, 8u, 9u, 15u, 16u, 17u, 31u, 32u, 33u, 63u, 64u, 65u, 65536u, 65537u})
    for (int pattern = 0; pattern < 2; ++pattern) {
      std::vector<T> input(count);
      std::uint64_t exact = 0;
      for (auto& value : input) {
        value = pattern ? random_sample<T>(rng, bits) : T((1u << bits) - 1);
        exact += value;
      }
      T scalar = 0, highway = 0;
      ScalarBlurAverage{}(input.data(), count, bits, &scalar);
      HighwayBlurAverage{}(input.data(), count, bits, &highway);
      CHECK(scalar == highway && highway == exact / count);
      CHECK(simd::interpolation_rows::sum(input.data(), count) == exact);
      if constexpr (std::is_same_v<T, std::uint16_t>)
        if (bits == 16 && count == 65537 && pattern == 0)
          CHECK(exact == UINT32_MAX);
    }
  if constexpr (std::is_same_v<T, std::uint16_t>)
    if (bits < 16) {
      std::vector<T> values(65, 0);
      for (std::size_t i = 0; i < values.size(); ++i) {
        values[i] = T(1u << bits);
        T output = 19;
        CHECK(rejected([&] { HighwayBlurAverage{}(values.data(), values.size(), bits, &output); }));
        CHECK(output == 19);
        values[i] = 0;
      }
    }
  T sample = 0, output = 0;
  CHECK(rejected([&] { HighwayBlurAverage{}(&sample, 0, bits, &output); }));
  CHECK(rejected([&] { HighwayBlurAverage{}(&sample, 65538, bits, &output); }));
}
template <class T>
void blur_repeated_directions(int bits) {
  constexpr int width = 7, height = 4, pad = 8;
  std::mt19937 rng(39123);
  Buffer<T> input(width + 2 * pad, height + 2 * pad);
  for (auto& sample : input.data)
    sample = random_sample<T>(rng, bits);
  RenderPhaseGeometry geometry{1, 1, 1, pad, pad, {}};
  geometry.phases[0] = {input.width, input.height};
  SubpixelPhases<T> phases{1, {}};
  phases.planes[0] = input.read();
  DenseFlowField f{width, height, std::vector<std::int16_t>(width * height),
                                std::vector<std::int16_t>(width * height)};
  auto b = f;
  const int xs[] = {0, 3, 3, 4, 4, 0, 0};
  const int ys[] = {0, 0, 2, 2, 3, 0, 0};
  for (int i = 0; i < width * height; ++i) {
    const int index = (i / 5) % 7;
    f.x[i] = static_cast<std::int16_t>(xs[index]);
    f.y[i] = static_cast<std::int16_t>(ys[index]);
    b.x[i] = static_cast<std::int16_t>(-ys[(index + 2) % 7]);
    b.y[i] = static_cast<std::int16_t>(-xs[(index + 2) % 7]);
  }
  for (int precision : {1, 2, 9})
    for (int time : {0, 85, 256}) {
      Buffer<T> scalar(width, height), highway(width, height);
      BlurSamplingPlan(geometry, width, height, precision, time).sample(f, b, phases, scalar.view(), bits);
      simd::BlurSamplingPlan(geometry, width, height, precision, time).sample(f, b, phases, highway.view(), bits);
      same(scalar.data, highway.data);
    }
}

void float_blur() {
  const float maximum = std::numeric_limits<float>::max(), tiny = std::numeric_limits<float>::denorm_min();
  const std::vector<std::vector<float>> sequences{
      {maximum, 1, -maximum}, {maximum, -maximum, 1},  {0x1p60f, 1, -0x1p60f, 3},
      {-0.0f, -0.0f},         {tiny, 2 * tiny, -tiny}, std::vector<float>(65537, 1.75f + 0x1p-23f)};
  for (const auto& input : sequences) {
    float scalar = 19, highway = 19;
    ScalarBlurAverage{}(input.data(), input.size(), 32, &scalar);
    HighwayBlurAverage{}(input.data(), input.size(), 32, &highway);
    CHECK(std::memcmp(&scalar, &highway, sizeof(float)) == 0);
  }
  float first = 0, second = 0;
  HighwayBlurAverage{}(sequences[0].data(), 3, 32, &first);
  HighwayBlurAverage{}(sequences[1].data(), 3, 32, &second);
  CHECK(first == 0 && second == mask_detail::binary32(1.0 / 3.0));
  for (const std::uint32_t pattern : {0x7fa12345u, 0x7fc54321u, 0x80000000u, 0u, 0x7f800000u, 0xff800000u, 1u}) {
    float input, scalar = 19, highway = 19;
    std::memcpy(&input, &pattern, sizeof(input));
    ScalarBlurAverage{}(&input, 1, 32, &scalar);
    HighwayBlurAverage{}(&input, 1, 32, &highway);
    CHECK(std::memcmp(&scalar, &pattern, sizeof(float)) == 0);
    CHECK(std::memcmp(&highway, &pattern, sizeof(float)) == 0);
  }
  for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    const float input[] = {0, bad};
    float output = 0;
    CHECK(rejected([&] { ScalarBlurAverage{}(input, 2, 32, &output); }));
    CHECK(rejected([&] { HighwayBlurAverage{}(input, 2, 32, &output); }));
  }
}

// End each raw row immediately before an inaccessible page, so vector tails
// cannot silently read or write beyond the logical array.
template <class T>
struct EndRow {
  void* memory;
  T* data;
  std::size_t page;
  explicit EndRow(int width) {
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
    CHECK(width > 0 && std::size_t(width) <= page / sizeof(T));
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
    data = reinterpret_cast<T*>(static_cast<unsigned char*>(memory) + page - std::size_t(width) * sizeof(T));
    for (int i = 0; i < width; ++i)
      ::new (data + i) T(0);
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
};
template <class T>
void guarded_sampled(int bits) {
  std::mt19937 rng(91273);
  for (const int width : widths)
    for (const int pel : {1, 2})
      for (const int odd : {0, 1}) {
        const int source_width = ((width + 11) / 4) * 4 + odd;
        RenderPhaseGeometry g{pel, 1, 1, 4, 0, {}};
        SubpixelPhases<T> images[2];
        std::array<std::unique_ptr<EndRow<T>>, 8> sources;
        for (int side = 0; side < 2; ++side) {
          images[side].pel = pel;
          for (int a = 0; a < pel * pel; ++a) {
            g.phases[a] = {source_width, 1};
            auto& source = sources[side * 4 + a];
            source = std::make_unique<EndRow<T>>(source_width);
            for (int i = 0; i < source_width; ++i)
              source->data[i] = random_sample<T>(rng, bits);
            images[side].planes[a] =
                checked_plane<const T>(source->data, source_width, 1, std::ptrdiff_t(source_width) * sizeof(T),
                                       std::size_t(source_width) * sizeof(T));
          }
        }
        DenseFlowField fields[4];
        for (int k = 0; k < 4; ++k) {
          auto& f = fields[k];
          f.width = width;
          f.height = 1;
          f.x.resize(width);
          f.y.resize(width);
          for (int i = 0; i < width; ++i) {
            f.x[i] = std::int16_t((i + k) % 7 - 3);
            f.y[i] = std::int16_t(pel == 2 ? (i + k) % 2 : 0);
          }
          // Exercise a gather of the very last pixel before the guard page.
          f.x.back() = std::int16_t((source_width - 1 - (4 + width - 1)) * pel);
        }
        std::vector<std::uint8_t> mf(width), mb(width);
        for (int i = 0; i < width; ++i) {
          mf[i] = std::uint8_t(rng());
          mb[i] = std::uint8_t(rng());
        }
        EndRow<T> expected(width), actual(width);
        const auto view = [&](T* data) {
          return checked_plane(data, width, 1, std::ptrdiff_t(width) * sizeof(T), std::size_t(width) * sizeof(T));
        };
        for (int time : times)
          for (bool extra : {false, true}) {
            const auto* bb = extra ? &fields[2] : nullptr;
            const auto* ff = extra ? &fields[3] : nullptr;
            const InterpolationSamplingPlan scalar(g, g, width, 1, time, bits);
            const simd::InterpolationSamplingPlan highway(g, g, width, 1, time, bits);
            scalar.preflight(fields[0], fields[1], bb, ff);
            scalar.render_preflighted(images[0], images[1], fields[0], fields[1], bb, ff, mf, mb, view(expected.data));
            highway.render_preflighted(images[0], images[1], fields[0], fields[1], bb, ff, mf, mb, view(actual.data));
            CHECK(std::memcmp(expected.data, actual.data, std::size_t(width) * sizeof(T)) == 0);
          }
      }
}
template <class T>
void guarded_rows() {
  for (const int width : widths) {
    EndRow<T> a(width), c(width), x(width), y(width), mf(width), mb(width), out(width);
    for (int i = 0; i < width; ++i) {
      a.data[i] = T(i % 2 ? 65535 : 10);
      c.data[i] = T(i % 3 ? 65535 : 21);
      x.data[i] = T(i % 2 ? 65535 : 0);
      y.data[i] = T(i % 3 ? 0 : 65535);
      mf.data[i] = T(i % 2 ? 255 : 0);
      mb.data[i] = T(i % 3 ? 127 : 1);
    }
    for (const int time : times) {
      for (const bool extra : {false, true}) {
        simd::interpolation_rows::compose(a.data, c.data, x.data, y.data, mf.data, mb.data, width, extra, time,
                                          out.data);
        for (int i = 0; i < width; ++i) {
          if constexpr (std::is_same_v<T, float>) {
            const float expected = extra ? interpolation_extra(a.data[i], c.data[i], x.data[i], y.data[i],
                                                               int(mf.data[i]), int(mb.data[i]), time, 32)
                                         : interpolation_basic(a.data[i], c.data[i], x.data[i], y.data[i],
                                                               int(mf.data[i]), int(mb.data[i]), time, 32);
            CHECK(std::memcmp(&expected, out.data + i, sizeof(T)) == 0);
          } else {
            const auto expected =
                extra ? interpolation_extra<std::uint16_t>(std::uint16_t(a.data[i]), std::uint16_t(c.data[i]),
                                                           std::uint16_t(x.data[i]), std::uint16_t(y.data[i]),
                                                           int(mf.data[i]), int(mb.data[i]), time, 16)
                      : interpolation_basic<std::uint16_t>(std::uint16_t(a.data[i]), std::uint16_t(c.data[i]),
                                                           std::uint16_t(x.data[i]), std::uint16_t(y.data[i]),
                                                           int(mf.data[i]), int(mb.data[i]), time, 16);
            CHECK(out.data[i] == expected);
          }
        }
      }
      simd::interpolation_rows::blend(a.data, c.data, width, time, out.data);
      for (int i = 0; i < width; ++i) {
        if constexpr (std::is_same_v<T, float>) {
          const auto expected = interpolation_blend(a.data[i], c.data[i], time, 32);
          CHECK(std::memcmp(&expected, out.data + i, sizeof(T)) == 0);
        } else {
          CHECK(out.data[i] ==
                interpolation_blend<std::uint16_t>(std::uint16_t(a.data[i]), std::uint16_t(c.data[i]), time, 16));
        }
      }
    }
  }
}
template <class T>
void guarded_sum() {
  for (const int width : widths) {
    EndRow<T> input(width);
    std::uint64_t expected = 0;
    for (int i = 0; i < width; ++i) {
      input.data[i] = std::numeric_limits<T>::max();
      expected += input.data[i];
    }
    CHECK(simd::interpolation_rows::sum(input.data, width) == expected);
  }
}
} // namespace

int main() {
  try {
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      std::cout << "Testing " << hwy::TargetName(target) << '\n';
      CHECK(std::strcmp(simd::detail::target_name(), hwy::TargetName(target)) == 0);
      compositions<std::uint8_t>(8);
      compositions<std::uint16_t>(10);
      compositions<std::uint16_t>(16);
      compositions<float>(32);
      float_values();
      numeric_errors();
      integer_blur<std::uint8_t>(8);
      integer_blur<std::uint16_t>(10);
      integer_blur<std::uint16_t>(16);
      float_blur();
      blur_repeated_directions<std::uint8_t>(8);
      blur_repeated_directions<std::uint16_t>(10);
      blur_repeated_directions<float>(32);
      guarded_rows<std::uint32_t>();
      guarded_rows<float>();
      guarded_sampled<std::uint8_t>(8);
      guarded_sampled<std::uint16_t>(10);
      guarded_sampled<std::uint16_t>(16);
      guarded_sampled<float>(32);
      guarded_sum<std::uint8_t>();
      guarded_sum<std::uint16_t>();
    }
    hwy::SetSupportedTargetsForTest(0);
    std::cout << "Interpolation exact differentials passed\n";
    return 0;
  } catch (const std::exception& error) {
    hwy::SetSupportedTargetsForTest(0);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
