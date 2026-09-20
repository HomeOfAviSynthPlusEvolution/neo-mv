#include "core/depan/sampling.hpp"

#include <iostream>
#include <string>
#include <new>
#include <vector>
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
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Depan sampling assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
template <class T>
struct Buffer {
  int width, height, stride;
  std::vector<T> data;
  Buffer(int w, int h, int gap = 3) : width(w), height(h), stride(w + gap), data(std::size_t(stride) * h + 1, T(253)) {}
  span2d::Plane<T> view() {
    return checked_plane(data.data() + 1, width, height, std::ptrdiff_t(stride) * sizeof(T),
                         (data.size() - 1) * sizeof(T));
  }
  span2d::Plane<const T> read() { return view(); }
  void gradient() {
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        view().row(y)[x] = T(y * 50 + x * 10);
  }
  void guards() const {
    CHECK(data[0] == T(253));
    for (int y = 0; y < height; ++y)
      for (int x = width; x < stride; ++x)
        CHECK(data[1 + std::size_t(y) * stride + x] == T(253));
  }
};
template <class T>
Buffer<T> run(Buffer<T>& source, Transform map, int mode, int mirror = 0, int blur = 0, int bits = 8,
              int border = 250) {
  Buffer<T> output(source.width, source.height, 5);
  SamplingPlan(source.width, source.height, bits, mode, mirror, blur, border, map).render(source.read(), output.view());
  source.guards();
  output.guards();
  return output;
}
void nearest_and_borders() {
  Buffer<std::uint8_t> source(5, 5);
  source.gradient();
  Transform map;
  map.tx = 2;
  auto output = run(source, map, 0);
  CHECK(output.view().row(0)[0] == 20 && output.view().row(0)[3] == 250);
  map.tx = -2;
  CHECK(run(source, map, 0, 4, 0).view().row(0)[0] == 20);
  CHECK(run(source, map, 0, 4, 2).view().row(0)[0] == 15);
  map.tx = -2.75f;
  map.v = 0.25f;
  CHECK(run(source, map, 0, 4, 2).view().row(0)[0] == 20); // R ignores blur.
  map = {};
  map.tx = -0.75f;
  CHECK(run(source, map, 0).view().row(0)[0] == 250);
  map.ty = 1;
  map.v = 0.25f;
  CHECK(run(source, map, 0).view().row(0)[0] == 50); // R truncates after +0.5.
  map = {};
  map.ty = -10;
  CHECK(run(source, map, 0, 3).view().row(0)[0] == 250); // -10 -> 10 -> -2, once each.
  map = {};
  map.tx = -10;
  CHECK(run(source, map, 0, 12).view().row(0)[0] == 40); // Horizontal clamp differs from general reflection.
  map.v = 0.25f;
  CHECK(run(source, map, 0, 12).view().row(0)[0] == 250);
  map = {};
  map.tx = 5;
  CHECK(run(source, map, 0, 8, 2).view().row(0)[0] == 30);
  CHECK(run(source, map, 1, 8, 2).view().row(0)[0] == 35);
  map.tx = 4;
  CHECK(run(source, map, 1, 0).view().row(0)[0] == 250);
  CHECK(run(source, map, 1, 8).view().row(0)[0] == 40);
  map.tx = 5;
  map.ty = 4;
  CHECK(run(source, map, 1, 8, 2).view().row(0)[0] == 230); // Last-row T ignores blur.
  map.u = 2;
  CHECK(run(source, map, 1, 8, 2).view().row(0)[0] == 250); // Last-row Z does not reflect X.
}
void tiny_image_tables() {
  for (int width = 1; width <= 5; ++width)
    for (int height = 1; height <= 5; ++height) {
      Buffer<std::uint8_t> source(width, height);
      source.gradient();
      for (int mode = 0; mode <= 2; ++mode) {
        if (mode == 2 && height == 1) {
          rejects([&] { run(source, {}, mode); });
          continue;
        }
        const auto identity = run(source, {}, mode);
        for (int y = 0; y < height; ++y)
          for (int x = 0; x < width; ++x) {
            const int expected = mode == 1 && y < height - 1 && x == width - 1 ? 250 : y * 50 + x * 10;
            CHECK(identity.data[1 + std::size_t(y) * identity.stride + x] == expected);
          }
        if (mode == 2) {
          Transform scale;
          scale.u = 0;
          auto result = run(source, scale, mode);
          for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
              CHECK(result.view().row(y)[x] == (y == height - 1 ? y * 50 - 25 : y * 50));
          scale.w = std::numeric_limits<float>::denorm_min();
          result = run(source, scale, mode); // R falls back to one sample, never bottom averaging.
          CHECK(result.view().row(height - 1)[0] == (height - 1) * 50);
          Transform left;
          left.tx = -1;
          result = run(source, left, mode);
          for (int y = 0; y < height; ++y)
            CHECK(result.view().row(y)[0] == 250); // Includes W=1, W-2=-1.
        }
      }
    }
  Buffer<std::uint8_t> one(1, 2);
  one.view().row(0)[0] = 10;
  one.view().row(1)[0] = 21;
  Transform scale;
  scale.u = 2;
  const auto result = run(one, scale, 2);
  CHECK(result.data[1] == 10 && result.data[1 + result.stride] == 15);
}
void quantized_interpolation() {
  Buffer<std::uint8_t> source(2, 2);
  source.view().row(0)[0] = 0;
  source.view().row(0)[1] = 10;
  source.view().row(1)[0] = 20;
  source.view().row(1)[1] = 31;
  Transform half;
  half.tx = half.ty = 0.5f;
  CHECK(run(source, half, 1).view().row(0)[0] == 15);
  CHECK(run(source, half, 2).view().row(0)[0] == 15); // B2 near-edge rule.
  Transform tiny;
  tiny.tx = 1.0f / 64;
  CHECK(run(source, tiny, 1).view().row(0)[0] == 0);
  CHECK(run(source, tiny, 1).view().row(0)[1] == 250); // Missing zero-weight tap is not read.
  // A tiny negative coordinate has a fractional difference rounded to 1.
  tiny.tx = -std::numeric_limits<float>::denorm_min();
  SamplingPlan endpoint(2, 2, 8, 1, 0, 0, 250, tiny);
  endpoint.row_coordinates(0, [&](int x, SamplingCoordinates q) {
    if (x == 1)
      CHECK(q.i == 0 && q.fx == 1);
  });
  CHECK(run(source, tiny, 1).view().row(0)[1] == 10);

  const std::array<std::int64_t, 4> at0{0, 2048, 0, 0}, at128{-256, 1280, 1280, -256}, at256{0, 0, 2048, 0};
  CHECK(cubic_coefficients(0) == at0 && cubic_coefficients(128) == at128 && cubic_coefficients(256) == at256);
  Buffer<std::uint16_t> cubic(4, 4);
  for (int y = 0; y < 4; ++y)
    std::fill_n(cubic.view().row(y).data(), 4, std::uint16_t{0});
  cubic.view().row(1)[1] = 100;
  CHECK(run(cubic, half, 2, 0, 0, 16).view().row(1)[1] == 39);
  Transform scale;
  scale.u = scale.h = 0;
  scale.tx = scale.ty = 1.5f;
  CHECK(run(cubic, scale, 2, 0, 0, 16).view().row(0)[0] == 39);
  cubic.view().row(1)[1] = 65535;
  CHECK(run(cubic, half, 2, 0, 0, 16).view().row(1)[1] == 25600);
  CHECK(run(cubic, scale, 2, 0, 0, 16).view().row(0)[0] == 25599); // QT bias versus QZR floor.
  cubic.view().row(1)[1] = 0;
  cubic.view().row(1)[0] = 65535;
  CHECK(run(cubic, half, 2, 0, 0, 16).view().row(1)[1] == 0); // Negative lobe clamps.
  cubic.view().row(1)[0] = 0;
  for (int y : {1, 2})
    for (int x : {1, 2})
      cubic.view().row(y)[x] = 65535;
  CHECK(run(cubic, half, 2, 0, 0, 16).view().row(1)[1] == 65535); // Positive overshoot clamps.
  for (int bits : {10, 16}) {
    const auto maximum = std::uint16_t((1u << bits) - 1);
    for (int y = 0; y < 4; ++y)
      std::fill_n(cubic.view().row(y).data(), 4, maximum);
    CHECK(run(cubic, half, 1, 0, 0, bits).view().row(0)[0] == maximum);
    CHECK(run(cubic, half, 2, 0, 0, bits).view().row(1)[1] == maximum);
  }
}
void finite_coordinates_and_blur() {
  Buffer<std::uint8_t> source(5, 5);
  source.gradient();
  Transform map;
  map.tx = 1000;
  SamplingPlan clamped(4, 4, 8, 0, 0, 0, 250, map);
  CHECK(clamped.map().tx == 72);
  map.tx = -1000;
  CHECK(run(source, map, 0, 4, INT32_MAX).view().row(0)[0] == 39);
  map = {};
  map.u = 1000000000.0f;
  CHECK(run(source, map, 0, 8, INT32_MAX).view().row(0)[1] == 0);
  // A huge finite coordinate is an ordinary border input, not an integer-cast error.
  Buffer<std::uint8_t> tiny_image(2, 2);
  tiny_image.gradient();
  map.u = std::numeric_limits<float>::max();
  CHECK(run(tiny_image, map, 0).view().row(0)[1] == 250);
  CHECK(run(tiny_image, map, 1, 8, INT32_MAX).view().row(0)[1] == 0);
  map.v = std::numeric_limits<float>::denorm_min();
  CHECK(run(tiny_image, map, 0).view().row(0)[1] == 250); // No unused third-column recurrence.
  Buffer<std::uint8_t> wide(3, 2);
  wide.gradient();
  rejects([&] { run(wide, map, 0); }); // Required recurrence overflows binary32.
  map = {};
  map.h = std::numeric_limits<float>::max();
  CHECK(run(tiny_image, map, 2).view().row(1)[0] == 250);
  Buffer<std::uint8_t> tall(2, 3);
  tall.gradient();
  rejects([&] { run(tall, map, 2); });
  map = {};
  map.u = std::numeric_limits<float>::infinity();
  rejects([&] { run(tiny_image, map, 0); });

  map = {};
  map.u = 0.1f;
  map.v = 0.25f;
  SamplingPlan(8, 2, 8, 1, 0, 0, 0, map).row_coordinates(0, [&](int x, SamplingCoordinates q) {
    if (x == 7)
      CHECK(q.i == 0 && q.fx == 0x1.666668p-1f); // Seven separately rounded additions.
  });
  SamplingPlan(8, 2, 8, 2, 0, 0, 0, map).row_coordinates(0, [&](int x, SamplingCoordinates q) {
    if (x == 7)
      CHECK(q.i == 0 && q.fx == 0x1.666666p-1f); // Independently rounded multiplication.
  });
}

struct EndImage {
  void* memory;
  std::uint8_t* data;
  std::size_t page;
  explicit EndImage(int count) {
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    page = info.dwPageSize;
    memory = VirtualAlloc(nullptr, 2 * page, MEM_RESERVE, PAGE_NOACCESS);
    if (!memory)
      throw std::bad_alloc();
    if (!VirtualAlloc(memory, page, MEM_COMMIT, PAGE_READWRITE)) {
      VirtualFree(memory, 0, MEM_RELEASE);
      throw std::bad_alloc();
    }
#else
    const auto size = sysconf(_SC_PAGESIZE);
    if (size <= 0)
      throw std::runtime_error("cannot query page size");
    page = static_cast<std::size_t>(size);
    memory = mmap(nullptr, 2 * page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (memory == MAP_FAILED)
      throw std::bad_alloc();
    if (mprotect(memory, page, PROT_READ | PROT_WRITE) != 0) {
      munmap(memory, 2 * page);
      throw std::bad_alloc();
    }
#endif
    data = static_cast<std::uint8_t*>(memory) + page - count;
    for (int i = 0; i < count; ++i)
      ::new (data + i) std::uint8_t(static_cast<std::uint8_t>(10 + i));
  }
  ~EndImage() {
#if defined(_WIN32)
    VirtualFree(memory, 0, MEM_RELEASE);
#else
    munmap(memory, 2 * page);
#endif
  }
  EndImage(const EndImage&) = delete;
  EndImage& operator=(const EndImage&) = delete;
};
void protected_tiny_image_images() {
  for (int width = 1; width <= 5; ++width)
    for (int height = 2; height <= 5; ++height) {
      EndImage input(width * height), output(width * height);
      const auto source = checked_plane<const std::uint8_t>(input.data, width, height, width, width * height);
      const auto target = checked_plane(output.data, width, height, width, width * height);
      for (int mode = 0; mode <= 2; ++mode) {
        SamplingPlan(width, height, 8, mode, 0, 0, 250, {}).render(source, target);
        for (int y = 0; y < height; ++y)
          for (int x = 0; x < width; ++x)
            CHECK(target.row(y)[x] == (mode == 1 && y < height - 1 && x == width - 1 ? 250 : source.row(y)[x]));
      }
    }
}
void views_and_validation() {
  Buffer<std::uint16_t> source(3, 3), output(3, 3, 1);
  source.gradient();
  const SamplingPlan plan(3, 3, 10, 0, 0, 0, 1023, {});
  rejects([&] { plan.render(source.read(), source.view()); });
  rejects([&] { plan.render(source.read(), output.view().subplane(0, 0, 2, 3)); });
  source.view().row(2)[2] = 1024;
  const auto before = output.data;
  Transform outside;
  outside.tx = 1000;
  rejects([&] { SamplingPlan(3, 3, 10, 0, 0, 0, 1023, outside).render(source.read(), output.view()); });
  CHECK(output.data == before); // Every visible source sample is checked, even if unused by this map.
  for (int mode : {-1, 3})
    rejects([&] { SamplingPlan(3, 3, 8, mode, 0, 0, 0, {}); });
  rejects([&] { SamplingPlan(3, 3, 8, 0, 16, 0, 0, {}); });
  rejects([&] { SamplingPlan(3, 3, 8, 0, 0, -1, 0, {}); });
  rejects([&] { SamplingPlan(3, 3, 8, 0, 0, 0, 256, {}); });
}
} // namespace
int main() {
  try {
    nearest_and_borders();
    tiny_image_tables();
    quantized_interpolation();
    finite_coordinates_and_blur();
    protected_tiny_image_images();
    views_and_validation();
    std::cout << "Depan sampling specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
