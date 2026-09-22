// Manual synthetic sampling benchmark; never registered with CTest.
// Setup, scalar/Highway comparison and warmup are outside the timed interval.
// Interpolation timing includes its result allocation and destruction.
// Depan timing includes plan construction and source view extraction.
#include "core/flow/sampling.hpp"
#include "core/interpolation/sampling.hpp"
#include "core/interpolation/blur.hpp"
#include "highway/flow_sampling.hpp"
#include "highway/interpolation_sampling.hpp"
#include "highway/interpolation.hpp"
#include "highway/depan.hpp"
#include "hwy/targets.h"
#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
volatile std::uint64_t sink = 0;
void escape(const void* pointer) {
#if defined(__GNUC__) || defined(__clang__)
  asm volatile("" : : "g"(pointer) : "memory");
#else
  (void)pointer;
  std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}
int number(const char* text, int limit) {
  std::size_t end = 0;
  const int value = std::stoi(text, &end);
  if (end != std::strlen(text) || value < 1 || value > limit)
    throw std::invalid_argument("numeric argument out of range");
  return value;
}
template <class T>
void run(const std::string& op, int width, int height, int iterations, int samples, bool highway) {
  using namespace neo_mv;
  const int bits = std::is_same_v<T, float> ? 32 : int(sizeof(T) * 8);
  const int pel = 4, pad = 16, stride = width + 2 * pad + 3;
  const std::size_t count = std::size_t(width) * height;
  RenderPhaseGeometry geometry{pel, 1, 1, pad, pad, {}};
  std::array<std::vector<T>, 16> storage;
  SubpixelPhases<T> phases{pel, {}};
  for (int a = 0; a < 16; ++a) {
    const int w = width + 2 * pad - (a % pel == 3), h = height + 2 * pad - (a / pel == 3);
    geometry.phases[a] = {w, h};
    storage[a].resize(std::size_t(stride) * h);
    for (std::size_t i = 0; i < storage[a].size(); ++i)
      storage[a][i] = T((i * 17 + a * 29) % 251);
    phases.planes[a] =
        checked_plane<const T>(storage[a].data(), w, h, stride * sizeof(T), storage[a].size() * sizeof(T));
  }
  DenseFlowField f{width, height, std::vector<std::int16_t>(count), std::vector<std::int16_t>(count)}, b = f;
  for (std::size_t i = 0; i < count; ++i) {
    f.x[i] = static_cast<std::int16_t>(int(i % 31) - 15);
    f.y[i] = static_cast<std::int16_t>(15 - int((i / width) % 31));
    b.x[i] = -f.x[i];
    b.y[i] = -f.y[i];
  }
  std::vector<T> scalar(count), simd(count);
  const auto a = checked_plane(scalar.data(), width, height, width * sizeof(T), scalar.size() * sizeof(T));
  const auto c = checked_plane(simd.data(), width, height, width * sizeof(T), simd.size() * sizeof(T));
  const FlowSamplingPlan flow(geometry, width, height, 128);
  const simd::FlowSamplingPlan hflow(geometry, width, height, 128);
  const InterpolationSamplingPlan inter(geometry, geometry, width, height, 128, bits);
  const simd::InterpolationSamplingPlan hinter(geometry, geometry, width, height, 128, bits);
  const BlurSamplingPlan blur(geometry, width, height, 1, 256);
  const simd::BlurSamplingPlan hblur(geometry, width, height, 1, 256);
  auto invoke = [&](bool use_highway) {
    auto output = use_highway ? c : a;
    if (op == "flow") {
      if (use_highway)
        hflow.sample(f, phases, output);
      else
        flow.sample(f, phases, output);
    } else if (op == "flow-preflight") {
      if (use_highway)
        hflow.preflight(f);
      else
        flow.preflight(f);
    } else if (op == "interpolation") {
      const auto values = use_highway ? hinter.sample(phases, phases, b, f) : inter.sample(phases, phases, b, f);
      escape(values.data());
      output.row(0)[0] = values.front().A;
    } else if (op == "blur") {
      if (use_highway)
        hblur.sample(f, b, phases, output, bits, HighwayBlurAverage{});
      else
        blur.sample(f, b, phases, output, bits);
    } else if (op == "depan0" || op == "depan1" || op == "depan2") {
      if constexpr (std::is_same_v<T, float>) {
        throw std::invalid_argument("Depan requires integer samples");
      } else {
        const int mode = op.back() - '0';
        const depan::Transform map{0.3f, -0.2f, 1.001f, -0.003f, 0.003f, 1.001f};
        const auto source = phases.planes[0].subplane(pad, pad, width, height);
        if (use_highway)
          depan::HighwaySamplingPlan(width, height, bits, mode, 0, 0, 0, map).render(source, output);
        else
          depan::SamplingPlan(width, height, bits, mode, 0, 0, 0, map).render(source, output);
      }
    } else
      throw std::invalid_argument("unknown operation");
    escape(output.data());
  };
  if (op == "interpolation") {
    const auto x = inter.sample(phases, phases, b, f), y = hinter.sample(phases, phases, b, f);
    for (std::size_t i = 0; i < count; ++i)
      for (auto member : {&InterpolationSamples<T>::A, &InterpolationSamples<T>::C, &InterpolationSamples<T>::A0,
                          &InterpolationSamples<T>::C0, &InterpolationSamples<T>::E, &InterpolationSamples<T>::K})
        if (std::memcmp(&(x[i].*member), &(y[i].*member), sizeof(T)))
          throw std::runtime_error("interpolation samples differ from scalar");
  }
  invoke(false);
  invoke(true);
  if (std::memcmp(scalar.data(), simd.data(), scalar.size() * sizeof(T)))
    throw std::runtime_error("scalar/Highway output mismatch");
  for (int i = 0; i < 3; ++i)
    invoke(highway);
  for (int sample = 0; sample < samples; ++sample) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i)
      invoke(highway);
    const auto end = std::chrono::steady_clock::now();
    const auto& output = highway ? simd : scalar;
    std::uint64_t checksum = 0;
    for (std::size_t i = 0; i < output.size(); ++i) {
      std::uint32_t bits_value = 0;
      std::memcpy(&bits_value, &output[i], sizeof(T));
      checksum += bits_value;
    }
    sink = checksum;
    std::cout << sample << ',' << iterations << ',' << std::fixed << std::setprecision(3)
              << std::chrono::duration<double, std::nano>(end - start).count() / iterations << ',' << checksum << '\n';
  }
}
} // namespace
int main(int argc, char** argv) {
  try {
    if (argc != 8 && argc != 9)
      throw std::invalid_argument(
          "usage: neo_mv_sampling_benchmark u8|u16|f32 flow|flow-preflight|interpolation|blur|depan0|depan1|depan2 "
          "width height iterations samples scalar|highway [target]");
    const std::string type = argv[1], op = argv[2], backend = argv[7];
    const int width = number(argv[3], 8192), height = number(argv[4], 8192);
    const int iterations = number(argv[5], 1000000), samples = number(argv[6], 1000);
    if (backend != "scalar" && backend != "highway")
      throw std::invalid_argument("unknown backend");
    if (argc == 9) {
      bool found = false;
      for (auto target : hwy::SupportedAndGeneratedTargets())
        if (std::strcmp(argv[8], hwy::TargetName(target)) == 0) {
          hwy::SetSupportedTargetsForTest(target);
          found = true;
          break;
        }
      if (!found)
        throw std::invalid_argument("target not supported and generated");
    }
    std::cout << "# " << type << ' ' << op << ' ' << width << 'x' << height << ' ' << backend << ' '
              << (backend == "highway" ? neo_mv::simd::detail::target_name() : "scalar")
              << "\nsample,iterations,ns_per_call,checksum\n";
    if (type == "u8")
      run<std::uint8_t>(op, width, height, iterations, samples, backend == "highway");
    else if (type == "u16")
      run<std::uint16_t>(op, width, height, iterations, samples, backend == "highway");
    else if (type == "f32")
      run<float>(op, width, height, iterations, samples, backend == "highway");
    else
      throw std::invalid_argument("unknown sample type");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
