// Manual synthetic benchmark. No reference implementation or private host payloads.
// Usage: neo_mv_benchmark u8|u16|f32 sad|satd|analyse-sad|analyse-satd
//        width scalar|highway iterations samples [Highway target name]
// Each CSV row is an independent timed sample; setup and scalar comparison are untimed.
// Analyse timing includes result allocation, checksum consumption and destruction.
#include "../tests/core/motion_fixture.hpp"
#include "core/motion/analyse.hpp"
#include "highway/kernels.hpp"
#include "hwy/targets.h"
#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

namespace {
volatile std::uint64_t sink = 0;

template <class Call>
void measure(Call&& call, const std::string& label, int iterations, int samples) {
  for (int i = 0; i < 32; ++i)
    sink = call(i);
  for (int s = 0; s < samples; ++s) {
    std::uint64_t checksum = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
      // Prevent pure scalar calls being hoisted across iterations, without a CPU fence.
      std::atomic_signal_fence(std::memory_order_seq_cst);
      checksum += call(i);
    }
    const auto end = std::chrono::steady_clock::now();
    sink = checksum;
    const double ns = std::chrono::duration<double, std::nano>(end - start).count() / iterations;
    std::cout << label << ',' << s << ',' << iterations << ',' << std::fixed << std::setprecision(3) << ns << ','
              << checksum << '\n';
  }
}

template <class T>
void randomize(std::vector<T>& data, std::mt19937& rng) {
  for (auto& v : data) {
    if constexpr (std::is_same_v<T, float>)
      v = float(rng() % 65536) / 65535.0f;
    else
      v = T(rng());
  }
}

std::uint64_t digest(const neo_mv::MotionGrid& field) {
  std::uint64_t result = 0;
  for (const auto& v : field.values)
    result =
        result * 0x9e3779b1U + std::uint32_t(v.vector.x) + 31ULL * std::uint32_t(v.vector.y) + std::uint64_t(v.error);
  return result;
}

template <class T>
void run(const std::string& op, int width, bool highway, int iterations, int samples, const std::string& label) {
  std::mt19937 rng(75142);
  const bool satd = op == "satd" || op == "analyse-satd";
  if (op == "analyse-sad" || op == "analyse-satd") {
    MotionFixture<T> f(256, 128, width, width, 16, 1, false);
    randomize(f.source[0], rng);
    randomize(f.reference[0][0], rng);
    neo_mv::AnalyseControls controls;
    controls.metric = satd ? neo_mv::MotionMetric::satd : neo_mv::MotionMetric::sad;
    controls.badrange = 0;
    const std::vector<neo_mv::SamplingGeometry> geometries{f.geometry};
    const std::vector<neo_mv::SamplingFrames<T>> frames{f.frames};
    const auto scalar = [&] {
      return neo_mv::analyse_vectors<T>(f.metadata, geometries, frames, controls);
    };
    const auto simd = [&] {
      return neo_mv::analyse_vectors<T, neo_mv::HighwayKernels<T>>(f.metadata, geometries, frames, controls);
    };
    const auto a = scalar(), b = simd();
    if (a.values.size() != b.values.size())
      throw std::runtime_error("analysis size mismatch");
    for (std::size_t i = 0; i < a.values.size(); ++i) {
      const auto& x = a.values[i];
      const auto& y = b.values[i];
      if (x.vector.x != y.vector.x || x.vector.y != y.vector.y || x.error != y.error)
        throw std::runtime_error("analysis differs from scalar");
    }
    if (highway)
      measure([&](int) { return digest(simd()); }, label, iterations, samples);
    else
      measure([&](int) { return digest(scalar()); }, label, iterations, samples);
    return;
  }
  const int stride = width + 19;
  std::vector<T> a(std::size_t(stride) * width), b(a.size());
  randomize(a, rng);
  randomize(b, rng);
  const auto source =
      neo_mv::checked_plane<const T>(a.data() + 1, width, width, stride * sizeof(T), (a.size() - 1) * sizeof(T));
  std::array<span2d::Plane<const T>, 16> refs;
  for (int i = 0; i < 16; ++i)
    refs[i] =
        neo_mv::checked_plane<const T>(b.data() + i, width, width, stride * sizeof(T), (b.size() - i) * sizeof(T));
  const auto metric = satd ? neo_mv::BlockMetric::satd : neo_mv::BlockMetric::sad;
  for (const auto ref : refs)
    if (neo_mv::block_metric(source, ref, metric) != neo_mv::simd::block_metric(source, ref, metric))
      throw std::runtime_error("metric differs from scalar");
  if (highway)
    measure([&](int i) { return neo_mv::simd::block_metric(source, refs[i & 15], metric); }, label, iterations,
            samples);
  else
    measure([&](int i) { return neo_mv::block_metric(source, refs[i & 15], metric); }, label, iterations, samples);
}

int number(const char* text, int maximum) {
  std::size_t end = 0;
  const int value = std::stoi(text, &end);
  if (end != std::strlen(text) || value < 1 || value > maximum)
    throw std::invalid_argument("numeric argument out of range");
  return value;
}
} // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 7 && argc != 8)
      throw std::invalid_argument("usage: neo_mv_benchmark u8|u16|f32 sad|satd|analyse-sad|analyse-satd width "
                                  "scalar|highway iterations samples [target]");
    const std::string type = argv[1], op = argv[2], backend = argv[4];
    const int width = number(argv[3], 128), iterations = number(argv[5], 1000000000), samples = number(argv[6], 1000);
    if (backend != "scalar" && backend != "highway")
      throw std::invalid_argument("unknown backend");
    if (op != "sad" && op != "satd" && op != "analyse-sad" && op != "analyse-satd")
      throw std::invalid_argument("unknown operation");
    if (argc == 8) {
      bool found = false;
      for (auto target : hwy::SupportedAndGeneratedTargets())
        if (std::strcmp(argv[7], hwy::TargetName(target)) == 0) {
          hwy::SetSupportedTargetsForTest(target);
          found = true;
          break;
        }
      if (!found)
        throw std::invalid_argument("target is not supported and generated");
    }
    const auto label = type + ',' + op + ',' + std::to_string(width) + ',' + backend + ',' +
                       (backend == "highway" ? neo_mv::simd::detail::target_name() : "scalar");
    std::cout << "type,operation,width,backend,target,sample,iterations,ns_per_call,checksum\n";
    if (type == "u8")
      run<std::uint8_t>(op, width, backend == "highway", iterations, samples, label);
    else if (type == "u16")
      run<std::uint16_t>(op, width, backend == "highway", iterations, samples, label);
    else if (type == "f32")
      run<float>(op, width, backend == "highway", iterations, samples, label);
    else
      throw std::invalid_argument("unknown sample type");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
