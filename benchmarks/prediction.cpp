// Manual whole-grid prediction benchmark, including dispatch and plan setup.
#include "highway/prediction.hpp"
#include "highway/rows.hpp"
#include "hwy/targets.h"
#include <chrono>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <atomic>

int main(int argc, char** argv) {
  using namespace neo_mv;
  if (argc > 1) {
    bool found = false;
    for (auto target : hwy::SupportedAndGeneratedTargets())
      if (std::string(argv[1]) == hwy::TargetName(target)) {
        hwy::SetSupportedTargetsForTest(target);
        found = true;
        break;
      }
    if (!found)
      return 1;
  }
  std::cout << simd::detail::target_name() << '\n';
  for (int width : {16, 64, 240})
    for (int overlap : {0, 2, 4}) {
      MotionGrid parent{width, width / 2, std::vector<MotionTriple>(std::size_t(width) * width / 2)};
      for (std::size_t i = 0; i < parent.values.size(); ++i)
        parent.values[i] = {{int(i * 31 % 2048) - 1024, int(i * 17 % 2048) - 1024}, std::int64_t(i * 137 % 1000000)};
      MotionGrid child{2 * width, width, std::vector<MotionTriple>(std::size_t(2 * width) * width)};
      PredictionGeometry g{8, 8, overlap, overlap, 1, 2};
      std::vector<double> times[2];
      const int iterations = std::max(20, 200000 / (width * width));
      for (int j = 0; j < 20; ++j) {
        interpolate_predictions(parent, g, child);
        simd::interpolate_predictions(parent, g, child);
      }
      for (int round = 0; round < 9; ++round)
        for (int k = 0; k < 2; ++k) {
          const int backend = (k + round) % 2;
          const auto begin = std::chrono::steady_clock::now();
          for (int j = 0; j < iterations; ++j) {
            if (backend)
              simd::interpolate_predictions(parent, g, child);
            else
              interpolate_predictions(parent, g, child);
#if defined(__GNUC__) || defined(__clang__)
            asm volatile("" : : "g"(child.values.data()) : "memory");
#else
            std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
          }
          times[backend].push_back(
              std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count() / iterations);
        }
      for (auto& t : times)
        std::sort(t.begin(), t.end());
      std::cout << width << ',' << overlap << ',' << times[0][4] << ',' << times[1][4] << ','
                << times[0][4] / times[1][4] << '\n';
    }
}
