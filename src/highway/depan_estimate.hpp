#pragma once
#include "core/depan/estimate_fft.hpp"
#include "core/depan/estimate_motion.hpp"

namespace neo_mv::simd::estimate {
bool native_fma();
// Validate count readable float samples; an empty range is allowed.
void samples_finite(const float* values, std::size_t count);
// Inputs and accumulators must be finite. Preserve ordered binary32 addition
// and the first strictly greater maximum; return count if none exceeds it.
std::size_t scan_row(const float* values, std::size_t count, float& sum, float& maximum);
struct MotionScan {
  static void validate(const float* values, std::size_t count) { samples_finite(values, count); }
  static std::size_t scan(const float* values, std::size_t count, float& sum, float& maximum) {
    return scan_row(values, count, sum, maximum);
  }
};
// Replace current with conj(current) * previous. Both ranges have count
// complex elements; previous is immutable and the ranges must not overlap.
void product(std::complex<float>* current, const std::complex<float>* previous, std::size_t count);
inline std::vector<float> correlate(const depan::estimate::FftPlan& plan, const std::vector<float>& current,
                                    const std::vector<float>& previous) {
  auto a = plan.forward(current, samples_finite);
  const auto b = plan.forward(previous, samples_finite);
  product(a.data(), b.data(), a.size());
  return plan.inverse(a, samples_finite);
}
} // namespace neo_mv::simd::estimate
