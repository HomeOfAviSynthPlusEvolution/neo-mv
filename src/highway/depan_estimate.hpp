#pragma once
#include "core/depan/estimate_fft.hpp"

namespace neo_mv::simd::estimate {
// Replace current with conj(current) * previous. Both ranges have count
// complex elements; previous is immutable and the ranges must not overlap.
void product(std::complex<float>* current, const std::complex<float>* previous, std::size_t count);
inline std::vector<float> correlate(const depan::estimate::FftPlan& plan, const std::vector<float>& current,
                                    const std::vector<float>& previous) {
  auto a = plan.forward(current);
  const auto b = plan.forward(previous);
  product(a.data(), b.data(), a.size());
  return plan.inverse(a);
}
} // namespace neo_mv::simd::estimate
