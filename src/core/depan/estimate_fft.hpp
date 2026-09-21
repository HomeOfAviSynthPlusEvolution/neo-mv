#pragma once
#include <complex>
#include <cstddef>
#include <vector>

namespace neo_mv::depan::estimate {
enum class FftProfile { scalar, native, sse2, avx2, avx512 };
// Validate every sample without modifying it; throw if any sample is non-finite.
// Complex buffers are supplied as interleaved real/imaginary float samples.
using SampleValidator = void (*)(const float*, std::size_t);
// Compiled lane capacity, not a promise that every small transform uses SIMD.
int fft_lanes(FftProfile profile) noexcept;
const char* fft_profile_name(FftProfile profile) noexcept;
// Immutable shape; all writable transform storage belongs to the calling request.
// Both pinned float profiles are single-threaded and unnormalized.
class FftPlan {
  int width_, height_;
  std::size_t real_count_, complex_count_;
  FftProfile profile_;

public:
  FftPlan(int width, int height, FftProfile profile = FftProfile::scalar);
  int width() const { return width_; }
  int height() const { return height_; }
  std::size_t real_count() const { return real_count_; }
  std::size_t complex_count() const { return complex_count_; }
  std::vector<std::complex<float>> forward(const std::vector<float>& input, SampleValidator validator = nullptr) const;
  std::vector<float> inverse(const std::vector<std::complex<float>>& input, SampleValidator validator = nullptr) const;
  std::vector<float> correlate(const std::vector<float>& current, const std::vector<float>& previous) const;
};
} // namespace neo_mv::depan::estimate
