#include "core/depan/estimate_fft.hpp"
#include "core/depan/estimate_fft_backend.hpp"
#include "core/depan/numeric.hpp"
#include <limits>
#if NEO_MV_ENABLE_SIMD
#include "hwy/targets.h"
#endif

namespace neo_mv::depan::estimate {
namespace detail {
const FftBackend& native_fft() noexcept {
#if NEO_MV_ENABLE_SIMD
#if NEO_MV_FFT_X86_TARGETS
  const auto targets = hwy::SupportedTargets();
  if (targets & (HWY_AVX3 | HWY_AVX3_SPR | HWY_AVX3_ZEN4 | HWY_AVX3_DL)) {
    if (avx512_fft().lanes > 1)
      return avx512_fft();
  }
  if (targets & HWY_AVX2) {
    if (avx2_fft().lanes > 1)
      return avx2_fft();
  }
  if (targets & (HWY_SSE2 | HWY_SSSE3 | HWY_SSE4)) {
    if (sse2_fft().lanes > 1)
      return sse2_fft();
  }
#else
  if (native_target_fft().lanes > 1)
    return native_target_fft();
#endif
#endif
  return scalar_fft();
}
} // namespace detail

namespace {
const detail::FftBackend& backend(FftProfile profile) noexcept {
  switch (profile) {
    case FftProfile::scalar: return detail::scalar_fft();
#if NEO_MV_FFT_X86_TARGETS
    case FftProfile::sse2: return detail::sse2_fft();
    case FftProfile::avx2: return detail::avx2_fft();
    case FftProfile::avx512: return detail::avx512_fft();
#endif
    case FftProfile::native:
    default:
      return detail::native_fft();
  }
}
template <class T>
std::size_t checked_count(int width, int height) {
  const auto limit = (std::min)(std::vector<T>{}.max_size(),
                                static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(T));
  if (std::size_t(width) > limit / std::size_t(height))
    throw std::overflow_error("DepanEstimate FFT storage is unrepresentable");
  return std::size_t(width) * std::size_t(height);
}
void validate(const std::vector<float>& values, SampleValidator validator) {
  if (validator) {
    validator(values.data(), values.size());
    return;
  }
  for (float value : values)
    finite(value);
}
void validate(const std::vector<std::complex<float>>& values, SampleValidator validator) {
  if (validator) {
    // std::complex<float> arrays permit this interleaved scalar access.
    // checked_count bounds the byte size, so doubling the count is safe.
    validator(reinterpret_cast<const float*>(values.data()), values.size() * 2);
    return;
  }
  for (auto value : values) {
    finite(value.real());
    finite(value.imag());
  }
}
} // namespace
int fft_lanes(FftProfile profile) noexcept {
  return backend(profile).lanes;
}
const char* fft_profile_name(FftProfile profile) noexcept {
  switch (profile) {
    case FftProfile::scalar: return "pocketfft-scalar";
    case FftProfile::sse2: return "pocketfft-sse2";
    case FftProfile::avx2: return "pocketfft-avx2";
    case FftProfile::avx512: return "pocketfft-avx512";
    case FftProfile::native:
    default:
      return fft_lanes(profile) > 1 ? "pocketfft-native" : "pocketfft-scalar";
  }
}
FftPlan::FftPlan(int width, int height, FftProfile profile) : width_(width), height_(height), profile_(profile) {
  if (width < 2 || width % 2 || height < 2)
    throw std::invalid_argument("invalid DepanEstimate FFT dimensions");
  real_count_ = checked_count<float>(width, height);
  complex_count_ = checked_count<std::complex<float>>(width / 2 + 1, height);
}
std::vector<std::complex<float>> FftPlan::forward(const std::vector<float>& input, SampleValidator validator) const {
  if (input.size() != real_count_)
    throw std::invalid_argument("incorrect DepanEstimate real input size");
  validate(input, validator);
  std::vector<std::complex<float>> output(complex_count_);
  backend(profile_).forward(width_, height_, input.data(), output.data());
  validate(output, validator);
  return output;
}
std::vector<float> FftPlan::inverse(const std::vector<std::complex<float>>& input, SampleValidator validator) const {
  if (input.size() != complex_count_)
    throw std::invalid_argument("incorrect DepanEstimate half-spectrum size");
  validate(input, validator);
  std::vector<float> output(real_count_);
  backend(profile_).inverse(width_, height_, input.data(), output.data());
  validate(output, validator);
  return output;
}
std::vector<float> FftPlan::correlate(const std::vector<float>& current, const std::vector<float>& previous) const {
  auto a = forward(current);
  const auto b = forward(previous);
  for (std::size_t i = 0; i < a.size(); ++i) {
    const float ar = a[i].real(), ai = a[i].imag(), br = b[i].real(), bi = b[i].imag();
    a[i] = {add(mul(ar, br), mul(ai, bi)), sub(mul(ar, bi), mul(ai, br))};
  }
  return inverse(a);
}
} // namespace neo_mv::depan::estimate
