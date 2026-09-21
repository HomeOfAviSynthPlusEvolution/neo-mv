#include "core/depan/estimate_fft.hpp"
#include "core/depan/estimate_fft_backend.hpp"
#include "core/depan/numeric.hpp"
#include <limits>

namespace neo_mv::depan::estimate {
namespace {
const detail::FftBackend& backend(FftProfile profile) noexcept {
  return profile == FftProfile::native ? detail::native_fft() : detail::scalar_fft();
}
template <class T>
std::size_t checked_count(int width, int height) {
  const auto limit = (std::min)(std::vector<T>{}.max_size(),
                                static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(T));
  if (std::size_t(width) > limit / std::size_t(height))
    throw std::overflow_error("DepanEstimate FFT storage is unrepresentable");
  return std::size_t(width) * std::size_t(height);
}
void validate(const std::vector<float>& values) {
  for (float value : values)
    finite(value);
}
void validate(const std::vector<std::complex<float>>& values) {
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
  return fft_lanes(profile) > 1 ? "pocketfft-native" : "pocketfft-scalar";
}
FftPlan::FftPlan(int width, int height, FftProfile profile) : width_(width), height_(height), profile_(profile) {
  if (width < 2 || width % 2 || height < 2)
    throw std::invalid_argument("invalid DepanEstimate FFT dimensions");
  real_count_ = checked_count<float>(width, height);
  complex_count_ = checked_count<std::complex<float>>(width / 2 + 1, height);
}
std::vector<std::complex<float>> FftPlan::forward(const std::vector<float>& input) const {
  if (input.size() != real_count_)
    throw std::invalid_argument("incorrect DepanEstimate real input size");
  validate(input);
  std::vector<std::complex<float>> output(complex_count_);
  backend(profile_).forward(width_, height_, input.data(), output.data());
  validate(output);
  return output;
}
std::vector<float> FftPlan::inverse(const std::vector<std::complex<float>>& input) const {
  if (input.size() != complex_count_)
    throw std::invalid_argument("incorrect DepanEstimate half-spectrum size");
  validate(input);
  std::vector<float> output(real_count_);
  backend(profile_).inverse(width_, height_, input.data(), output.data());
  validate(output);
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
