#include "core/depan/estimate_fft.hpp"
#include "core/depan/numeric.hpp"
#include <limits>

// This private namespace and single translation unit isolate the dependency
// configuration from other plugins; no process-wide cache or worker pool exists.
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_c90e55b3
#define POCKETFFT_NO_VECTORS
#define POCKETFFT_NO_MULTITHREADING
#define POCKETFFT_CACHE_SIZE 0
#include <pocketfft_hdronly.h>

namespace neo_mv::depan::estimate {
namespace pf = neo_mv_pocketfft_c90e55b3;
namespace {
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
FftPlan::FftPlan(int width, int height) : width_(width), height_(height) {
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
  const pf::shape_t shape{std::size_t(height_), std::size_t(width_)}, axes{0, 1};
  const pf::stride_t real{std::ptrdiff_t(width_) * std::ptrdiff_t(sizeof(float)), std::ptrdiff_t(sizeof(float))};
  const pf::stride_t complex{std::ptrdiff_t(width_ / 2 + 1) * std::ptrdiff_t(sizeof(std::complex<float>)), std::ptrdiff_t(sizeof(std::complex<float>))};
  pf::r2c(shape, real, complex, axes, pf::FORWARD, input.data(), output.data(), 1.0f, 1);
  validate(output);
  return output;
}
std::vector<float> FftPlan::inverse(const std::vector<std::complex<float>>& input) const {
  if (input.size() != complex_count_)
    throw std::invalid_argument("incorrect DepanEstimate half-spectrum size");
  validate(input);
  std::vector<float> output(real_count_);
  const pf::shape_t shape{std::size_t(height_), std::size_t(width_)}, axes{0, 1};
  const pf::stride_t real{std::ptrdiff_t(width_) * std::ptrdiff_t(sizeof(float)), std::ptrdiff_t(sizeof(float))};
  const pf::stride_t complex{std::ptrdiff_t(width_ / 2 + 1) * std::ptrdiff_t(sizeof(std::complex<float>)), std::ptrdiff_t(sizeof(std::complex<float>))};
  pf::c2r(shape, complex, real, axes, pf::BACKWARD, input.data(), output.data(), 1.0f, 1);
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
