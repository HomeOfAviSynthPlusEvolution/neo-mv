#pragma once
#include <complex>

namespace neo_mv::depan::estimate::detail {
struct FftBackend {
  int lanes;
  void (*forward)(int width, int height, const float* input, std::complex<float>* output);
  void (*inverse)(int width, int height, const std::complex<float>* input, float* output);
};
const FftBackend& scalar_fft() noexcept;
const FftBackend& native_fft() noexcept;
} // namespace neo_mv::depan::estimate::detail
