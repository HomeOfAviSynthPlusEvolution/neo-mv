#include "core/depan/estimate_fft_backend.hpp"

// Built twice with separate dependency namespaces. Both copies have no shared
// cache or worker pool. Native SIMD uses the compiler's baseline ISA only.
#if NEO_MV_FFT_NATIVE
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_native_c90e55b3
#else
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_c90e55b3
#define POCKETFFT_NO_VECTORS
#endif
#define POCKETFFT_NO_MULTITHREADING
#define POCKETFFT_CACHE_SIZE 0
#include <pocketfft_hdronly.h>

namespace neo_mv::depan::estimate::detail {
namespace {
namespace pf = POCKETFFT_NAMESPACE;
void forward(int width, int height, const float* input, std::complex<float>* output) {
  const pf::shape_t shape{std::size_t(height), std::size_t(width)}, axes{0, 1};
  const pf::stride_t real{std::ptrdiff_t(width) * std::ptrdiff_t(sizeof(float)), std::ptrdiff_t(sizeof(float))};
  const pf::stride_t complex{std::ptrdiff_t(width / 2 + 1) * std::ptrdiff_t(sizeof(std::complex<float>)),
                             std::ptrdiff_t(sizeof(std::complex<float>))};
  pf::r2c(shape, real, complex, axes, pf::FORWARD, input, output, 1.0f, 1);
}
void inverse(int width, int height, const std::complex<float>* input, float* output) {
  const pf::shape_t shape{std::size_t(height), std::size_t(width)}, axes{0, 1};
  const pf::stride_t real{std::ptrdiff_t(width) * std::ptrdiff_t(sizeof(float)), std::ptrdiff_t(sizeof(float))};
  const pf::stride_t complex{std::ptrdiff_t(width / 2 + 1) * std::ptrdiff_t(sizeof(std::complex<float>)),
                             std::ptrdiff_t(sizeof(std::complex<float>))};
  pf::c2r(shape, complex, real, axes, pf::BACKWARD, input, output, 1.0f, 1);
}
} // namespace
#if NEO_MV_FFT_NATIVE
const FftBackend& native_fft() noexcept {
#else
const FftBackend& scalar_fft() noexcept {
#endif
  static const FftBackend backend{int(pf::detail::VLEN<float>::val), forward, inverse};
  return backend;
}
} // namespace neo_mv::depan::estimate::detail
