#include "core/depan/estimate_fft_backend.hpp"

// Built multiple times with separate dependency namespaces. No shared
// cache or worker pool exists between targets.
#if defined(NEO_MV_FFT_SCALAR)
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_scalar_c90e55b3
#ifndef POCKETFFT_NO_VECTORS
#define POCKETFFT_NO_VECTORS
#endif
#define BACKEND_FN scalar_fft
#elif defined(NEO_MV_FFT_SSE2)
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_sse2_c90e55b3
#define BACKEND_FN sse2_fft
#elif defined(NEO_MV_FFT_AVX2)
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_avx2_c90e55b3
#define BACKEND_FN avx2_fft
#elif defined(NEO_MV_FFT_AVX512)
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_avx512_c90e55b3
#define BACKEND_FN avx512_fft
#else
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_native_c90e55b3
#define BACKEND_FN native_target_fft
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

const FftBackend& BACKEND_FN() noexcept {
  static const FftBackend backend{int(pf::detail::VLEN<float>::val), forward, inverse};
  return backend;
}
} // namespace neo_mv::depan::estimate::detail
