#include "core/depan/estimate_fft_backend.hpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

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
#if defined(NEO_MV_FFT_AVX512)
  // Padding improves measured AVX512 throughput for these 4 KiB-multiple
  // row strides; cache-set contention is a suspected cause. Other layouts
  // avoid the extra allocation and copy.
  if (width % 1024 == 0 && height >= 16) {
    const std::size_t pitch = std::size_t(width) + 16;
    const auto limit = (std::min)(std::vector<float>{}.max_size(),
        std::size_t(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(float));
    if (pitch <= limit / std::size_t(height)) {
      // The inverse transform writes every active sample before the copy.
      // Padding is never read, so initializing the scratch buffer is redundant.
      std::unique_ptr<float[]> temporary(new float[pitch * std::size_t(height)]);
      const pf::stride_t padded{std::ptrdiff_t(pitch) * std::ptrdiff_t(sizeof(float)),
                                 std::ptrdiff_t(sizeof(float))};
      pf::c2r(shape, complex, padded, axes, pf::BACKWARD, input, temporary.get(), 1.0f, 1);
      for (int y = 0; y < height; ++y)
        std::copy_n(temporary.get() + std::size_t(y) * pitch, width, output + std::size_t(y) * width);
      return;
    }
  }
#endif
  pf::c2r(shape, complex, real, axes, pf::BACKWARD, input, output, 1.0f, 1);
}
} // namespace

const FftBackend& BACKEND_FN() noexcept {
  static const FftBackend backend{int(pf::detail::VLEN<float>::val), forward, inverse};
  return backend;
}
} // namespace neo_mv::depan::estimate::detail
