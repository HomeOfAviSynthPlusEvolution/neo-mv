#include "core/motion/dct.hpp"
#include <algorithm>

// A distinct namespace avoids ODR collisions with the DePan FFT backends.
// One thread, fixed arithmetic across the scalar and Highway search paths.
#define POCKETFFT_NAMESPACE neo_mv_pocketfft_dct_c90e55b3
#define POCKETFFT_NO_VECTORS
#define POCKETFFT_NO_MULTITHREADING
#define POCKETFFT_CACHE_SIZE 0
#include <pocketfft_hdronly.h>

namespace neo_mv {
DctWorkspace::DctWorkspace(int width, int height, int bits) : width_(width), height_(height) {
  if (width < 1 || width > 128 || height < 1 || height > 128 || bits < 8 || bits > 16)
    throw std::invalid_argument("DCT requires blocks up to 128x128 and 8-16 bit integer samples");
  maximum_ = (1 << bits) - 1;
  midpoint_ = 1 << (bits - 1);
  scale_ = int(std::sqrt(float(width * height)) + 0.5f);
  const auto size = std::size_t(width) * height;
  input_.resize(size);
  transformed_.resize(size);
  source_.resize(size);
  reference_.resize(size);
}

void DctWorkspace::transform(std::vector<int>& output) {
  namespace pf = neo_mv_pocketfft_dct_c90e55b3;
  const pf::shape_t shape{std::size_t(height_), std::size_t(width_)}, axes{0, 1};
  const pf::stride_t strides{std::ptrdiff_t(width_) * std::ptrdiff_t(sizeof(float)), std::ptrdiff_t(sizeof(float))};
  pf::dct(shape, strides, strides, axes, 2, input_.data(), transformed_.data(), 1.0f, false, 1);
  const float ac = 0.7071067811865475244f / float(width_ * height_);
  const float dc = 0.125f / float(width_ * height_);
  for (std::size_t i = 0; i < output.size(); ++i) {
    const float value = transformed_[i] * (i == 0 ? dc : ac);
    // All intermediates are finite and bounded by 4 * 16384 * 65535.
    // AC rounds to nearest, ties to even, independent of the host FP mode;
    // DC truncates towards zero, as in the upstream FFT implementation.
    int quantized;
    if (i == 0)
      quantized = int(value);
    else {
      const float low = std::floor(value), fraction = value - low;
      quantized = int(low);
      if (fraction > 0.5f || (fraction == 0.5f && quantized % 2 != 0))
        ++quantized;
    }
    output[i] = std::clamp(quantized + midpoint_, 0, maximum_);
  }
}
} // namespace neo_mv
