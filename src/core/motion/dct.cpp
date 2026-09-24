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
  // Unnormalized 2-D DC is 4 * sum(input), so its quantizer is
  // trunc(sum(input) / (2 * area)). Integer samples are exactly represented
  // in input_; the sum is at most 128 * 128 * 65535 < 2^30.
  // Computing DC through float can lose a one-level pixel difference and
  // incorrectly cross an integer boundary before truncation.
  std::int64_t sum = 0;
  for (const auto sample : input_)
    sum += static_cast<std::int64_t>(sample);
  output[0] = std::clamp(int(sum / (2 * width_ * height_)) + midpoint_, 0, maximum_);
  for (std::size_t i = 1; i < output.size(); ++i) {
    const float value = transformed_[i] * ac;
    // Round the computed AC coefficient to nearest, ties to even. The FFT
    // approximation itself can still cross an exact rounding boundary.
    const float low = std::floor(value), fraction = value - low;
    int quantized = int(low);
    if (fraction > 0.5f || (fraction == 0.5f && quantized % 2 != 0))
      ++quantized;
    output[i] = std::clamp(quantized + midpoint_, 0, maximum_);
  }
}
} // namespace neo_mv
