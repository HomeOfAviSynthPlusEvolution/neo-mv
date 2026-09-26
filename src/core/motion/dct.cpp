#include "core/motion/dct.hpp"
#include "core/motion/dct_fft.hpp"

namespace neo_mv {
DctWorkspace::DctWorkspace(int width, int height, int bits, bool simd) : width_(width), height_(height), simd_(simd) {
  // Validate before shifts, products, or allocating buffers.
  dct_detail::cosine_table(width);
  dct_detail::cosine_table(height);
  if (bits < 8 || bits > 16)
    throw std::invalid_argument("DCT requires 8-16 bit integer samples");
  maximum_ = (1 << bits) - 1;
  midpoint_ = 1 << (bits - 1);
  scale_ = int(std::sqrt(double(width * height)) + 0.5);
  stride_ = dct_detail::padded_stride(width);
  const auto scratch = stride_ * dct_detail::padded_stride(height);
  input_.resize(scratch);
  rows_.resize(scratch);
  transformed_.resize(scratch);
  source_.resize(width * height);
  reference_.resize(width * height);
}

void DctWorkspace::transform(std::vector<int>& output) {
  dct_detail::transform_block(width_, height_, input_.data(), rows_.data(), transformed_.data(), simd_);
  dct_detail::quantize_ac(width_, height_, transformed_.data(), output.data(), maximum_, simd_);
  // Preserve exact DC even when a large block's sum cannot fit in binary32.
  output[0] = midpoint_ + int(sum_ / (2 * width_ * height_));
}
} // namespace neo_mv
