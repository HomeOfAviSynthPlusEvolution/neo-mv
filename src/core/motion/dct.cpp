#include "core/motion/dct.hpp"
#include "core/motion/dct_fft.hpp"
#include "core/motion/dct_refine.hpp"
#include <algorithm>

namespace neo_mv {
DctWorkspace::DctWorkspace(int width, int height, int bits, bool simd)
    : width_(width), height_(height), simd_(simd), error_(dct_detail::ac_error_bound(width, height, bits)) {
  maximum_ = (1 << bits) - 1;
  midpoint_ = 1 << (bits - 1);
  scale_ = int(std::sqrt(double(width * height)) + 0.5);
  const auto size = std::size_t(width) * height;
  samples_.resize(size);
  input_.resize(size);
  rows_.resize(size);
  transformed_.resize(size);
  source_.resize(size);
  reference_.resize(size);
}

void DctWorkspace::transform(std::vector<int>& output) {
  dct_detail::transform_block(width_, height_, input_.data(), rows_.data(), transformed_.data(), simd_);
  // Unnormalized DC is 4*sum(samples); its quantizer truncates sum/(2*area).
  std::int64_t sum = 0;
  for (auto sample : samples_)
    sum += sample;
  output[0] = std::clamp(int(sum / (2 * width_ * height_)) + midpoint_, 0, maximum_);
  dct_detail::quantize_ac(samples_.data(), width_, height_, transformed_.data(), output.data(), maximum_, error_,
                          simd_);
}
} // namespace neo_mv
