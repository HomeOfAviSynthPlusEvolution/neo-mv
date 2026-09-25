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
  const double normalization = 0x1.6a09e667f3bcdp-1 / double(width_ * height_);
  for (std::size_t i = 1; i < output.size(); ++i) {
    const double value = transformed_[i] * normalization;
    const double low = std::floor(value), fraction = value - low;
    int quantized = int(low) + (fraction > 0.5 || (fraction == 0.5 && int(low) % 2 != 0));
    // Twice the proved FFT enclosure also covers rounding of the boundary
    // distance subtraction. Ambiguous cells use exact integer refinement.
    if (std::abs(value - (low + 0.5)) <= 2 * error_)
      quantized = dct_detail::refine_ac(samples_.data(), width_, height_, int(i % width_), int(i / width_), value);
    output[i] = std::clamp(quantized + midpoint_, 0, maximum_);
  }
}
} // namespace neo_mv
