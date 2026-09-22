#pragma once

#include "core/render/degrain_weights.hpp"

namespace neo_mv {

template <class T>
struct WeightedReferenceBlock {
  bool available;
  span2d::Plane<const T> samples;
};

// Inputs are complete blocks from render sampling. The frame composer remains
// responsible for validating every field and its whole-domain geometry before
// generating these blocks, independently of availability and user weights.
template <class T, bool Validated = false>
void weighted_render_block(span2d::Plane<const T> centre, const std::vector<WeightedReferenceBlock<T>>& references,
                           const DegrainWeights& weights, span2d::Plane<T> output, int bits) {
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  if constexpr (!Validated) {
    if (references.size() < 2 || references.size() > 50 || references.size() % 2 != 0 ||
        references.size() != weights.reference.size() || weights.centre < 0 || weights.centre > 256)
      throw std::invalid_argument("invalid weighted block reference count or centre weight");
    int sum = weights.centre;
    for (std::size_t i = 0; i < references.size(); ++i) {
      const int w = weights.reference[i];
      if (w < 0 || w > 256 || (!references[i].available && w != 0))
        throw std::invalid_argument("invalid weighted block reference weight");
      sum += w;
    }
    if (sum != 256)
      throw std::invalid_argument("weighted block weights must sum to 256");
    validate_plane(output);
    const auto validate_input = [&](span2d::Plane<const T> input) {
      validate_plane(input);
      if (input.width() != output.width() || input.height() != output.height() || active_rows_overlap(input, output))
        throw std::invalid_argument("weighted block geometry mismatch or output alias");
      for (int y = 0; y < input.height(); ++y)
        for (int x = 0; x < input.width(); ++x)
          subpixel_detail::valid_sample(input.row(y)[x], maximum);
    };
    validate_input(centre);
    for (const auto& r : references)
      if (r.available)
        validate_input(r.samples);
  }
  for (int y = 0; y < output.height(); ++y)
    for (int x = 0; x < output.width(); ++x) {
      const T c = centre.row(y)[x];
      if constexpr (std::is_same_v<T, float>) {
        float z = c * static_cast<float>(weights.centre);
        if (!std::isfinite(z))
          throw std::overflow_error("non-finite weighted centre sample");
        for (std::size_t i = 0; i < references.size(); ++i) {
          const auto& r = references[i];
          const float sample = r.available ? r.samples.row(y)[x] : c;
          const float product = sample * static_cast<float>(weights.reference[i]);
          z = z + product;
          if (!std::isfinite(product) || !std::isfinite(z))
            throw std::overflow_error("non-finite weighted reference intermediate");
        }
        output.row(y)[x] = z / 256.0f;
      } else {
        std::int64_t z = 128 + std::int64_t(c) * weights.centre;
        for (std::size_t i = 0; i < references.size(); ++i) {
          const auto& r = references[i];
          const T sample = r.available ? r.samples.row(y)[x] : c;
          z += std::int64_t(sample) * weights.reference[i];
        }
        output.row(y)[x] = static_cast<T>(z / 256);
      }
    }
}

} // namespace neo_mv
