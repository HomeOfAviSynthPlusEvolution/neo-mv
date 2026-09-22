#pragma once

#include "core/interpolation/input.hpp"
#include "core/mask/scores.hpp"

namespace neo_mv {

struct DenseInterpolationFields {
  int width, height;
  DenseFlowField B, F;
  std::optional<DenseFlowField> BB, FF;
  OverwriteVector<std::uint8_t> mB, mF;
};
using DenseInterpolationField = DenseInterpolationFields;

template <class Resampler = GridResamplingPlan>
class DenseInterpolationPlan {
  std::array<AnalysisMetadata, 2> metadata_;
  DenseFlowPlan<Resampler> backward_, forward_;
  Resampler masks_;
  float f_;

  static float normalization(double ml) {
    const float value = mask_detail::binary32(ml);
    if (value <= 0)
      throw std::invalid_argument("interpolation ml must be positive");
    const float reciprocal = 1.0f / value;
    if (!std::isfinite(reciprocal))
      throw std::invalid_argument("interpolation normalization is not finite");
    return reciprocal;
  }
  static AnalysisMetadata mask_metadata(AnalysisMetadata m) {
    // Only the internal event output precision changes. Public fields are
    // validated with their original metadata before this event kernel runs.
    m.bits = 8;
    return m;
  }
  void validate(const MotionGrid& grid, std::size_t direction) const {
    const auto& m = metadata_[direction];
    if (grid.width != m.blocks_x || grid.height != m.blocks_y || grid.values.size() != field_detail::count(m))
      throw std::invalid_argument("inconsistent interpolation motion grid");
    for (std::size_t i = 0; i < grid.values.size(); ++i)
      field_detail::vector(m, grid.values[i], static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
  }
  OverwriteVector<std::uint8_t> mask(const MotionGrid& grid, std::size_t direction, int time256) const {
    const auto small = OcclusionMaskPlan<std::uint8_t>(mask_metadata(metadata_[direction]), f_, 1, time256)
                           .template generate<true>(grid);
    const auto& g = backward_.geometry();
    OverwriteVector<std::uint8_t> output(dense_detail::count(g.width, g.height));
    masks_.template resize<std::uint8_t, true>(dense_detail::plane(small.data(), g.blocks_x, g.blocks_y, small.size()),
                                               dense_detail::plane(output.data(), g.width, g.height, output.size()), 8);
    return output;
  }

public:
  DenseInterpolationPlan(AnalysisMetadata backward, AnalysisMetadata forward, int ratio_x, int ratio_y, double ml = 100)
      : metadata_{backward, forward}, backward_(backward, ratio_x, ratio_y), forward_(forward, ratio_x, ratio_y),
        masks_(backward_.geometry()), f_(normalization(ml)) {
    interpolation_detail::validate_pair_metadata(backward, forward);
    // Validate all required constants at creation, including paths whose
    // eventual output falls back or whose interpolation time is an endpoint.
    OcclusionMaskPlan<std::uint8_t>(mask_metadata(backward), f_, 1, 0);
    OcclusionMaskPlan<std::uint8_t>(mask_metadata(forward), f_, 1, 0);
  }
  const GridResamplingGeometry& geometry() const { return backward_.geometry(); }
  float normalization() const { return f_; }
  template <bool Validated = false>
  DenseInterpolationFields generate(const MotionGrid& backward, const MotionGrid& forward, int time256,
                                    const MotionGrid* extra_backward = nullptr,
                                    const MotionGrid* extra_forward = nullptr) const {
    if (time256 < 0 || time256 > 256 || bool(extra_backward) != bool(extra_forward))
      throw std::invalid_argument("invalid interpolation time or unpaired extra fields");
    if constexpr (!Validated) {
      validate(backward, 0);
      validate(forward, 1);
      if (extra_backward) {
        validate(*extra_backward, 0);
        validate(*extra_forward, 1);
      }
    }
    const auto& g = geometry();
    DenseInterpolationFields result{g.width,
                                    g.height,
                                    backward_.template generate<true>(backward, 0),
                                    forward_.template generate<true>(forward, 0),
                                    {},
                                    {},
                                    mask(backward, 0, 256 - time256),
                                    mask(forward, 1, time256)};
    if (extra_backward) {
      result.BB = backward_.template generate<true>(*extra_backward, 0);
      result.FF = forward_.template generate<true>(*extra_forward, 0);
    }
    return result;
  }
};

} // namespace neo_mv
