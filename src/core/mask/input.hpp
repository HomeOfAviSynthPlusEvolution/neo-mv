#pragma once

#include "core/mask/numeric.hpp"
#include "core/motion/scene_classification.hpp"

namespace neo_mv {

struct MaskParameters {
  double ml = 100, gamma = 1, time = 100, scval = 0;
  std::int64_t thscd1 = 400;
  double thscd2 = 51;
};

// Immutable host-neutral mask admission. Carrier pixels, Super geometry and
// temporal reference availability do not participate in this contract.
template <class T>
class MaskInputPlan {
  AnalysisMetadata metadata_;
  float f_ = 0, gamma_ = 0;
  int time256_ = 0;
  T fallback_{};
  SceneClassifier scene_;

public:
  MaskInputPlan(AnalysisMetadata metadata, std::int64_t frames, MaskParameters parameters = {})
      : metadata_(metadata),
        scene_(scene_descriptor(metadata), parameters.thscd1, mask_detail::binary32(parameters.thscd2)) {
    if (frames <= 0 || !valid_analysis_metadata(metadata_))
      throw std::invalid_argument("invalid mask creation descriptor");
    mask_detail::validate_storage<T>(metadata_.bits);
    const auto width =
        std::int64_t(metadata_.blocks_x) * (metadata_.block_width - metadata_.overlap_x) + metadata_.overlap_x;
    const auto height =
        std::int64_t(metadata_.blocks_y) * (metadata_.block_height - metadata_.overlap_y) + metadata_.overlap_y;
    if (width < metadata_.real_width || width > metadata_.width || height < metadata_.real_height ||
        height > metadata_.height)
      throw std::invalid_argument("mask grid does not cover visible video within the working image");

    const float ml = mask_detail::binary32(parameters.ml);
    gamma_ = mask_detail::binary32(parameters.gamma);
    const float scval = mask_detail::binary32(parameters.scval);
    if (ml <= 0 || gamma_ < 0)
      throw std::invalid_argument("mask ml must be positive and gamma nonnegative");
    f_ = 1.0f / ml;
    if (!std::isfinite(f_))
      throw std::invalid_argument("mask reciprocal normalization is not finite");
    if (!std::isfinite(parameters.time) || parameters.time < 0 || parameters.time > 100)
      throw std::invalid_argument("mask time must be finite and in [0,100]");
    time256_ = static_cast<int>((parameters.time * 256.0) / 100.0);
    if constexpr (std::is_same_v<T, float>) {
      fallback_ = scval;
    } else {
      const double q = std::trunc(static_cast<double>(scval + 0.5f));
      const auto maximum = (std::uint32_t{1} << metadata_.bits) - 1;
      if (!std::isfinite(q) || q < 0 || q > maximum)
        throw std::invalid_argument("mask fallback sample is outside output precision");
      fallback_ = static_cast<T>(q);
    }
  }

  const AnalysisMetadata& metadata() const { return metadata_; }
  float f() const { return f_; }
  float gamma() const { return gamma_; }
  int time256() const { return time256_; }
  T fallback() const { return fallback_; }
  int bits() const { return metadata_.bits; }

  bool eligible(const AnalysisField& field) const {
    if (field.state == FieldState::invalid_metadata)
      return false;
    if (!valid_analysis_metadata(field.metadata))
      throw std::invalid_argument("mask field state contradicts invalid metadata");
    for (auto scalar : field_detail::scalars) {
      if (scalar.member != &AnalysisMetadata::levels && metadata_.*(scalar.member) != field.metadata.*(scalar.member))
        throw std::invalid_argument("mask analysis metadata changed");
    }
    if (metadata_.chroma != field.metadata.chroma)
      throw std::invalid_argument("mask analysis chroma changed");
    // The classifier checks every entry even after a scene cut is known.
    return scene_(field) == 0;
  }
};

} // namespace neo_mv
