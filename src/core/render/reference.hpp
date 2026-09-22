#pragma once

#include "core/motion/scene_classification.hpp"
#include "core/motion/super_input.hpp"

#include <optional>

namespace neo_mv {

// Host-neutral description of the visible clip. Fixed format is established by
// the adapter; frame rate deliberately does not participate in compatibility.
struct RenderVideo {
  std::int32_t width, height, bits;
  bool chroma;
  std::int32_t ratio_x, ratio_y;
  std::int64_t frames;
};
struct RenderCoverage {
  std::int32_t width, height;
};
using RenderPlanes = std::array<bool, 3>;

// Levels never participates in render compatibility. Delta may differ between
// Degrain members, but must remain fixed for each member across output frames.
inline bool same_render_analysis(const AnalysisMetadata& a, const AnalysisMetadata& b, bool compare_delta = true) {
  if (a.chroma != b.chroma)
    return false;
  for (auto scalar : field_detail::scalars) {
    if (scalar.member == &AnalysisMetadata::levels || (!compare_delta && scalar.member == &AnalysisMetadata::delta))
      continue;
    if (a.*(scalar.member) != b.*(scalar.member))
      return false;
  }
  return true;
}

// Structural admission only: the render sampling kernel must additionally
// admit the whole public vector domain in every processed plane. Super block
// hints and analysis Levels do not determine the consumed grid or image level.
template <class T>
RenderCoverage validate_render_geometry(const RenderVideo& clip, const SuperPlan<T>& super, std::int64_t super_frames,
                                        const AnalysisMetadata& m, std::int64_t vector_frames,
                                        RenderPlanes processed = {true, true, true}) {
  const auto& p = super.params();
  const auto& level = super.geometry().planes[0].levels[0];
  if (clip.frames <= 0 || super_frames != clip.frames || vector_frames < clip.frames || clip.width != p.width ||
      clip.height != p.height || clip.bits != super.bits() || clip.chroma != p.chroma || clip.ratio_x != p.ratio_x ||
      clip.ratio_y != p.ratio_y)
    throw std::invalid_argument("render clip and Super video descriptions do not match");
  if (!valid_analysis_metadata(m) || !geometry_detail::block_pair(m.block_width, m.block_height) ||
      m.width != level.width || m.height != level.height || m.real_width != clip.width ||
      m.real_height != clip.height || m.pad_x != p.pad_x || m.pad_y != p.pad_y || m.pel != p.pel ||
      (m.chroma && (!clip.chroma || m.ratio_x != clip.ratio_x || m.ratio_y != clip.ratio_y)))
    throw std::invalid_argument("analysis metadata does not match render geometry");
  if (clip.chroma && (processed[1] || processed[2]) &&
      (m.block_width % clip.ratio_x || m.block_height % clip.ratio_y || m.overlap_x % clip.ratio_x ||
       m.overlap_y % clip.ratio_y))
    throw std::invalid_argument("render blocks or overlap are not chroma aligned");
  const auto width = std::int64_t(m.blocks_x) * (m.block_width - m.overlap_x) + m.overlap_x;
  const auto height = std::int64_t(m.blocks_y) * (m.block_height - m.overlap_y) + m.overlap_y;
  if (width < clip.width || width > m.width || height < clip.height || height > m.height)
    throw std::invalid_argument("render grid does not cover visible video within the working image");
  return {static_cast<std::int32_t>(width), static_cast<std::int32_t>(height)};
}

// F and S(z) from Phase 2 reference availability. This is intentionally not the
// sequential integer area/depth scaling used by motion search. The saved
// analysis precision is independent of the render sample type.
class RenderThresholdScale {
  double factor_;

public:
  explicit RenderThresholdScale(const AnalysisMetadata& m) {
    if (!valid_analysis_metadata(m))
      throw std::invalid_argument("invalid render threshold analysis metadata");
    const double area = double(std::int64_t(m.block_width) * m.block_height) / 64.0;
    const double chroma = m.chroma ? 1.0 + 2.0 / double(m.ratio_x * m.ratio_y) : 1.0;
    const double depth = double((1 << std::min(16, m.bits)) - 1) / 255.0;
    factor_ = (area * chroma) * depth;
  }

  std::int64_t operator()(std::int64_t value) const {
    if (value < 0)
      throw std::invalid_argument("render threshold must be nonnegative");
    const double scaled = double(value) * factor_ + 0.5;
    if (!std::isfinite(scaled) || scaled >= 0x1p63)
      throw std::overflow_error("render threshold exceeds int64 range");
    return static_cast<std::int64_t>(scaled);
  }
};

// Construction receives metadata from a metadata-only read of frame zero.
// Evaluation receives the complete decoder result, so malformed arrays cannot
// be hidden by an unavailable temporal reference or by a zero user weight.
class ReferenceAvailability {
  AnalysisMetadata saved_;
  std::int64_t frames_;
  SceneClassifier scene_;

public:
  ReferenceAvailability(AnalysisMetadata saved, std::int64_t frames, std::int64_t thscd1, double thscd2)
      : saved_(saved), frames_(frames), scene_(scene_descriptor(saved), thscd1, thscd2) {
    if (!valid_analysis_metadata(saved_) || frames_ <= 0)
      throw std::invalid_argument("invalid reference creation descriptor");
  }

  const AnalysisMetadata& metadata() const { return saved_; }
  const SceneThresholds& thresholds() const { return scene_.thresholds; }

  // An engaged result is the exact reference index, including n when d=0.
  // All checks precede temporal fallback. No reference image is accessed here.
  std::optional<std::int64_t> operator()(const AnalysisField& field, std::int64_t n,
                                         SceneClassifier::Counter count = scalar_scene_count) const {
    if (n < 0 || n >= frames_)
      throw std::invalid_argument("render frame index outside clip");
    if (field.state == FieldState::invalid_metadata)
      return std::nullopt;
    if (!valid_analysis_metadata(field.metadata) || !same_render_analysis(saved_, field.metadata))
      throw std::invalid_argument("render analysis metadata changed");
    // SceneClassifier validates all complete entries, including those after a
    // scene cut has already been established. MetadataOnly remains unavailable.
    const bool changed = scene_(field, count) != 0;
    if (field.state == FieldState::metadata_only)
      return std::nullopt;
    if (saved_.delta > 0 && n > INT64_MAX - saved_.delta)
      throw std::overflow_error("reference frame index exceeds int64 range");
    const auto reference = n + std::int64_t(saved_.delta);
    if (changed || reference < 0 || reference >= frames_)
      return std::nullopt;
    return reference;
  }
};

} // namespace neo_mv
