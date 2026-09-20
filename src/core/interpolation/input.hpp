#pragma once

#include "core/flow/sampling.hpp"
#include "core/mask/numeric.hpp"
#include "core/render/frame.hpp"

namespace neo_mv {

namespace interpolation_detail {
inline int validate_pair_metadata(const AnalysisMetadata& backward, const AnalysisMetadata& forward) {
  if (!valid_analysis_metadata(backward) || !valid_analysis_metadata(forward) || backward.delta <= 0 ||
      std::int64_t(forward.delta) != -std::int64_t(backward.delta) || !same_render_analysis(backward, forward, false))
    throw std::invalid_argument("interpolation requires matching positive/negative vector descriptors");
  return backward.delta;
}
} // namespace interpolation_detail

enum class PairMode { fallback, basic, extra };
struct PairSelection {
  PairMode mode = PairMode::fallback;
  AnalysisField B, F, BB, FF;
};

// Direction 0 is bw (+d), direction 1 is fw (-d). No field's nominal target
// participates in eligibility: image dependencies belong to the selected pair.
template <class T>
class InterpolationInputPlan {
  RenderVideo video_;
  std::array<AnalysisMetadata, 2> metadata_;
  SamplingGeometry sampling_;
  int distance_;
  SceneClassifier scene_;

public:
  InterpolationInputPlan(RenderVideo clip, const SuperPlan<T>& super, std::int64_t super_frames,
                         std::array<AnalysisMetadata, 2> metadata, std::array<std::int64_t, 2> vector_frames,
                         std::int64_t thscd1 = 400, double thscd2 = 51)
      : video_(clip), metadata_(metadata), sampling_(super_sampling_geometry(super)[0]),
        distance_(interpolation_detail::validate_pair_metadata(metadata[0], metadata[1])),
        scene_(scene_descriptor(metadata[1]), thscd1, mask_detail::binary32(thscd2)) {
    if (clip.frames <= 0 || clip.frames > INT32_MAX)
      throw std::invalid_argument("interpolation frame count outside supported range");
    for (int direction = 0; direction < 2; ++direction)
      validate_render_geometry(clip, super, super_frames, metadata_[direction], vector_frames[direction]);
    for (int k = 0; k < plane_count(); ++k) {
      const auto geometry = phase_geometry(k);
      // Reuse creation admission of logical phases and zero displacement only.
      FlowSamplingPlan(geometry, clip.width / geometry.ratio_x, clip.height / geometry.ratio_y, 0);
    }
  }
  const RenderVideo& video() const { return video_; }
  const AnalysisMetadata& metadata(std::size_t direction) const { return metadata_.at(direction); }
  int distance() const { return distance_; }
  std::int64_t frames() const { return video_.frames; }
  int plane_count() const { return video_.chroma ? 3 : 1; }
  RenderPhaseGeometry phase_geometry(int k) const { return render_phase_geometry(sampling_, k); }
  const SceneThresholds& thresholds() const { return scene_.thresholds; }
  bool in_range(std::int64_t left, std::int64_t right) const {
    return left >= 0 && left < frames() && right >= 0 && right < frames();
  }
  bool eligible(const AnalysisField& field, std::size_t direction) const {
    const auto& saved = metadata_.at(direction);
    if (field.state == FieldState::invalid_metadata)
      return false;
    if (!valid_analysis_metadata(field.metadata) || !same_render_analysis(saved, field.metadata))
      throw std::invalid_argument("interpolation vector metadata changed");
    return scene_(field) == 0;
  }
  bool main_eligible(const AnalysisField& backward, const AnalysisField& forward) const {
    const bool b = eligible(backward, 0), f = eligible(forward, 1);
    return b && f;
  }
  bool extra_eligible(const AnalysisField& backward, const AnalysisField& forward) const {
    return main_eligible(backward, forward);
  }
  template <class ReadField>
  PairSelection select(std::int64_t left, std::int64_t right, ReadField&& read, bool extras = true) const {
    PairSelection result;
    if (!in_range(left, right))
      return result;
    if (right - left != distance_)
      throw std::invalid_argument("interpolation pair does not match temporal distance");
    result.B = read(0, left);
    result.F = read(1, right);
    if (!main_eligible(result.B, result.F))
      return result;
    result.mode = PairMode::basic;
    if (extras) {
      result.BB = read(0, right);
      result.FF = read(1, left);
      if (extra_eligible(result.BB, result.FF))
        result.mode = PairMode::extra;
    }
    return result;
  }
  void validate_image(const RenderImage<T>& image) const {
    if (!image.plan)
      throw std::invalid_argument("missing interpolation Super plan");
    const auto& p = image.plan->params();
    if (p.width != video_.width || p.height != video_.height || image.plan->bits() != video_.bits ||
        p.chroma != video_.chroma || p.ratio_x != video_.ratio_x || p.ratio_y != video_.ratio_y)
      throw std::invalid_argument("interpolation Super format changed");
    const auto actual = super_sampling_geometry(*image.plan)[0];
    if (actual.pel != sampling_.pel)
      throw std::invalid_argument("interpolation Super pel changed");
    for (int k = 0; k < plane_count(); ++k) {
      const auto& expected = sampling_.planes[k];
      const auto& other = actual.planes[k];
      if (other.pad_x != expected.pad_x || other.pad_y != expected.pad_y ||
          other.current.width != expected.current.width || other.current.height != expected.current.height ||
          image.planes[k].pel != sampling_.pel)
        throw std::invalid_argument("interpolation Super level-zero geometry changed");
      for (int a = 0; a < sampling_.pel * sampling_.pel; ++a) {
        const auto view = image.planes[k].planes[a];
        validate_plane(view);
        if (other.reference[a].width != expected.reference[a].width ||
            other.reference[a].height != expected.reference[a].height || view.width() != expected.reference[a].width ||
            view.height() != expected.reference[a].height)
          throw std::invalid_argument("interpolation Super phase domain or storage changed");
      }
    }
  }
};

} // namespace neo_mv
