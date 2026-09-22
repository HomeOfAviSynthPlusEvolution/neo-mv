#pragma once

#include "core/mask/numeric.hpp"
#include "core/render/frame.hpp"
#include "kernels/flow_scalar.hpp"

namespace neo_mv {
struct FlowParameters {
  double time = 100;
  bool fields = false;
  std::int64_t thscd1 = 400;
  double thscd2 = 51;
  std::optional<bool> tff;
};

template <class T, class Kernels = ScalarFlowKernels<T>>
class FlowFramePlan {
  RenderVideo video_;
  SamplingGeometry sampling_;
  ReferenceAvailability availability_;
  bool fields_;
  std::optional<bool> tff_;
  std::vector<typename Kernels::DenseFlow> dense_;
  std::vector<typename Kernels::Sampling> sampling_plans_;

  int plane_count() const { return video_.chroma ? 3 : 1; }
  int field_shift(std::int64_t n, std::optional<bool> current_top, std::optional<bool> reference_top) const {
    if (!field_correction_active())
      return 0;
    if (tff_) {
      current_top = *tff_ != (n % 2 != 0);
      reference_top = !*current_top; // Only odd deltas reach this path.
    }
    if (!current_top || !reference_top)
      throw std::invalid_argument("Flow requires current and reference field parity");
    return *current_top == *reference_top ? 0 : *current_top ? sampling_.pel / 2 : -sampling_.pel / 2;
  }
  void validate_clip(const RenderPixels<T>& clip) const {
    for (int k = 0; k < plane_count(); ++k) {
      validate_plane(clip[k]);
      if (clip[k].width() != sampling_plans_[k].width() || clip[k].height() != sampling_plans_[k].height())
        throw std::invalid_argument("Flow clip storage dimensions changed");
    }
  }
  RenderOutput<T> copy_clip(const RenderPixels<T>& clip) const {
    RenderOutput<T> output;
    for (int k = 0; k < plane_count(); ++k) {
      output.emplace_back(clip[k].width(), clip[k].height());
      auto out = output.back().view();
      for (int y = 0; y < out.height(); ++y)
        std::memcpy(out.row(y).data(), clip[k].row(y).data(), std::size_t(out.width()) * sizeof(T));
    }
    return output;
  }

public:
  FlowFramePlan(RenderVideo clip, const SuperPlan<T>& super, std::int64_t super_frames, AnalysisMetadata metadata,
                std::int64_t vectors_frames, FlowParameters parameters = {})
      : video_(clip), sampling_(super_sampling_geometry(super)[0]),
        availability_(metadata, clip.frames, parameters.thscd1, mask_detail::binary32(parameters.thscd2)),
        fields_(parameters.fields), tff_(parameters.tff) {
    validate_render_geometry(clip, super, super_frames, metadata, vectors_frames);
    if (!std::isfinite(parameters.time) || parameters.time < 0 || parameters.time > 100)
      throw std::invalid_argument("invalid Flow time");
    const int time256 = static_cast<int>((parameters.time * 256.0) / 100.0);
    for (int k = 0; k < plane_count(); ++k) {
      const int rx = k ? clip.ratio_x : 1, ry = k ? clip.ratio_y : 1;
      dense_.emplace_back(metadata, rx, ry);
      sampling_plans_.emplace_back(render_phase_geometry(sampling_, k), clip.width / rx, clip.height / ry, time256);
    }
  }
  const AnalysisMetadata& metadata() const { return availability_.metadata(); }
  bool field_correction_active() const { return fields_ && sampling_.pel > 1 && metadata().delta % 2 != 0; }
  std::optional<std::int64_t> reference(const AnalysisField& field, std::int64_t n) const {
    return availability_(field, n, Kernels::scene_count);
  }

  // Also usable by the host for the current Super needed only to read parity.
  // Validation compares consumed level-zero domains, not pyramid level counts.
  void validate_reference(const RenderImage<T>& image) const {
    if (!image.plan)
      throw std::invalid_argument("missing Flow reference Super plan");
    const auto& p = image.plan->params();
    if (p.width != video_.width || p.height != video_.height || image.plan->bits() != video_.bits ||
        p.chroma != video_.chroma || p.ratio_x != video_.ratio_x || p.ratio_y != video_.ratio_y)
      throw std::invalid_argument("Flow reference Super format changed");
    const auto actual = super_sampling_geometry(*image.plan)[0];
    if (actual.pel != sampling_.pel)
      throw std::invalid_argument("Flow reference Super pel changed");
    for (int k = 0; k < plane_count(); ++k) {
      const auto& expected = sampling_.planes[k];
      const auto& other = actual.planes[k];
      if (other.pad_x != expected.pad_x || other.pad_y != expected.pad_y ||
          other.current.width != expected.current.width || other.current.height != expected.current.height ||
          image.planes[k].pel != sampling_.pel)
        throw std::invalid_argument("Flow reference Super geometry changed");
      for (int a = 0; a < sampling_.pel * sampling_.pel; ++a) {
        const auto view = image.planes[k].planes[a];
        validate_plane(view);
        if (other.reference[a].width != expected.reference[a].width ||
            other.reference[a].height != expected.reference[a].height || view.width() != expected.reference[a].width ||
            view.height() != expected.reference[a].height)
          throw std::invalid_argument("Flow reference phase domain or storage changed");
      }
    }
  }

  RenderOutput<T> render(const RenderPixels<T>& clip, const AnalysisField& field, std::int64_t n,
                         const RenderImage<T>* reference_image = nullptr, std::optional<bool> current_top = {},
                         std::optional<bool> reference_top = {}) const {
    validate_clip(clip);
    if (!reference(field, n))
      return copy_clip(clip);
    if (!reference_image)
      throw std::invalid_argument("missing available Flow reference");
    validate_reference(*reference_image);
    const int shift = field_shift(n, current_top, reference_top);
    std::vector<DenseFlowField> fields;
    for (int k = 0; k < plane_count(); ++k)
      fields.push_back(dense_[k].generate(field.grid, shift));
    // No reference sample is read until every plane's actual coordinates pass.
    for (int k = 0; k < plane_count(); ++k)
      sampling_plans_[k].preflight(fields[k]);
    RenderOutput<T> output;
    for (int k = 0; k < plane_count(); ++k) {
      output.emplace_back(sampling_plans_[k].width(), sampling_plans_[k].height());
      Kernels::sample(sampling_plans_[k], fields[k], reference_image->planes[k], output.back().view());
    }
    return output;
  }
};
} // namespace neo_mv
