#pragma once

#include "core/interpolation/input.hpp"
#include "kernels/interpolation_scalar.hpp"

namespace neo_mv {
inline int interpolation_time_coefficient(double parameter) {
  const float value = mask_detail::binary32(parameter);
  if (value < 0 || value > 100)
    throw std::invalid_argument("interpolation time must be in [0,100]");
  const float scaled = value * 256.0f;
  return static_cast<int>(scaled / 100.0f);
}

template <class T, class Kernels>
class TemporalFrameBase {
protected:
  InterpolationInputPlan<T> input_;
  RenderVideo video_;

public:
  TemporalFrameBase(InterpolationInputPlan<T> input, RenderVideo video) : input_(std::move(input)), video_(video) {}
  const InterpolationInputPlan<T>& input() const { return input_; }
  bool main_eligible(const AnalysisField& backward, const AnalysisField& forward) const {
    return input_.main_eligible(backward, forward, Kernels::scene_count);
  }
  bool extra_eligible(const AnalysisField& backward, const AnalysisField& forward) const {
    return input_.extra_eligible(backward, forward, Kernels::scene_count);
  }
  void validate_clip(const RenderPixels<T>& clip) const {
    for (int k = 0; k < input_.plane_count(); ++k) {
      validate_plane(clip[k]);
      const int rx = k ? video_.ratio_x : 1, ry = k ? video_.ratio_y : 1;
      if (clip[k].width() != video_.width / rx || clip[k].height() != video_.height / ry)
        throw std::invalid_argument("temporal clip storage dimensions changed");
    }
  }
  RenderOutput<T> allocate() const {
    RenderOutput<T> result;
    for (int k = 0; k < input_.plane_count(); ++k)
      result.emplace_back(video_.width / (k ? video_.ratio_x : 1), video_.height / (k ? video_.ratio_y : 1));
    return result;
  }
  RenderOutput<T> copy(const RenderPixels<T>& clip) const {
    validate_clip(clip);
    auto result = allocate();
    for (int k = 0; k < input_.plane_count(); ++k) {
      auto out = result[k].view();
      for (int y = 0; y < out.height(); ++y)
        std::memcpy(out.row(y).data(), clip[k].row(y).data(), std::size_t(out.width()) * sizeof(T));
    }
    return result;
  }
};

template <class T, class Kernels = ScalarInterpolationKernels<T>>
class InterpolationFramePlan : public TemporalFrameBase<T, Kernels> {
  using Base = TemporalFrameBase<T, Kernels>;
  std::vector<typename Kernels::Dense> dense_;

public:
  InterpolationFramePlan(InterpolationInputPlan<T> input, RenderVideo video, double ml = 100)
      : Base(std::move(input), video) {
    for (int k = 0; k < this->input_.plane_count(); ++k)
      dense_.emplace_back(this->input_.metadata(0), this->input_.metadata(1), k ? video.ratio_x : 1,
                          k ? video.ratio_y : 1, ml);
  }
  RenderOutput<T> blend(const RenderPixels<T>& a, const RenderPixels<T>& b, int time) const {
    this->validate_clip(a);
    this->validate_clip(b);
    auto result = this->allocate();
    for (int k = 0; k < this->input_.plane_count(); ++k)
      Kernels::blend(a[k], b[k], result[k].view(), time, this->video_.bits);
    return result;
  }
  // Eligible is internal: the same immutable fields passed main/extra eligibility.
  template <bool Eligible = false>
  RenderOutput<T> motion(const AnalysisField& B, const AnalysisField& F, const AnalysisField* BB,
                         const AnalysisField* FF, const RenderImage<T>& left, const RenderImage<T>& right,
                         int time) const {
    if constexpr (!Eligible)
      if (!this->main_eligible(B, F) || bool(BB) != bool(FF) || (BB && !this->extra_eligible(*BB, *FF)))
        throw std::invalid_argument("motion interpolation requires eligible fields");
    this->input_.validate_image(left);
    this->input_.validate_image(right);
    using DenseResult = decltype(dense_[0].generate(B.grid, F.grid, time));
    std::vector<DenseResult> fields;
    std::vector<typename Kernels::Sampling> plans;
    for (int k = 0; k < this->input_.plane_count(); ++k) {
      fields.push_back(
          dense_[k].template generate<true>(B.grid, F.grid, time, BB ? &BB->grid : nullptr, FF ? &FF->grid : nullptr));
      const auto g = this->input_.phase_geometry(k);
      plans.emplace_back(g, g, this->video_.width / g.ratio_x, this->video_.height / g.ratio_y, time,
                         this->video_.bits);
      const auto& f = fields.back();
      plans.back().preflight(f.B, f.F, f.BB ? &*f.BB : nullptr, f.FF ? &*f.FF : nullptr);
    }
    // Every used plane has passed preflight before any reference pixel is read.
    auto result = this->allocate();
    for (int k = 0; k < this->input_.plane_count(); ++k) {
      const auto& f = fields[k];
      plans[k].render_preflighted(left.planes[k], right.planes[k], f.B, f.F, f.BB ? &*f.BB : nullptr,
                                  f.FF ? &*f.FF : nullptr, f.mF, f.mB, result[k].view());
    }
    return result;
  }
};

template <class T, class Kernels = ScalarInterpolationKernels<T>>
class BlurFramePlan : public TemporalFrameBase<T, Kernels> {
  using Base = TemporalFrameBase<T, Kernels>;
  std::vector<typename Kernels::DenseFlow> dense_;
  std::vector<typename Kernels::BlurSampling> plans_;

public:
  BlurFramePlan(InterpolationInputPlan<T> input, RenderVideo video, double blur, std::int64_t precision)
      : Base(std::move(input), video) {
    const auto prec = static_cast<int>(std::clamp(precision, std::int64_t(INT32_MIN), std::int64_t(INT32_MAX)));
    const int time = blur_time_coefficient(mask_detail::binary32(blur));
    for (int k = 0; k < this->input_.plane_count(); ++k) {
      const auto g = this->input_.phase_geometry(k);
      dense_.emplace_back(this->input_.metadata(0), g.ratio_x, g.ratio_y);
      plans_.emplace_back(g, video.width / g.ratio_x, video.height / g.ratio_y, prec, time);
    }
  }
  // Eligible is internal: the same immutable fields passed main/extra eligibility.
  template <bool Eligible = false>
  RenderOutput<T> motion(const AnalysisField& B, const AnalysisField& F, const RenderImage<T>& image) const {
    if constexpr (!Eligible)
      if (!this->main_eligible(B, F))
        throw std::invalid_argument("motion blur requires eligible fields");
    this->input_.validate_image(image);
    std::vector<DenseFlowField> backward, forward;
    for (int k = 0; k < this->input_.plane_count(); ++k) {
      backward.push_back(dense_[k].template generate<true>(B.grid, 0));
      forward.push_back(dense_[k].template generate<true>(F.grid, 0));
      plans_[k].preflight(forward.back(), backward.back());
    }
    auto result = this->allocate();
    for (int k = 0; k < this->input_.plane_count(); ++k)
      plans_[k].template sample<T, typename Kernels::BlurAverage, true, true>(forward[k], backward[k], image.planes[k],
                                                                              result[k].view(), this->video_.bits);
    return result;
  }
};
} // namespace neo_mv
