#pragma once

#include "core/render/block_sampling.hpp"
#include "core/render/reference.hpp"

namespace neo_mv {

struct CompensationSelection {
  bool reference;
  RenderDisplacement displacement;
};

class CompensationRule {
  int time_, pel_;
  std::int32_t delta_;
  std::int64_t threshold_;
  bool fields_;
  std::optional<bool> tff_;

  void validate_shift(int shift) const {
    if (shift != 0 && (!fields_ || delta_ % 2 == 0 || (shift != pel_ / 2 && shift != -pel_ / 2)))
      throw std::invalid_argument("invalid compensation field shift");
  }

public:
  CompensationRule(const AnalysisMetadata& m, std::int64_t thsad = 10000, double time = 100, bool fields = false,
                   std::optional<bool> tff = {})
      : pel_(m.pel), delta_(m.delta), threshold_(RenderThresholdScale(m)(thsad)), fields_(fields), tff_(tff) {
    if (!std::isfinite(time) || time < 0 || time > 100 || (fields && pel_ == 1))
      throw std::invalid_argument("invalid compensation time or field mode");
    const double scaled = time * 256.0;
    time_ = static_cast<int>(scaled / 100.0);
  }
  int time_coefficient() const { return time_; }
  int pel() const { return pel_; }
  std::int64_t threshold() const { return threshold_; }

  int field_shift(std::int64_t n, std::optional<bool> current_top = {}, std::optional<bool> reference_top = {}) const {
    if (n < 0)
      throw std::invalid_argument("negative compensation frame index");
    if (!fields_ || delta_ % 2 == 0)
      return 0;
    if (tff_) {
      current_top = *tff_ != (n % 2 != 0);
      reference_top = !*current_top; // odd d always reverses parity, without forming n+d
    }
    if (!current_top || !reference_top)
      throw std::invalid_argument("compensation requires current and reference field parity");
    if (*current_top == *reference_top)
      return 0;
    return *current_top ? pel_ / 2 : -pel_ / 2;
  }

  RenderDisplacement reference_displacement(MotionVector v, int shift) const {
    validate_shift(shift);
    return {std::int64_t(v.x) * time_ / 256, std::int64_t(v.y) * time_ / 256 + shift};
  }
  CompensationSelection select(MotionVector v, std::int64_t sad, int shift) const {
    if (sad < 0)
      throw std::invalid_argument("negative compensation SAD");
    const auto d = reference_displacement(v, shift);
    return sad < threshold_ ? CompensationSelection{true, d} : CompensationSelection{false, {0, shift}};
  }
  void admit(const RenderPhaseGeometry& g, BlockRegion b, CandidateDomain domain) const {
    if (g.pel != pel_)
      throw std::invalid_argument("compensation pel differs from render phases");
    validate_render_domain(g, b, domain, time_);
    render_footprint(g, b, {0, 0});
    if (fields_ && delta_ % 2 != 0) {
      render_footprint(g, b, {0, -pel_ / 2});
      render_footprint(g, b, {0, pel_ / 2});
    }
  }
};

// The frame composer must preflight ALL vectors/planes before generating any
// blocks, even those whose SAD selects the current image. The direct block
// entry also checks its reference footprint before selecting an image.
inline void validate_compensation_footprint(const CompensationRule& rule, const RenderPhaseGeometry& g, BlockRegion b,
                                            MotionVector v, int shift) {
  if (g.pel != rule.pel())
    throw std::invalid_argument("compensation pel differs from render phases");
  render_footprint(g, b, rule.reference_displacement(v, shift));
}

template <class T>
void sample_compensated_block(const CompensationRule& rule, const RenderPhaseGeometry& g, BlockRegion b,
                              MotionTriple vector, int shift, const SubpixelPhases<T>& current,
                              const SubpixelPhases<T>& reference, span2d::Plane<T> output, int bits) {
  validate_compensation_footprint(rule, g, b, vector.vector, shift);
  const auto selected = rule.select(vector.vector, vector.error, shift);
  validate_plane(output);
  // Both images are required inputs even when this block selects only one.
  // Reject aliasing against the unselected image as well as the selected one.
  for (const auto* image : {&current, &reference}) {
    if (image->pel != g.pel)
      throw std::invalid_argument("compensation image phase count mismatch");
    for (int a = 0; a < g.pel * g.pel; ++a) {
      const auto plane = image->planes[a];
      validate_plane(plane);
      if (plane.width() != g.phases[a].width || plane.height() != g.phases[a].height ||
          active_rows_overlap(plane, output))
        throw std::invalid_argument("compensation image geometry mismatch or output alias");
    }
  }
  sample_render_block(g, b, selected.displacement, selected.reference ? reference : current, output, bits);
}

} // namespace neo_mv
