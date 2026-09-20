#pragma once

#include "kernels/render_scalar.hpp"
#include <cstring>
#include <utility>

namespace neo_mv {

// Borrowed level-zero views. The payload owner and plan must outlive each call.
template <class T>
struct RenderImage {
  const SuperPlan<T>* plan = nullptr;
  std::array<SubpixelPhases<T>, 3> planes{};
};
template <class T>
using RenderPixels = std::array<span2d::Plane<const T>, 3>;
template <class T>
using RenderOutput = std::vector<super_detail::PlaneBuffer<T>>;

// Shared immutable grid. Only consumed level-zero geometry participates in
// compatibility; pyramid levels and Super's block hints are not render inputs.
template <class T>
class RenderFramePlan {
  RenderVideo clip_;
  AnalysisMetadata metadata_;
  SamplingGeometry sampling_;
  RenderPlanes processed_;
  std::array<std::optional<OverlapCompositionPlan>, 3> composition_;

public:
  RenderFramePlan(RenderVideo clip, const SuperPlan<T>& super, std::int64_t super_frames,
                  const AnalysisMetadata& metadata, std::int64_t vector_frames, RenderPlanes processed)
      : clip_(clip), metadata_(metadata), sampling_(super_sampling_geometry(super)[0]), processed_(processed) {
    validate_render_geometry(clip, super, super_frames, metadata, vector_frames, processed);
    for (int k = 0; k < plane_count(); ++k) {
      if (!processed_[k])
        continue;
      const auto g = phase_geometry(k);
      composition_[k].emplace(BlockCompositionGeometry{
          metadata.block_width / g.ratio_x, metadata.block_height / g.ratio_y, metadata.overlap_x / g.ratio_x,
          metadata.overlap_y / g.ratio_y, metadata.blocks_x, metadata.blocks_y, clip.width / g.ratio_x,
          clip.height / g.ratio_y, metadata.width / g.ratio_x, metadata.height / g.ratio_y});
    }
  }
  int plane_count() const { return clip_.chroma ? 3 : 1; }
  int bits() const { return clip_.bits; }
  bool processed(int k) const { return processed_.at(k); }
  const AnalysisMetadata& metadata() const { return metadata_; }
  RenderPhaseGeometry phase_geometry(int k) const { return render_phase_geometry(sampling_, k); }
  const OverlapCompositionPlan& composition(int k) const { return composition_.at(k).value(); }
  BlockRegion block(int x, int y) const {
    return {x * (metadata_.block_width - metadata_.overlap_x), y * (metadata_.block_height - metadata_.overlap_y),
            metadata_.block_width, metadata_.block_height};
  }
  template <class F>
  void each_block(F&& f) const {
    for (int y = 0; y < metadata_.blocks_y; ++y)
      for (int x = 0; x < metadata_.blocks_x; ++x)
        f(block(x, y), field_detail::bounds(metadata_, x, y), std::size_t(y) * metadata_.blocks_x + x);
  }
  void validate_image(const RenderImage<T>& image) const {
    if (!image.plan)
      throw std::invalid_argument("missing render Super plan");
    const auto& p = image.plan->params();
    if (p.width != clip_.width || p.height != clip_.height || image.plan->bits() != clip_.bits ||
        p.chroma != clip_.chroma || p.ratio_x != clip_.ratio_x || p.ratio_y != clip_.ratio_y)
      throw std::invalid_argument("render Super format changed");
    const auto actual = super_sampling_geometry(*image.plan)[0];
    if (actual.pel != sampling_.pel)
      throw std::invalid_argument("render Super pel changed");
    for (int k = 0; k < plane_count(); ++k) {
      const auto& expected = sampling_.planes[k];
      const auto& other = actual.planes[k];
      if (expected.pad_x != other.pad_x || expected.pad_y != other.pad_y ||
          expected.current.width != other.current.width || expected.current.height != other.current.height ||
          image.planes[k].pel != sampling_.pel)
        throw std::invalid_argument("render Super level-zero geometry changed");
      for (int a = 0; a < sampling_.pel * sampling_.pel; ++a) {
        const auto extent = expected.reference[a];
        const auto view = image.planes[k].planes[a];
        validate_plane(view);
        if (extent.width != other.reference[a].width || extent.height != other.reference[a].height ||
            extent.width != view.width() || extent.height != view.height())
          throw std::invalid_argument("render Super phase domain or storage changed");
      }
    }
  }
  RenderOutput<T> copy_clip(const RenderPixels<T>& pixels) const {
    RenderOutput<T> output;
    for (int k = 0; k < plane_count(); ++k) {
      const int rx = k ? clip_.ratio_x : 1, ry = k ? clip_.ratio_y : 1;
      validate_plane(pixels[k]);
      if (pixels[k].width() != clip_.width / rx || pixels[k].height() != clip_.height / ry)
        throw std::invalid_argument("render clip storage geometry changed");
      output.emplace_back(pixels[k].width(), pixels[k].height());
      auto dst = output.back().view();
      for (int y = 0; y < dst.height(); ++y)
        std::memcpy(dst.row(y).data(), pixels[k].row(y).data(), std::size_t(dst.width()) * sizeof(T));
    }
    return output;
  }
  template <class Kernels = ScalarRenderKernels<T>, class Generate>
  super_detail::PlaneBuffer<T> compose(int k, Generate&& generate) const {
    const auto& plan = composition(k);
    const auto& g = plan.geometry();
    std::vector<super_detail::PlaneBuffer<T>> blocks;
    each_block([&](BlockRegion b, CandidateDomain, std::size_t i) {
      blocks.emplace_back(g.block_width, g.block_height);
      generate(b, i, blocks.back().view());
    });
    std::vector<span2d::Plane<const T>> views;
    for (const auto& b : blocks)
      views.push_back(b.view());
    super_detail::PlaneBuffer<T> result(g.visible_width, g.visible_height);
    Kernels::compose_render_blocks(plan, views, result.view(), bits());
    return result;
  }
};

struct CompensateParameters {
  std::int64_t thsad = 10000, thscd1 = 400;
  double time = 100, thscd2 = 51;
  bool fields = false;
  std::optional<bool> tff;
};
template <class T, class Kernels = ScalarRenderKernels<T>>
class CompensateFramePlan {
  RenderFramePlan<T> grid_;
  ReferenceAvailability availability_;
  CompensationRule rule_;

public:
  CompensateFramePlan(RenderVideo clip, const SuperPlan<T>& super, std::int64_t super_frames, const AnalysisMetadata& m,
                      std::int64_t vector_frames, CompensateParameters p = {})
      : grid_(clip, super, super_frames, m, vector_frames, {true, true, true}),
        availability_(m, clip.frames, p.thscd1, p.thscd2), rule_(m, p.thsad, p.time, p.fields, p.tff) {
    grid_.each_block([&](BlockRegion b, CandidateDomain domain, std::size_t) {
      for (int k = 0; k < grid_.plane_count(); ++k)
        rule_.admit(grid_.phase_geometry(k), b, domain);
    });
  }
  void validate_current(const RenderImage<T>& current) const { grid_.validate_image(current); }
  std::optional<std::int64_t> reference(const AnalysisField& field, std::int64_t n) const {
    return availability_(field, n);
  }
  RenderOutput<T> render(std::int64_t n, const AnalysisField& field, const RenderPixels<T>& clip,
                         const RenderImage<T>& current, const RenderImage<T>* reference_image,
                         std::optional<bool> current_top = {}, std::optional<bool> reference_top = {}) const {
    validate_current(current);
    const auto selected = reference(field, n);
    auto output = grid_.copy_clip(clip);
    if (!selected)
      return output;
    if (!reference_image)
      throw std::invalid_argument("missing required compensation reference");
    grid_.validate_image(*reference_image);
    const int shift = rule_.field_shift(n, current_top, reference_top);
    // All actual reference footprints are checked before generating ANY block,
    // including cropped rectangles and blocks selecting current pixels by SAD.
    grid_.each_block([&](BlockRegion b, CandidateDomain, std::size_t i) {
      for (int k = 0; k < grid_.plane_count(); ++k)
        validate_compensation_footprint(rule_, grid_.phase_geometry(k), b, field.grid.values[i].vector, shift);
    });
    for (int k = 0; k < grid_.plane_count(); ++k)
      output[k] = grid_.template compose<Kernels>(k, [&](BlockRegion b, std::size_t i, span2d::Plane<T> dst) {
        Kernels::sample_compensated_block(rule_, grid_.phase_geometry(k), b, field.grid.values[i], shift,
                                          current.planes[k], reference_image->planes[k], dst, grid_.bits());
      });
    return output;
  }
};

struct DegrainParameters {
  std::array<std::int64_t, 2> near{400, 400}, far{400, 400};
  std::array<double, 2> limit{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  RenderPlanes planes{true, true, true};
  std::vector<std::int64_t> weights; // Empty means omitted at this core boundary.
  std::int64_t thscd1 = 400;
  double thscd2 = 51;
};
template <class T, class Kernels = ScalarRenderKernels<T>>
class DegrainFramePlan {
  RenderFramePlan<T> grid_;
  std::vector<ReferenceAvailability> availability_;
  DegrainWeightPlan weights_;
  std::array<ChangeLimit<T>, 2> limits_;

  static const AnalysisMetadata& first(const std::vector<AnalysisMetadata>& members) {
    validate_degrain_pairs(members);
    return members.front();
  }

public:
  DegrainFramePlan(RenderVideo clip, const SuperPlan<T>& super, std::int64_t super_frames,
                   const std::vector<AnalysisMetadata>& members, const std::vector<std::int64_t>& vector_frames,
                   DegrainParameters p = {})
      : grid_(clip, super, super_frames, first(members), clip.frames, p.planes),
        weights_(members.front(), static_cast<int>(members.size() / 2), p.near, p.far,
                 p.weights.empty() ? std::vector<std::int64_t>(members.size() + 1, 1) : p.weights),
        limits_{ChangeLimit<T>(p.limit[0], clip.bits), ChangeLimit<T>(p.limit[1], clip.bits)} {
    if (vector_frames.size() != members.size())
      throw std::invalid_argument("Degrain member frame count mismatch");
    for (std::size_t i = 0; i < members.size(); ++i) {
      validate_render_geometry(clip, super, super_frames, members[i], vector_frames[i], p.planes);
      availability_.emplace_back(members[i], clip.frames, p.thscd1, p.thscd2);
    }
    grid_.each_block([&](BlockRegion b, CandidateDomain domain, std::size_t) {
      for (int k = 0; k < grid_.plane_count(); ++k)
        if (grid_.processed(k)) {
          const auto g = grid_.phase_geometry(k);
          validate_render_domain(g, b, domain);
          render_footprint(g, b, {0, 0});
        }
    });
  }
  void validate_current(const RenderImage<T>& current) const { grid_.validate_image(current); }
  std::vector<std::optional<std::int64_t>> references(const std::vector<AnalysisField>& fields, std::int64_t n) const {
    if (fields.size() != availability_.size())
      throw std::invalid_argument("Degrain field count mismatch");
    std::vector<std::optional<std::int64_t>> result;
    for (std::size_t i = 0; i < fields.size(); ++i)
      result.push_back(availability_[i](fields[i], n));
    return result;
  }
  RenderOutput<T> render(std::int64_t n, const std::vector<AnalysisField>& fields, const RenderPixels<T>& clip,
                         const RenderImage<T>& current, const std::vector<RenderImage<T>>& images) const {
    validate_current(current);
    const auto selected = references(fields, n);
    if (images.size() != selected.size())
      throw std::invalid_argument("Degrain reference image count mismatch");
    for (std::size_t i = 0; i < selected.size(); ++i)
      if (selected[i])
        grid_.validate_image(images[i]); // Required even for zero user coefficients.
    auto output = grid_.copy_clip(clip);
    for (int k = 0; k < grid_.plane_count(); ++k) {
      if (!grid_.processed(k))
        continue;
      const auto g = grid_.phase_geometry(k);
      auto composed = grid_.template compose<Kernels>(k, [&](BlockRegion b, std::size_t index, span2d::Plane<T> dst) {
        super_detail::PlaneBuffer<T> centre(dst.width(), dst.height());
        Kernels::sample_render_block(g, b, {0, 0}, current.planes[k], centre.view(), grid_.bits());
        std::vector<super_detail::PlaneBuffer<T>> sampled;
        std::vector<WeightedReferenceBlock<T>> blocks(selected.size());
        std::vector<ReferenceReliability> reliability;
        sampled.reserve(selected.size());
        for (std::size_t i = 0; i < selected.size(); ++i) {
          reliability.push_back({bool(selected[i]), selected[i] ? fields[i].grid.values[index].error : 0});
          if (!selected[i])
            continue;
          sampled.emplace_back(dst.width(), dst.height());
          const auto v = fields[i].grid.values[index].vector;
          Kernels::sample_render_block(g, b, {v.x, v.y}, images[i].planes[k], sampled.back().view(), grid_.bits());
          blocks[i] = {true, std::as_const(sampled.back()).view()};
        }
        Kernels::weighted_render_block(std::as_const(centre).view(), blocks, weights_(reliability, k), dst,
                                       grid_.bits());
      });
      const auto centre =
          current.planes[k].planes[0].subplane(g.pad_x, g.pad_y, output[k].view().width(), output[k].view().height());
      Kernels::limit_render_plane(limits_[k ? 1 : 0], std::as_const(composed).view(), centre, output[k].view());
    }
    return output;
  }
};

} // namespace neo_mv
