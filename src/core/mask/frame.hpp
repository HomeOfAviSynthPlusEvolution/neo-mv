#pragma once

#include "core/mask/input.hpp"
#include "kernels/mask_scalar.hpp"

#include <variant>

namespace neo_mv {
enum class MaskKind { VectorLength, SAD, Occlusion };

namespace mask_frame_detail {
template <class T>
bool overlaps(span2d::Plane<T> output, const void* data, std::size_t bytes) {
  if (!bytes)
    return false;
  const auto begin = reinterpret_cast<std::uintptr_t>(data);
  if (bytes > std::numeric_limits<std::uintptr_t>::max() - begin)
    throw std::invalid_argument("mask input address span is unrepresentable");
  const auto end = begin + bytes;
  for (int y = 0; y < output.height(); ++y) {
    const auto row = reinterpret_cast<std::uintptr_t>(output.row(y).data());
    if (row < end && begin < row + static_cast<std::size_t>(output.width()) * sizeof(T))
      return true;
  }
  return false;
}
} // namespace mask_frame_detail

// Pixel-only composition. The host creates an independent GRAY output and
// installs exactly _Range=[1]; no carrier pixels or property map enter this API.
template <class T, class Kernels = ScalarMaskKernels<T>>
class MaskFramePlan {
  using VectorLength = typename Kernels::VectorLength;
  using SAD = typename Kernels::SAD;
  using Occlusion = typename Kernels::Occlusion;
  using ScorePlan = std::variant<VectorLength, SAD, Occlusion>;
  MaskInputPlan<T> input_;
  ScorePlan scores_;
  typename Kernels::GridResampling resampling_;

  static ScorePlan make_scores(MaskKind kind, const MaskInputPlan<T>& input) {
    const auto& m = input.metadata();
    switch (kind) {
      case MaskKind::VectorLength:
        return ScorePlan(std::in_place_type<VectorLength>, m, input.f(), input.gamma(), input.time256());
      case MaskKind::SAD:
        return ScorePlan(std::in_place_type<SAD>, m, input.f(), input.gamma(), input.time256());
      case MaskKind::Occlusion:
        return ScorePlan(std::in_place_type<Occlusion>, m, input.f(), input.gamma(), input.time256());
    }
    throw std::invalid_argument("invalid mask kind");
  }
  static GridResamplingGeometry geometry(const AnalysisMetadata& m) {
    return {m.blocks_x,  m.blocks_y,  m.block_width, m.block_height,
            m.overlap_x, m.overlap_y, m.real_width,  m.real_height};
  }

public:
  MaskFramePlan(MaskKind kind, AnalysisMetadata metadata, std::int64_t frames, MaskParameters parameters = {})
      : input_(metadata, frames, parameters), scores_(make_scores(kind, input_)),
        resampling_(geometry(input_.metadata())) {}

  const AnalysisMetadata& metadata() const { return input_.metadata(); }

  void render(const AnalysisField& field, span2d::Plane<T> output) const {
    validate_plane(output);
    const auto& m = metadata();
    if (output.width() != m.real_width || output.height() != m.real_height)
      throw std::invalid_argument("mask output dimensions do not match creation metadata");
    if (mask_frame_detail::overlaps(output, &field, sizeof(field)) ||
        mask_frame_detail::overlaps(output, field.grid.values.data(), field.grid.values.size() * sizeof(MotionTriple)))
      throw std::invalid_argument("mask output aliases analysis input");
    if (!input_.eligible(field, Kernels::scene_count)) {
      for (int y = 0; y < output.height(); ++y)
        std::fill_n(output.row(y).data(), output.width(), input_.fallback());
      return;
    }
    const auto grid = std::visit([&](const auto& plan) { return plan.generate(field.grid); }, scores_);
    if (static_cast<std::uintmax_t>(m.blocks_x) >
        static_cast<std::uintmax_t>(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(T))
      throw std::overflow_error("mask grid row stride is unrepresentable");
    const auto view = checked_plane(grid.data(), m.blocks_x, m.blocks_y,
                                    static_cast<std::ptrdiff_t>(m.blocks_x) * sizeof(T), grid.size() * sizeof(T));
    resampling_.resize(view, output, m.bits);
  }
};
} // namespace neo_mv
