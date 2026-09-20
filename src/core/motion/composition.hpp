#pragma once

#include "core/motion/analysis_field.hpp"
#include "core/motion/search.hpp"
#include "kernels/scalar.hpp"

namespace neo_mv {

// Shared Analyse/Recalculate scaling, with the explicitly ordered binary64
// conversion and signed truncation (negative thresholds are not rounded away).
inline std::int64_t scale_precision(std::int64_t value, int bits) {
  if (!((bits >= 8 && bits <= 16) || bits == 32))
    throw std::invalid_argument("invalid analysis precision");
  const double maximum = double((1 << std::min(16, bits)) - 1);
  return prediction_detail::truncate(((double(value) * maximum) / 255.0) + 0.5);
}
inline std::int64_t scale_area(std::int64_t value, int width, int height) {
  if (width <= 0 || height <= 0)
    throw std::invalid_argument("invalid analysis block area");
  return prediction_detail::weight(value, std::int64_t(width) * height) / 64;
}

template <class T>
void validate_analysis_precision(int bits) {
  static_assert(supported_sample<T>, "unsupported analysis storage");
  if ((std::is_same_v<T, float> && bits != 32) || (std::is_same_v<T, std::uint8_t> && bits != 8) ||
      (std::is_same_v<T, std::uint16_t> && (bits < 9 || bits > 16)))
    throw std::invalid_argument("analysis precision does not match sample storage");
}

inline void validate_owned_field(const AnalysisField& field) {
  if (field.state == FieldState::invalid_metadata || !valid_analysis_metadata(field.metadata))
    throw std::invalid_argument("invalid input analysis metadata");
  if (field.state == FieldState::metadata_only)
    return;
  const auto& m = field.metadata;
  if (field.state != FieldState::complete || field.grid.width != m.blocks_x || field.grid.height != m.blocks_y ||
      field.grid.values.size() != field_detail::count(m))
    throw std::invalid_argument("inconsistent input analysis grid");
  for (std::size_t i = 0; i < field.grid.values.size(); ++i)
    field_detail::vector(m, field.grid.values[i], static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
}

inline BlockRegion analysis_block(const AnalysisMetadata& m, int bx, int by) {
  if (bx < 0 || bx >= m.blocks_x || by < 0 || by >= m.blocks_y)
    throw std::invalid_argument("analysis block index outside grid");
  return {prediction_detail::coordinate(std::int64_t(bx) * (m.block_width - m.overlap_x)),
          prediction_detail::coordinate(std::int64_t(by) * (m.block_height - m.overlap_y)), m.block_width,
          m.block_height};
}

inline CandidateDomain analysis_domain(const AnalysisMetadata& m, BlockRegion block, int bound_pad_x = -1,
                                       int bound_pad_y = -1) {
  const auto hp = bound_pad_x < 0 ? m.pad_x : bound_pad_x;
  const auto vp = bound_pad_y < 0 ? m.pad_y : bound_pad_y;
  const auto p = std::int64_t(m.pel);
  return {-p * (std::int64_t(block.x) + hp), -p * (std::int64_t(block.y) + vp),
          p * (std::int64_t(m.width) + hp - block.x - block.width),
          p * (std::int64_t(m.height) + vp - block.y - block.height)};
}

// Geometry-only admission. Finest grids use ceil; coarser grids are supplied
// by the multilevel planner. No known unsafe candidate may reach evaluation.
inline void validate_motion_layer(const AnalysisMetadata& m, const SamplingGeometry& g, bool finest,
                                  std::initializer_list<MotionVector> seeds = {}, int bound_pad_x = -1,
                                  int bound_pad_y = -1) {
  if (!valid_analysis_metadata(m) || !geometry_detail::block_pair(m.block_width, m.block_height) || g.pel != m.pel ||
      g.ratio_x != m.ratio_x || g.ratio_y != m.ratio_y || g.chroma != m.chroma || g.planes[0].pad_x != m.pad_x ||
      g.planes[0].pad_y != m.pad_y || m.overlap_x % m.ratio_x || m.overlap_y % m.ratio_y)
    throw std::invalid_argument("invalid motion layer geometry");
  const auto sx = m.block_width - m.overlap_x, sy = m.block_height - m.overlap_y;
  const auto cover_x = std::int64_t(m.blocks_x) * sx + m.overlap_x;
  const auto cover_y = std::int64_t(m.blocks_y) * sy + m.overlap_y;
  if (cover_x > m.width || cover_y > m.height ||
      (finest && (m.blocks_x != (std::int64_t(m.real_width) - m.overlap_x + sx - 1) / sx ||
                  m.blocks_y != (std::int64_t(m.real_height) - m.overlap_y + sy - 1) / sy)))
    throw std::invalid_argument("motion grid does not fit working geometry");
  if (field_detail::count(m) > std::vector<MotionTriple>{}.max_size())
    throw std::overflow_error("motion grid allocation size is unrepresentable");
  for (int k = 0; k < (m.chroma ? 3 : 1); ++k) {
    const auto& plane = g.planes[k];
    const int rx = k == 0 ? 1 : m.ratio_x, ry = k == 0 ? 1 : m.ratio_y;
    if (plane.pad_x != m.pad_x / rx || plane.pad_y != m.pad_y / ry ||
        ((finest || k == 0) && (m.width % rx || m.height % ry ||
                                plane.current.width != std::int64_t(m.width) / rx + 2 * std::int64_t(plane.pad_x) ||
                                plane.current.height != std::int64_t(m.height) / ry + 2 * std::int64_t(plane.pad_y))))
      throw std::invalid_argument("inconsistent logical Super extent");
  }
  for (int by = 0; by < m.blocks_y; ++by)
    for (int bx = 0; bx < m.blocks_x; ++bx) {
      const auto block = analysis_block(m, bx, by);
      validate_sampling_domain(g, block, analysis_domain(m, block, bound_pad_x, bound_pad_y), seeds);
    }
}

} // namespace neo_mv
