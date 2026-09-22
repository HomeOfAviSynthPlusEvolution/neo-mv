#pragma once

#include "core/super/pyramid.hpp"

namespace neo_mv {

inline std::vector<float> overlap_axis_window(int length, int overlap, int blocks, int index) {
  if (length <= 0 || overlap < 0 || overlap > length / 2 || blocks <= 0 || index < 0 || index >= blocks)
    throw std::invalid_argument("invalid overlap window geometry");
  std::vector<float> result(super_detail::sample_count<float>(length, 1), 1.0f);
  constexpr float pi = 0x1.921fb6p+1f;
  for (int i = 0; i < length; ++i) {
    const bool left = i < overlap && index != 0;
    const bool right = i >= length - overlap && index != blocks - 1;
    if (!left && !right)
      continue;
    const int z = left ? i - overlap : i - length + overlap;
    const float half = static_cast<float>(double(z) + 0.5);
    const float product = pi * half;
    const float angle = product / static_cast<float>(2 * overlap);
    const float cosine = std::cos(angle);
    result[i] = cosine * cosine;
  }
  return result;
}

struct BlockCompositionGeometry {
  int block_width, block_height, overlap_x, overlap_y;
  int blocks_x, blocks_y, visible_width, visible_height, working_width, working_height;
};

// Immutable per-plane plan. At most nine boundary window combinations are
// stored, regardless of grid size; a single block replaces both shoulders.
class OverlapCompositionPlan {
  BlockCompositionGeometry geometry_;
  std::array<std::vector<std::uint16_t>, 9> windows_;

  static int kind(int index, int count) { return index == 0 ? 0 : index == count - 1 ? 2 : 1; }

public:
  explicit OverlapCompositionPlan(BlockCompositionGeometry g) : geometry_(g) {
    if (g.block_width <= 0 || g.block_height <= 0 || g.overlap_x < 0 || g.overlap_y < 0 ||
        g.overlap_x > g.block_width / 2 || g.overlap_y > g.block_height / 2 || g.blocks_x <= 0 || g.blocks_y <= 0 ||
        g.visible_width <= 0 || g.visible_height <= 0 || g.working_width <= 0 || g.working_height <= 0)
      throw std::invalid_argument("invalid block composition geometry");
    const auto width = std::int64_t(g.blocks_x) * (g.block_width - g.overlap_x) + g.overlap_x;
    const auto height = std::int64_t(g.blocks_y) * (g.block_height - g.overlap_y) + g.overlap_y;
    if (width < g.visible_width || height < g.visible_height || width > g.working_width || height > g.working_height)
      throw std::invalid_argument("block grid does not cover visible image inside working dimensions");
    if (!has_overlap())
      return;
    const auto count = super_detail::sample_count<std::uint16_t>(g.block_width, g.block_height);
    std::array<std::vector<float>, 3> xs, ys;
    for (int kind = 0; kind < 3; ++kind) {
      const int bx = kind == 0 ? 0 : kind == 2 ? g.blocks_x - 1 : std::min(1, g.blocks_x - 1);
      const int by = kind == 0 ? 0 : kind == 2 ? g.blocks_y - 1 : std::min(1, g.blocks_y - 1);
      xs[kind] = overlap_axis_window(g.block_width, g.overlap_x, g.blocks_x, bx);
      ys[kind] = overlap_axis_window(g.block_height, g.overlap_y, g.blocks_y, by);
    }
    for (int ky = 0; ky < 3; ++ky)
      for (int kx = 0; kx < 3; ++kx) {
        auto& window = windows_[ky * 3 + kx];
        window.resize(count);
        for (int y = 0; y < g.block_height; ++y)
          for (int x = 0; x < g.block_width; ++x) {
            const float product = ys[ky][y] * xs[kx][x];
            const float scaled = product * 2048.0f;
            const float biased = scaled + 0.5f;
            window[std::size_t(y) * g.block_width + x] = static_cast<std::uint16_t>(biased);
          }
      }
  }
  const BlockCompositionGeometry& geometry() const { return geometry_; }
  bool has_overlap() const { return geometry_.overlap_x != 0 || geometry_.overlap_y != 0; }
  // The immutable row stays valid while this plan remains alive and unmodified.
  const std::uint16_t* coefficient_row(int bx, int by, int y) const {
    const auto& g = geometry_;
    if (!has_overlap() || bx < 0 || bx >= g.blocks_x || by < 0 || by >= g.blocks_y || y < 0 || y >= g.block_height)
      throw std::invalid_argument("invalid overlap coefficient row");
    return windows_[kind(by, g.blocks_y) * 3 + kind(bx, g.blocks_x)].data() + std::size_t(y) * g.block_width;
  }
  std::uint16_t coefficient(int bx, int by, int x, int y) const {
    const auto& g = geometry_;
    if (bx < 0 || bx >= g.blocks_x || by < 0 || by >= g.blocks_y || x < 0 || x >= g.block_width || y < 0 ||
        y >= g.block_height)
      throw std::invalid_argument("overlap coefficient index outside block grid");
    if (!has_overlap())
      return 2048;
    return windows_[kind(by, g.blocks_y) * 3 + kind(bx, g.blocks_x)][std::size_t(y) * g.block_width + x];
  }
};

template <class T, bool Validated = false>
void compose_render_blocks(const OverlapCompositionPlan& plan, const std::vector<span2d::Plane<const T>>& blocks,
                           span2d::Plane<T> output, int bits) {
  const auto& g = plan.geometry();
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  if constexpr (!Validated) {
    validate_plane(output);
    if (blocks.size() != std::uint64_t(g.blocks_x) * g.blocks_y || output.width() != g.visible_width ||
        output.height() != g.visible_height)
      throw std::invalid_argument("block composition storage geometry mismatch");
    for (auto block : blocks) {
      validate_plane(block);
      if (block.width() != g.block_width || block.height() != g.block_height || active_rows_overlap(block, output))
        throw std::invalid_argument("invalid block storage or output aliases input");
      // Cropping never excuses an invalid generated block rectangle.
      for (int y = 0; y < block.height(); ++y)
        for (int x = 0; x < block.width(); ++x)
          subpixel_detail::valid_sample(block.row(y)[x], maximum);
    }
  }
  const int sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
  for (int y = 0; y < g.visible_height; ++y)
    for (int x = 0; x < g.visible_width; ++x) {
      if (!plan.has_overlap()) {
        output.row(y)[x] = blocks[std::size_t(y / sy) * g.blocks_x + x / sx].row(y % sy)[x % sx];
        continue;
      }
      // At most two blocks on each axis cover a sample. Iterate by block row
      // then column, preserving the scalar float accumulation order.
      const int first_x = x < g.block_width ? 0 : (x - g.block_width) / sx + 1;
      const int first_y = y < g.block_height ? 0 : (y - g.block_height) / sy + 1;
      const int last_x = std::min(x / sx, g.blocks_x - 1), last_y = std::min(y / sy, g.blocks_y - 1);
      subpixel_detail::Acc<T> sum = 0;
      for (int by = first_y; by <= last_y; ++by)
        for (int bx = first_x; bx <= last_x; ++bx) {
          const int lx = x - bx * sx, ly = y - by * sy;
          const auto q = blocks[std::size_t(by) * g.blocks_x + bx].row(ly)[lx];
          const auto w = plan.coefficient(bx, by, lx, ly);
          if constexpr (std::is_same_v<T, float>) {
            const float product = q * static_cast<float>(w);
            const float contribution = product / 64.0f;
            sum = sum + contribution;
            if (!std::isfinite(product) || !std::isfinite(sum))
              throw std::overflow_error("non-finite overlap intermediate");
          } else {
            sum += (std::int64_t(q) * w) / 64;
          }
        }
      if constexpr (std::is_same_v<T, float>)
        output.row(y)[x] = sum / 32.0f;
      else
        output.row(y)[x] = static_cast<T>(std::clamp((sum + 16) / 32, std::int64_t{0}, maximum));
    }
}

} // namespace neo_mv
