#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace neo_mv {

struct SuperGeometryParams {
  std::int32_t width, height;
  std::int32_t block_width, block_height;
  std::int32_t overlap_x, overlap_y;
  std::int32_t pad_x, pad_y;
  std::int32_t ratio_x = 1, ratio_y = 1;
  std::int32_t pel = 2;
  bool chroma = false;
  bool one_level = false;
};

struct LevelGeometry {
  std::int32_t width, height;
  std::int32_t padded_width, padded_height;
  std::int32_t phase_count;
};

struct PlaneGeometry {
  std::int32_t actual_width = 0, actual_height = 0;
  std::int32_t pad_x = 0, pad_y = 0;
  std::vector<LevelGeometry> levels;
};

struct SuperGeometry {
  std::int32_t blocks_x, blocks_y;
  std::int32_t plane_count;
  std::array<PlaneGeometry, 3> planes;
};

namespace geometry_detail {
inline std::int32_t dimension(std::int64_t value) {
  if (value <= 0 || value > INT32_MAX)
    throw std::overflow_error("geometry dimension is not representable");
  return static_cast<std::int32_t>(value);
}

inline bool block_pair(std::int32_t x, std::int32_t y) {
  constexpr std::int32_t pairs[][2] = {{4, 4},   {8, 4},   {8, 8},   {16, 2},  {16, 8},   {16, 16},
                                       {32, 16}, {32, 32}, {64, 32}, {64, 64}, {128, 64}, {128, 128}};
  for (const auto& pair : pairs)
    if (pair[0] == x && pair[1] == y)
      return true;
  return false;
}

// F from kernel-geometry.md; callers supply nonnegative d/pad and ratio 1/2.
inline std::int32_t reduced(std::int32_t d, std::int32_t ratio, std::int32_t pad) {
  return static_cast<std::int32_t>(ratio * ((static_cast<std::int64_t>(d) / ratio + (pad >= ratio)) / 2));
}
} // namespace geometry_detail

inline SuperGeometry make_super_geometry(const SuperGeometryParams& p) {
  using namespace geometry_detail;
  if (!block_pair(p.block_width, p.block_height) || p.width < p.block_width || p.height < p.block_height ||
      p.overlap_x < 0 || p.overlap_x > p.block_width / 2 || p.overlap_y < 0 || p.overlap_y > p.block_height / 2 ||
      p.pad_x <= 0 || p.pad_y <= 0 || (p.ratio_x != 1 && p.ratio_x != 2) || (p.ratio_y != 1 && p.ratio_y != 2) ||
      (!p.chroma && (p.ratio_x != 1 || p.ratio_y != 1)) || (p.pel != 1 && p.pel != 2 && p.pel != 4))
    throw std::invalid_argument("invalid Super geometry parameters");
  if (p.width % p.ratio_x || p.height % p.ratio_y || p.block_width % p.ratio_x || p.block_height % p.ratio_y ||
      p.overlap_x % p.ratio_x || p.overlap_y % p.ratio_y)
    throw std::invalid_argument("Super geometry is not chroma aligned");

  const auto sx = p.block_width - p.overlap_x;
  const auto sy = p.block_height - p.overlap_y;
  const auto nx = dimension((static_cast<std::int64_t>(p.width) - p.overlap_x + sx - 1) / sx);
  const auto ny = dimension((static_cast<std::int64_t>(p.height) - p.overlap_y + sy - 1) / sy);
  const auto working_width = dimension(static_cast<std::int64_t>(nx) * sx + p.overlap_x);
  const auto working_height = dimension(static_cast<std::int64_t>(ny) * sy + p.overlap_y);
  int levels = 1;
  if (!p.one_level) {
    auto w = p.width, h = p.height;
    int reductions = 0;
    for (;;) {
      w = reduced(w, p.ratio_x, p.pad_x);
      h = reduced(h, p.ratio_y, p.pad_y);
      if (w < p.block_width || h < p.block_height)
        break;
      ++reductions;
    }
    levels = std::max(1, reductions); // Intentionally excludes the original level.
  }
  SuperGeometry result{nx, ny, p.chroma ? 3 : 1, {}};
  for (int plane = 0; plane < result.plane_count; ++plane) {
    const auto rx = plane == 0 ? 1 : p.ratio_x;
    const auto ry = plane == 0 ? 1 : p.ratio_y;
    auto& out = result.planes[plane];
    out.actual_width = p.width / rx;
    out.actual_height = p.height / ry;
    out.pad_x = p.pad_x / rx;
    out.pad_y = p.pad_y / ry;
    auto w = working_width / rx, h = working_height / ry;
    out.levels.reserve(levels);
    for (int level = 0; level < levels; ++level) {
      out.levels.push_back({dimension(w), dimension(h), dimension(static_cast<std::int64_t>(w) + 2LL * out.pad_x),
                            dimension(static_cast<std::int64_t>(h) + 2LL * out.pad_y), level == 0 ? p.pel * p.pel : 1});
      if (level + 1 == levels)
        break;
      // The recurrence uses GLOBAL chroma ratios even for a chroma plane.
      const auto next_w = reduced(w, p.ratio_x, out.pad_x);
      const auto next_h = reduced(h, p.ratio_y, out.pad_y);
      if (2LL * next_w > static_cast<std::int64_t>(w) + out.pad_x ||
          2LL * next_h > static_cast<std::int64_t>(h) + out.pad_y)
        throw std::invalid_argument("reduction exceeds readable Super extent");
      w = next_w;
      h = next_h;
    }
  }
  return result;
}

} // namespace neo_mv
