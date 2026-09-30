#pragma once
#include "core/render/fused.hpp"
#include <type_traits>
#include <vector>

namespace neo_mv {
namespace chroma_wiener_detail {
// Tap offsets -2..3, normalized by 128. Phase 4 is the existing six-tap
// Wiener half-pixel kernel [1,-5,20,20,-5,1]/32. Other phases interpolate
// coefficients between integer and half-pixel positions, before rounding.
inline constexpr std::array<std::array<int, 6>, 8> coefficients{{
    {{0, 0, 128, 0, 0, 0}},
    {{1, -5, 116, 20, -5, 1}},
    {{2, -10, 104, 40, -10, 2}},
    {{3, -15, 92, 60, -15, 3}},
    {{4, -20, 80, 80, -20, 4}},
    {{3, -15, 60, 92, -15, 3}},
    {{2, -10, 40, 104, -10, 2}},
    {{1, -5, 20, 116, -5, 1}},
}};
} // namespace chroma_wiener_detail

// Internal admitted render path. Only the integer Super plane is sampled;
// precomputed fractional planes and pelclip samples are not used here.
template <class T>
void sample_chroma_wiener(SampledRenderBlock<T>& output, std::vector<std::vector<T>>& storage,
                          const RenderPhaseGeometry& g, BlockRegion block, RenderDisplacement d,
                          const SubpixelPhases<T>& image, int bits) {
  if (g.ratio_x == 1 && g.ratio_y == 1)
    return;
  const auto footprint = render_footprint_admitted(g, block, d);
  const int dx = g.pel * g.ratio_x, dy = g.pel * g.ratio_y;
  const auto ax = std::int64_t(block.x) * g.pel + d.x;
  const auto ay = std::int64_t(block.y) * g.pel + d.y;
  const int fx = render_sampling_detail::remainder(ax, dx) * (8 / dx);
  const int fy = render_sampling_detail::remainder(ay, dy) * (8 / dy);
  if (!fx && !fy)
    return; // Existing descriptor already borrows the exact integer samples.

  const auto base = image.planes[0];
  const auto ox = g.pad_x + sampling_detail::floor_div(ax, dx);
  const auto oy = g.pad_y + sampling_detail::floor_div(ay, dy);
  const int width = footprint.width, height = footprint.height;
  const auto& horizontal = chroma_wiener_detail::coefficients[fx];
  const auto& vertical = chroma_wiener_detail::coefficients[fy];
  using A = std::conditional_t<std::is_floating_point_v<T>, double, std::int64_t>;
  const int top = fy ? -2 : 0, rows = height + (fy ? 5 : 0);
  // Keep signed horizontal results without clipping. Even 16-bit input at
  // the largest absolute coefficient sum (208) fits comfortably in int64.
  std::vector<A> intermediate(std::size_t(width) * rows);
  for (int y = 0; y < rows; ++y) {
    const int sy = int(std::clamp<std::int64_t>(oy + top + y, 0, base.height() - 1));
    const auto row = base.row(sy);
    for (int x = 0; x < width; ++x) {
      A value = 0;
      for (int tap = 0; tap < 6; ++tap)
        if (horizontal[tap]) {
          const int sx = int(std::clamp<std::int64_t>(ox + x + tap - 2, 0, base.width() - 1));
          value += A(row[sx]) * horizontal[tap];
        }
      intermediate[std::size_t(y) * width + x] = value;
    }
  }
  const auto maximum = subpixel_detail::sample_max<T>(bits);
  storage.emplace_back(std::size_t(width) * height);
  auto& pixels = storage.back();
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      A value = 0;
      for (int tap = 0; tap < 6; ++tap)
        if (vertical[tap])
          value += intermediate[std::size_t(y + tap - 2 - top) * width + x] * vertical[tap];
      if constexpr (std::is_floating_point_v<T>) {
        const auto normalized = value / 16384.0;
        if (!std::isfinite(normalized) || std::abs(normalized) > std::numeric_limits<T>::max())
          throw std::invalid_argument("non-finite Wiener chroma sample");
        pixels[std::size_t(y) * width + x] = T(normalized);
      } else
        pixels[std::size_t(y) * width + x] = subpixel_detail::round_clip<T>(value, 14, maximum);
    }
  output.data = pixels.data();
  output.stride = width;
}
} // namespace neo_mv
