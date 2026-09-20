#pragma once

#include "core/mask/grid_resampling.hpp"
#include "core/motion/analysis_field.hpp"

#include <vector>

namespace neo_mv {

struct DenseFlowField {
  int width, height;
  std::vector<std::int16_t> x, y;
};

namespace dense_detail {
inline GridResamplingGeometry geometry(const AnalysisMetadata& m, int ratio_x, int ratio_y) {
  if (!valid_analysis_metadata(m) || (ratio_x != 1 && ratio_x != 2) || (ratio_y != 1 && ratio_y != 2))
    throw std::invalid_argument("invalid dense flow metadata or plane ratios");
  if (m.block_width % ratio_x || m.overlap_x % ratio_x || m.real_width % ratio_x || m.block_height % ratio_y ||
      m.overlap_y % ratio_y || m.real_height % ratio_y)
    throw std::invalid_argument("dense flow geometry is not aligned to plane ratios");
  const auto width = std::int64_t(m.blocks_x) * (m.block_width - m.overlap_x) + m.overlap_x;
  const auto height = std::int64_t(m.blocks_y) * (m.block_height - m.overlap_y) + m.overlap_y;
  if (width < m.real_width || width > m.width || height < m.real_height || height > m.height)
    throw std::invalid_argument("dense flow grid does not cover visible image within working image");
  return {m.blocks_x,
          m.blocks_y,
          m.block_width / ratio_x,
          m.block_height / ratio_y,
          m.overlap_x / ratio_x,
          m.overlap_y / ratio_y,
          m.real_width / ratio_x,
          m.real_height / ratio_y};
}

inline std::size_t count(int width, int height) {
  const auto count = std::uint64_t(width) * height;
  if (width <= 0 || height <= 0 || count > std::vector<std::int16_t>().max_size() ||
      count > static_cast<std::uintmax_t>(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(std::int16_t))
    throw std::overflow_error("dense flow storage size is unrepresentable");
  return static_cast<std::size_t>(count);
}

template <class T>
span2d::Plane<T> plane(T* data, int width, int height, std::size_t samples) {
  if (static_cast<std::uintmax_t>(width) >
      static_cast<std::uintmax_t>(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(T))
    throw std::overflow_error("dense flow row stride is unrepresentable");
  return checked_plane(data, width, height, static_cast<std::ptrdiff_t>(width) * sizeof(T), samples * sizeof(T));
}

inline std::int16_t small_component(std::int64_t value, int ratio) {
  const auto clipped = std::clamp(value, std::int64_t{-32768}, std::int64_t{32767});
  const auto quotient = clipped / ratio - (clipped < 0 && clipped % ratio != 0 ? 1 : 0);
  return static_cast<std::int16_t>(quotient);
}
} // namespace dense_detail

// Plane-specific signed fields. The shared resampler is the only dispatched
// operation; saturation and chroma conversion always precede interpolation.
template <class Resampler = GridResamplingPlan>
class DenseFlowPlan {
  AnalysisMetadata metadata_;
  int ratio_x_, ratio_y_;
  GridResamplingGeometry geometry_;
  Resampler resampling_;

public:
  DenseFlowPlan(AnalysisMetadata metadata, int ratio_x, int ratio_y)
      : metadata_(metadata), ratio_x_(ratio_x), ratio_y_(ratio_y),
        geometry_(dense_detail::geometry(metadata, ratio_x, ratio_y)), resampling_(geometry_) {}

  const AnalysisMetadata& metadata() const { return metadata_; }
  const GridResamplingGeometry& geometry() const { return geometry_; }

  DenseFlowField generate(const MotionGrid& grid, int field_shift) const {
    const auto& m = metadata_;
    if (field_shift != 0 && (m.pel == 1 || m.delta % 2 == 0 || (field_shift != m.pel / 2 && field_shift != -m.pel / 2)))
      throw std::invalid_argument("invalid dense flow field correction");
    if (grid.width != m.blocks_x || grid.height != m.blocks_y || grid.values.size() != field_detail::count(m))
      throw std::invalid_argument("inconsistent dense flow motion grid");
    // Validate every public entry before allocating or clipping. Saturating a
    // malformed vector must never turn it into an accepted displacement.
    for (std::size_t i = 0; i < grid.values.size(); ++i)
      field_detail::vector(m, grid.values[i], static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
    const auto small_count = dense_detail::count(m.blocks_x, m.blocks_y);
    const auto output_count = dense_detail::count(geometry_.width, geometry_.height);
    std::vector<std::int16_t> small_x(small_count), small_y(small_count);
    for (std::size_t i = 0; i < grid.values.size(); ++i) {
      const auto vector = grid.values[i].vector;
      small_x[i] = dense_detail::small_component(vector.x, ratio_x_);
      small_y[i] = dense_detail::small_component(std::int64_t(vector.y) + field_shift, ratio_y_);
    }
    DenseFlowField output{geometry_.width, geometry_.height, std::vector<std::int16_t>(output_count),
                          std::vector<std::int16_t>(output_count)};
    const auto x_input =
        dense_detail::plane(static_cast<const std::int16_t*>(small_x.data()), m.blocks_x, m.blocks_y, small_x.size());
    const auto y_input =
        dense_detail::plane(static_cast<const std::int16_t*>(small_y.data()), m.blocks_x, m.blocks_y, small_y.size());
    resampling_.resize(x_input, dense_detail::plane(output.x.data(), output.width, output.height, output.x.size()), 16);
    resampling_.resize(y_input, dense_detail::plane(output.y.data(), output.width, output.height, output.y.size()), 16);
    return output;
  }
};

} // namespace neo_mv
