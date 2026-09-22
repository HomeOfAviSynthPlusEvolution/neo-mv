#pragma once

#include "core/motion/analysis_field.hpp"

namespace neo_mv {

struct SceneDescriptor {
  std::int32_t block_width, block_height, blocks_x, blocks_y;
  bool chroma;
  std::int32_t ratio_x, ratio_y, bits;
};
inline SceneDescriptor scene_descriptor(const AnalysisMetadata& m) {
  return {m.block_width, m.block_height, m.blocks_x, m.blocks_y, m.chroma, m.ratio_x, m.ratio_y, m.bits};
}
struct SceneThresholds {
  std::int64_t error;
  float count;
};

inline std::int64_t scalar_scene_count(const AnalysisMetadata& m, const MotionGrid& grid, std::int64_t threshold) {
  std::int64_t bad = 0;
  for (std::size_t i = 0; i < grid.values.size(); ++i) {
    const auto value = grid.values[i];
    field_detail::vector(m, value, static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
    if (value.error > threshold)
      ++bad;
  }
  return bad;
}

inline SceneThresholds scene_thresholds(SceneDescriptor d, std::int64_t thscd1, double thscd2) {
  if (d.block_width < 2 || d.block_height < 2 || d.blocks_x <= 0 || d.blocks_y <= 0 ||
      (d.ratio_x != 1 && d.ratio_x != 2) || (d.ratio_y != 1 && d.ratio_y != 2) ||
      !((d.bits >= 8 && d.bits <= 16) || d.bits == 32) || thscd1 < 0 || thscd1 > 16320 || !std::isfinite(thscd2) ||
      std::abs(thscd2) > std::numeric_limits<float>::max())
    throw std::invalid_argument("invalid scene descriptor or threshold arguments");
  // Range-check AFTER conversion: values that round to 100 or -0 are valid.
  const float t2 = static_cast<float>(thscd2);
  if (t2 < 0 || t2 > 100)
    throw std::invalid_argument("scene percentage must round into [0,100]");
  const auto area = std::int64_t(d.block_width) * d.block_height;
  const double a = double(area) / 64.0;
  const double chroma = d.chroma ? 1.0 + 2.0 / double(d.ratio_x * d.ratio_y) : 1.0;
  const double depth = double((1 << std::min(16, d.bits)) - 1) / 255.0;
  const double error = double(thscd1) * ((a * chroma) * depth) + 0.5;
  if (!std::isfinite(error) || error >= 0x1p63)
    throw std::overflow_error("scene error threshold exceeds int64 range");
  const double count = ((double(t2) * d.blocks_x) * d.blocks_y) / 100.0;
  return {static_cast<std::int64_t>(error), static_cast<float>(count)};
}

// Immutable cache owned by the caller; independent filters share no state.
class SceneClassifier {
public:
  const SceneDescriptor descriptor;
  const SceneThresholds thresholds;

  SceneClassifier(SceneDescriptor d, std::int64_t thscd1, double thscd2)
      : descriptor(d), thresholds(scene_thresholds(d, thscd1, thscd2)) {}

  // The counter validates every vector and error, even after the threshold
  // has been exceeded. Descriptor and storage admission stay shared.
  using Counter = std::int64_t (*)(const AnalysisMetadata&, const MotionGrid&, std::int64_t);
  int operator()(const AnalysisField& field, Counter count = scalar_scene_count) const {
    if (field.state == FieldState::invalid_metadata)
      return 1;
    const auto& m = field.metadata;
    const auto d = descriptor;
    if (!valid_analysis_metadata(m) || m.block_width != d.block_width || m.block_height != d.block_height ||
        m.blocks_x != d.blocks_x || m.blocks_y != d.blocks_y || m.chroma != d.chroma || m.ratio_x != d.ratio_x ||
        m.ratio_y != d.ratio_y || m.bits != d.bits)
      throw std::invalid_argument("scene field descriptor mismatch");
    if (field.state == FieldState::metadata_only)
      return 1;
    if (field.state != FieldState::complete || field.grid.width != d.blocks_x || field.grid.height != d.blocks_y ||
        field.grid.values.size() != std::uint64_t(d.blocks_x) * d.blocks_y)
      throw std::invalid_argument("malformed complete scene field");
    const auto bad = count(m, field.grid, thresholds.error);
    return static_cast<float>(bad) > thresholds.count ? 1 : 0;
  }
};

} // namespace neo_mv
