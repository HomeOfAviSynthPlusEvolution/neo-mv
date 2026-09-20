#pragma once

#include "core/motion/prediction.hpp"

#include <string>

namespace neo_mv {

struct AnalysisMetadata {
  std::int32_t width = 0, height = 0, real_width = 0, real_height = 0;
  std::int32_t pad_x = 0, pad_y = 0, pel = 0, levels = 0;
  bool chroma = false;
  std::int32_t ratio_x = 0, ratio_y = 0, block_width = 0, block_height = 0;
  std::int32_t overlap_x = 0, overlap_y = 0, blocks_x = 0, blocks_y = 0, delta = 0, bits = 0;
};
enum class FieldState { invalid_metadata, metadata_only, complete };
struct AnalysisField {
  AnalysisMetadata metadata;
  FieldState state = FieldState::invalid_metadata;
  MotionGrid grid{0, 0, {}};
};

// A read-only host adapter supplies typed property views, with live integer
// elements for the duration of read_analysis_field. Missing = count 0. Wrong
// types retain their real element count, but need no integer data pointer.
// The decoder copies every accepted value; no borrowed pointer escapes it.
struct IntegerPropertyView {
  bool integer = false;
  std::uint64_t count = 0;
  const std::int64_t* data = nullptr;
};
using AnalysisProperties = std::map<std::string, std::vector<std::int64_t>>;

namespace field_detail {
struct Scalar {
  const char* suffix;
  std::int32_t AnalysisMetadata::* member;
};
inline constexpr Scalar scalars[] = {{"AnalysisWidth", &AnalysisMetadata::width},
                                     {"AnalysisHeight", &AnalysisMetadata::height},
                                     {"AnalysisRealWidth", &AnalysisMetadata::real_width},
                                     {"AnalysisRealHeight", &AnalysisMetadata::real_height},
                                     {"AnalysisHPad", &AnalysisMetadata::pad_x},
                                     {"AnalysisVPad", &AnalysisMetadata::pad_y},
                                     {"AnalysisPel", &AnalysisMetadata::pel},
                                     {"AnalysisLevels", &AnalysisMetadata::levels},
                                     {"AnalysisXRatioUV", &AnalysisMetadata::ratio_x},
                                     {"AnalysisYRatioUV", &AnalysisMetadata::ratio_y},
                                     {"AnalysisBlkSizeX", &AnalysisMetadata::block_width},
                                     {"AnalysisBlkSizeY", &AnalysisMetadata::block_height},
                                     {"AnalysisOverlapX", &AnalysisMetadata::overlap_x},
                                     {"AnalysisOverlapY", &AnalysisMetadata::overlap_y},
                                     {"AnalysisNBlkX", &AnalysisMetadata::blocks_x},
                                     {"AnalysisNBlkY", &AnalysisMetadata::blocks_y},
                                     {"AnalysisDeltaFrame", &AnalysisMetadata::delta},
                                     {"AnalysisBitsPerSample", &AnalysisMetadata::bits}};
inline std::int64_t scalar(IntegerPropertyView view) {
  if (!view.integer || view.count == 0)
    return 0;
  if (!view.data)
    throw std::invalid_argument("null integer property storage");
  return view.data[0];
}
inline std::uint64_t count(const AnalysisMetadata& m) {
  return std::uint64_t(m.blocks_x) * m.blocks_y;
}
inline CandidateDomain bounds(const AnalysisMetadata& m, int bx, int by) {
  const auto x = std::int64_t(m.pad_x) + std::int64_t(bx) * (m.block_width - m.overlap_x);
  const auto y = std::int64_t(m.pad_y) + std::int64_t(by) * (m.block_height - m.overlap_y);
  return {prediction_detail::weight(-x, m.pel), prediction_detail::weight(-y, m.pel),
          prediction_detail::weight(std::int64_t(m.width) + 2 * std::int64_t(m.pad_x) - x - m.block_width, m.pel),
          prediction_detail::weight(std::int64_t(m.height) + 2 * std::int64_t(m.pad_y) - y - m.block_height, m.pel)};
}
inline void vector(const AnalysisMetadata& m, MotionTriple v, int bx, int by) {
  const auto domain = bounds(m, bx, by);
  if (v.error < 0 || v.vector.x < domain.left || v.vector.x >= domain.right || v.vector.y < domain.top ||
      v.vector.y >= domain.bottom)
    throw std::invalid_argument("malformed analysis vector or error");
}
} // namespace field_detail

inline bool valid_analysis_metadata(const AnalysisMetadata& m) {
  if (m.width <= 0 || m.height <= 0 || m.real_width <= 0 || m.real_height <= 0 || m.width < m.real_width ||
      m.height < m.real_height || m.pad_x < 0 || m.pad_y < 0 || (m.pel != 1 && m.pel != 2 && m.pel != 4) ||
      m.levels < 1 || (m.ratio_x != 1 && m.ratio_x != 2) || (m.ratio_y != 1 && m.ratio_y != 2) || m.block_width < 2 ||
      m.block_height < 2 || m.overlap_x < 0 || m.overlap_y < 0 || m.overlap_x > m.block_width / 2 ||
      m.overlap_y > m.block_height / 2 || m.blocks_x <= 0 || m.blocks_y <= 0 ||
      !((m.bits >= 8 && m.bits <= 16) || m.bits == 32))
    return false;
  // All endpoint expressions are monotone in the block indices. Check both
  // extremes before inspecting arrays, even for a metadata-only read.
  field_detail::bounds(m, 0, 0);
  field_detail::bounds(m, m.blocks_x - 1, m.blocks_y - 1);
  return true;
}

inline MotionVector unpack_vector(std::int64_t packed) {
  const auto pattern = static_cast<std::uint64_t>(packed);
  const auto signed_half = [](std::uint32_t half) {
    const auto value = half <= INT32_MAX ? std::int64_t(half) : std::int64_t(half) - 4294967296LL;
    return static_cast<std::int32_t>(value);
  };
  return {signed_half(static_cast<std::uint32_t>(pattern)), signed_half(static_cast<std::uint32_t>(pattern >> 32))};
}
inline std::int64_t pack_vector(MotionVector vector) {
  const auto pattern =
      std::uint64_t(static_cast<std::uint32_t>(vector.x)) | (std::uint64_t(static_cast<std::uint32_t>(vector.y)) << 32);
  return pattern <= INT64_MAX ? static_cast<std::int64_t>(pattern) : -1 - static_cast<std::int64_t>(~pattern);
}

// property(key) returns IntegerPropertyView without coercing wrong types.
template <class PropertyReader>
AnalysisField read_analysis_field(PropertyReader&& property, bool read_vectors = true,
                                  const std::string& prefix = "MVUtensils") {
  AnalysisField result;
  auto& m = result.metadata;
  for (auto scalar : field_detail::scalars)
    m.*(scalar.member) = static_cast<std::int32_t>(std::clamp(field_detail::scalar(property(prefix + scalar.suffix)),
                                                              std::int64_t(INT32_MIN), std::int64_t(INT32_MAX)));
  m.chroma = field_detail::scalar(property(prefix + "AnalysisChroma")) != 0;
  if (!valid_analysis_metadata(m))
    return result;
  result.state = FieldState::metadata_only;
  if (!read_vectors)
    return result;
  const auto vectors = property(prefix + "AnalysisVectors"), errors = property(prefix + "AnalysisSAD");
  const auto count = field_detail::count(m);
  // Compare BOTH counts before any array type or value check.
  if (vectors.count != count || errors.count != count)
    return result;
  if (!vectors.integer || !errors.integer)
    throw std::invalid_argument("analysis arrays must contain integers");
  if (!vectors.data || !errors.data)
    throw std::invalid_argument("null analysis array storage");
  if (count > result.grid.values.max_size())
    throw std::overflow_error("analysis grid allocation size is unrepresentable");
  result.grid = {m.blocks_x, m.blocks_y, {}};
  result.grid.values.resize(static_cast<std::size_t>(count));
  for (std::size_t i = 0; i < result.grid.values.size(); ++i) {
    const MotionTriple value{unpack_vector(vectors.data[i]), errors.data[i]};
    field_detail::vector(m, value, static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
    result.grid.values[i] = value;
  }
  result.state = FieldState::complete;
  return result;
}

// Produces canonical typed values in owned storage. A host writer must replace
// all scalar keys and remove both inherited array keys for metadata_only.
inline AnalysisProperties encode_analysis_field(const AnalysisField& field, const std::string& prefix = "MVUtensils") {
  const auto& m = field.metadata;
  if (field.state == FieldState::invalid_metadata || !valid_analysis_metadata(m))
    throw std::invalid_argument("cannot encode invalid analysis metadata");
  AnalysisProperties result;
  for (auto scalar : field_detail::scalars)
    result[prefix + scalar.suffix] = {m.*(scalar.member)};
  result[prefix + "AnalysisChroma"] = {m.chroma ? 1 : 0};
  if (field.state == FieldState::metadata_only)
    return result;
  if (field.state != FieldState::complete || field.grid.width != m.blocks_x || field.grid.height != m.blocks_y ||
      field.grid.values.size() != field_detail::count(m))
    throw std::invalid_argument("cannot encode inconsistent analysis grid");
  auto& vectors = result[prefix + "AnalysisVectors"];
  auto& errors = result[prefix + "AnalysisSAD"];
  vectors.reserve(field.grid.values.size());
  errors.reserve(field.grid.values.size());
  for (std::size_t i = 0; i < field.grid.values.size(); ++i) {
    const auto value = field.grid.values[i];
    field_detail::vector(m, value, static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
    vectors.push_back(pack_vector(value.vector));
    errors.push_back(value.error);
  }
  return result;
}

} // namespace neo_mv
