#pragma once

#include <dualsynth/video_filter.hpp>
#include "core/motion/analyse.hpp"
#include "core/motion/recalculate.hpp"
#include "core/motion/scene_classification.hpp"
#include "core/motion/super_input.hpp"
#include <cstring>
#include <memory>
#include <optional>
#include <variant>

namespace neo_mv::ds2 {
// These host runtimes own fields produced by the fully validating decoder.
// Request state is not modified between eligibility and pixel processing.
// Standalone core/kernel APIs continue to use validating counters.
template <class Kernels>
struct DecodedFieldKernels : Kernels {
  static constexpr auto scene_count = Kernels::scene_count_validated;
};

inline void require(bool ok, const char* message) {
  if (!ok)
    throw std::invalid_argument(message);
}
template <class T>
T unwrap(ds::Result<T> result) {
  if (!result.has_value())
    throw std::runtime_error(result.error().message);
  return std::move(result.value());
}
inline int saturate(std::int64_t value) {
  return static_cast<int>(std::clamp(value, std::int64_t(INT32_MIN), std::int64_t(INT32_MAX)));
}
struct Params {
  const ds::ParamValues& values;
  bool present(const std::string& key) const {
    for (const auto& entry : values.entries)
      if (entry.name == key)
        return true;
    return false;
  }
  int integer(const char* key, int fallback) const { return saturate(unwrap(values.get_int64(key, fallback))); }
  bool boolean(const char* key, bool fallback) const { return unwrap(values.get_bool(key, fallback)); }
  std::array<int, 2> pair(const char* key, std::array<int, 2> fallback) const {
    auto array = unwrap(values.get_int_array(key, {}));
    require(array.size() <= 2, "axis array must contain at most two elements");
    if (array.empty())
      return fallback;
    return {saturate(array[0]), saturate(array.size() == 1 ? array[0] : array[1])};
  }
  std::string prefix() const {
    auto value = unwrap(values.get_string("prefix", "MVUtensils"));
    require(value.find('\0') == std::string::npos, "prefix cannot contain NUL");
    return value;
  }
  std::optional<bool> tff() const { return present("tff") ? std::optional<bool>(boolean("tff", false)) : std::nullopt; }
};
inline ds::VideoOutputInfo output_info(const ds::VideoInputInfo& in) {
  return {in.width, in.height, in.num_frames, in.format, in.fps};
}
inline const ds::FrameProperties& properties(const ds::VideoFrameView& frame) {
  require(frame.properties != nullptr, "missing frame properties");
  return *frame.properties;
}
inline ds::FrameProperties& properties(ds::MutableVideoFrameView frame) {
  require(frame.properties != nullptr, "missing writable frame properties");
  return *frame.properties;
}
template <class T>
std::vector<T> property_array(const ds::FrameProperties& props, const std::string& key) {
  auto value = props.find(key);
  if (!value)
    throw std::invalid_argument("missing required property: " + key);
  const auto* array = std::get_if<std::vector<T>>(&*value);
  if (!array)
    throw std::invalid_argument("incorrect property type: " + key);
  return *array;
}
inline std::int64_t scalar(const ds::FrameProperties& props, const std::string& key) {
  const auto a = property_array<std::int64_t>(props, key);
  require(a.size() == 1, "expected exactly one integer property element");
  return a[0];
}
inline void set_scalar(ds::FrameProperties& props, const std::string& key, std::int64_t value) {
  props.set(key, std::vector<std::int64_t>{value});
}
inline ds::RequestedVideoFrame frame(ds::VideoFrameProvider& provider, int input, int n) {
  return unwrap(provider.get(input, n));
}
inline void validate_format(const ds::VideoInputInfo& in) {
  require(in.width > 0 && in.height > 0 && in.num_frames > 0, "fixed video geometry required");
  const auto& f = in.format;
  require(
      (f.color_family == ds::ColorFamily::Gray && f.plane_count == 1 && f.subsampling_w == 0 && f.subsampling_h == 0) ||
          (f.color_family == ds::ColorFamily::Yuv && f.plane_count == 3 && f.subsampling_w >= 0 &&
           f.subsampling_w <= 1 && f.subsampling_h >= 0 && f.subsampling_h <= 1),
      "Super requires GRAY/YUV with supported chroma ratios");
  const int bits = ds::bits_per_sample(f.sample_format);
  require((bits >= 8 && bits <= 16) || f.sample_format == ds::SampleFormat::Float32, "unsupported sample precision");
  require(in.width % (1 << f.subsampling_w) == 0 && in.height % (1 << f.subsampling_h) == 0,
          "unaligned chroma dimensions");
}
inline void validate_frame(const ds::VideoFrameView& f, const ds::VideoInputInfo& in) {
  require(f.format == in.format && f.plane_count == in.format.plane_count, "frame format changed");
  for (int k = 0; k < f.plane_count; ++k) {
    const auto& p = f.plane(k);
    require(p.width == (in.width >> (k ? in.format.subsampling_w : 0)) &&
                p.height == (in.height >> (k ? in.format.subsampling_h : 0)),
            "frame dimensions changed");
  }
}
template <class T>
span2d::Plane<const T> plane(const ds::PlaneView& p) {
  return checked_plane(static_cast<const T*>(p.data), p.width, p.height, p.stride_bytes,
                       std::numeric_limits<std::size_t>::max());
}
template <class T>
span2d::Plane<T> plane(const ds::MutablePlaneView& p) {
  return checked_plane(static_cast<T*>(p.data), p.width, p.height, p.stride_bytes,
                       std::numeric_limits<std::size_t>::max());
}
inline AnalysisField read_field(const ds::RequestedVideoFrame& frame, const std::string& prefix, bool vectors = true) {
  const auto& props = properties(frame.frame);
  std::map<std::string, std::vector<std::int64_t>> storage;
  auto read = [&](const std::string& key) -> IntegerPropertyView {
    const auto info = props.inspect(key);
    if (!info)
      return {};
    if (info->type != ds::PropertyType::Integer)
      return {false, info->count, nullptr};
    if (info->count == 0)
      return {true, 0, nullptr};
    auto found = storage.find(key);
    if (found == storage.end())
      found = storage.emplace(key, property_array<std::int64_t>(props, key)).first;
    require(found->second.size() == info->count, "analysis property count changed during read");
    return {true, info->count, found->second.data()};
  };
  auto metadata = read_analysis_field(read, false, prefix);
  if (!vectors || metadata.state == FieldState::invalid_metadata)
    return metadata;
  // Inspect both lengths before materializing either array, including arrays
  // of host object types which DS2 intentionally cannot read as values.
  const auto v = props.inspect(prefix + "AnalysisVectors"), s = props.inspect(prefix + "AnalysisSAD");
  const auto count = field_detail::count(metadata.metadata);
  if (!v || !s || v->count != count || s->count != count)
    return metadata;
  return read_analysis_field(read, true, prefix);
}
inline void write_field(ds::FrameProperties& props, const AnalysisField& field, const std::string& prefix) {
  auto encoded = encode_analysis_field(field, prefix);
  props.erase(prefix + "AnalysisVectors");
  props.erase(prefix + "AnalysisSAD");
  for (const auto& entry : encoded)
    props.set(entry.first, entry.second);
}
inline bool parity(const ds::VideoFrameView& frame, int n, std::optional<bool> tff) {
  if (tff)
    return *tff ^ ((n & 1) != 0);
  auto values = property_array<std::int64_t>(properties(frame), "_Field");
  require(!values.empty(), "fields requires readable _Field");
  return values[0] != 0;
}
inline bool same_metadata(AnalysisMetadata a, AnalysisMetadata b, bool compare_delta = true) {
  for (auto field : field_detail::scalars) {
    if (!compare_delta && field.member == &AnalysisMetadata::delta)
      continue;
    if (a.*field.member != b.*field.member)
      return false;
  }
  return a.chroma == b.chroma;
}
struct Runtime {
  ds::VideoInputInfo source;
  explicit Runtime(ds::VideoInputInfo in) : source(in) {}
  virtual ~Runtime() = default;
  virtual void request(ds::VideoRequestContext&) const = 0;
  virtual void process(ds::VideoProcessContext&) const = 0;
  virtual ds::VideoRequestPattern pattern(int) const { return ds::VideoRequestPattern::General; }
};
} // namespace neo_mv::ds2
