#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace neo_mv {
enum class MotionMetric { sad, satd, dct, sad_dct_global, sad_dct_local,
                          sad_satd_global, sad_satd_local, sad_satd_global_half };
enum class MetricTransform { sad, satd, dct };
enum class MetricMix { none, global, local };
struct MetricDescriptor {
  std::string_view name;
  MetricTransform transform;
  MetricMix mix;
  int dc_weight;
  int global_divisor;
  constexpr bool integer_only() const { return transform == MetricTransform::dct || mix != MetricMix::none; }
};
inline constexpr std::array<MetricDescriptor, 8> motion_metrics{{
    {"sad", MetricTransform::sad, MetricMix::none, 1, 1},
    {"satd", MetricTransform::satd, MetricMix::none, 1, 1},
    {"dct", MetricTransform::dct, MetricMix::none, 4, 1},
    {"sad_dct_global", MetricTransform::dct, MetricMix::global, 4, 1},
    {"sad_dct_local", MetricTransform::dct, MetricMix::local, 1, 1},
    {"sad_satd_global", MetricTransform::satd, MetricMix::global, 1, 1},
    {"sad_satd_local", MetricTransform::satd, MetricMix::local, 1, 1},
    {"sad_satd_global_half", MetricTransform::satd, MetricMix::global, 1, 2},
}};
inline const MetricDescriptor& metric_descriptor(MotionMetric metric) {
  const auto i = static_cast<std::size_t>(metric);
  if (i >= motion_metrics.size()) throw std::invalid_argument("unknown motion metric");
  return motion_metrics[i];
}
inline MotionMetric parse_motion_metric(std::string_view value) {
  for (std::size_t i = 0; i < motion_metrics.size(); ++i)
    if (motion_metrics[i].name == value) return static_cast<MotionMetric>(i);
  throw std::invalid_argument("metric must be sad, satd, dct, sad_dct_global, sad_dct_local, "
                              "sad_satd_global, sad_satd_local or sad_satd_global_half");
}
inline constexpr std::int64_t metric_unit = 65536;
// Multiplication by a power of two is exact here. Subtraction from floor is
// exact near a rounding boundary; do not add 0.5 (rounding-mode dependent).
inline std::int64_t quantize_metric_parameter(double value) {
  if (!std::isfinite(value) || value < 0 || value > 1)
    throw std::invalid_argument("metric_weight and metric_threshold must be finite and in [0,1]");
  const double scaled = value * double(metric_unit);
  const auto whole = static_cast<std::int64_t>(scaled);
  return whole + (scaled - double(whole) >= 0.5);
}
struct MetricConfig {
  MotionMetric mode = MotionMetric::sad;
  std::int64_t weight = metric_unit / 2, threshold = metric_unit / 32;
  constexpr MetricConfig() = default;
  constexpr MetricConfig(MotionMetric value) : mode(value) {}
};
inline MetricConfig make_metric_config(std::string_view name, std::optional<double> weight = {},
                                      std::optional<double> threshold = {}) {
  MetricConfig result(parse_motion_metric(name));
  if (metric_descriptor(result.mode).mix != MetricMix::local && (weight || threshold))
    throw std::invalid_argument("metric_weight and metric_threshold require a local metric");
  result.weight = quantize_metric_parameter(weight.value_or(0.5));
  result.threshold = quantize_metric_parameter(threshold.value_or(0.03125));
  return result;
}
inline void validate_motion_metric(MetricConfig config, int width, int height, int bits) {
  const auto& d = metric_descriptor(config.mode);
  if (d.transform == MetricTransform::satd && (width % 4 || height % 4))
    throw std::invalid_argument("SATD requires block width and height divisible by 4");
  if (d.integer_only() && (bits < 8 || bits > 16))
    throw std::invalid_argument("DCT and mixed metrics require 8-16 bit integer samples");
  if (config.weight < 0 || config.weight > metric_unit || config.threshold < 0 || config.threshold > metric_unit)
    throw std::invalid_argument("invalid quantized metric configuration");
}
// At most 128*128 uint16 pixels: S <= area*65535, T <= 8*area*65535,
// D4 <= (area+3)*65535*128/2. Multiplication by Q fits signed int64.
inline std::int64_t mix_metric(std::int64_t sad, std::int64_t transformed, std::int64_t weight,
                               std::int64_t unit = metric_unit) {
  return (sad * (unit - weight) + transformed * weight) / unit;
}
inline bool local_metric_active(std::int64_t source, std::int64_t reference, std::int64_t threshold) {
  return std::abs(source - reference) * metric_unit > (source + reference) * threshold;
}
} // namespace neo_mv
