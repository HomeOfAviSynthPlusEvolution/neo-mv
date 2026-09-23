#pragma once

#include "core/base/plane.hpp"
#include "core/depan/numeric.hpp"
#include "core/depan/transform.hpp"

#include <optional>
#include <vector>

namespace neo_mv::depan::estimate {
struct Peak {
  int ix = 0, iy = 0, dx = 0, dy = 0;
  float confidence = 0;
  bool good = false;
};
struct WindowMotion {
  float dx = 0, dy = 0, confidence = 0;
  bool good = false;
};
struct BasicMotion {
  float dx = 0, dy = 0, zoom = 1, confidence = 0;
  bool good = false;
};
namespace motion_detail {
struct ScalarScan {
  static void validate(const float* values, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i)
      finite(values[i]);
  }
  // Return count when the initial maximum is not exceeded. Addition remains
  // ordered even when another policy vectorizes the maximum search.
  static std::size_t scan(const float* values, std::size_t count, float& sum, float& maximum) {
    std::size_t index = count;
    for (std::size_t i = 0; i < count; ++i) {
      sum = add(sum, values[i]);
      if (values[i] > maximum) {
        maximum = values[i];
        index = i;
      }
    }
    return index;
  }
};
// Round exact integer counts directly, avoiding a binary64 double rounding.
inline float count32(std::uint64_t n) {
  int shift = 0;
  for (auto v = n; v > 0xffffffu; v >>= 1)
    ++shift;
  if (!shift)
    return f32(static_cast<double>(n));
  auto significand = n >> shift;
  const auto remainder = n & ((std::uint64_t{1} << shift) - 1);
  const auto half = std::uint64_t{1} << (shift - 1);
  if (remainder > half || (remainder == half && (significand & 1)))
    ++significand;
  return f32(std::ldexp(static_cast<double>(significand), shift));
}
template <class Scan = ScalarScan, bool SamplesValidated = false>
inline void surface_valid(span2d::Plane<const float> surface, int mx, int my) {
  validate_plane(surface);
  if (surface.width() % 2 || mx < 0 || my < 0 || mx >= surface.width() / 2 || my >= surface.height() / 2)
    throw std::invalid_argument("invalid DepanEstimate search geometry");
  if constexpr (!SamplesValidated)
    for (int y = 0; y < surface.height(); ++y)
      Scan::validate(surface.row(y).data(), static_cast<std::size_t>(surface.width()));
}
inline void trust_valid(float trust) {
  if (finite(trust) < 0 || trust > 100)
    throw std::invalid_argument("invalid DepanEstimate trust");
}
inline float fraction(float cm, float c0, float cp, int displacement, int limit) {
  const float f1 = div(sub(cp, cm), 2);
  const float f2 = sub(add(cp, cm), mul(2, c0));
  float a = f2 == 0 ? 0.0f : (std::min)(1.0f, (std::max)(-1.0f, div(-f1, f2)));
  if (std::abs(add(f32(displacement), a)) > f32(limit))
    a = 0;
  return a;
}
inline void window_valid(WindowMotion m) {
  finite(m.dx);
  finite(m.dy);
  finite(m.confidence);
}
inline void index_valid(int n, int frames) {
  if (frames <= 0 || n < 0 || n >= frames)
    throw std::invalid_argument("invalid DepanEstimate observation index");
}
} // namespace motion_detail

// SamplesValidated is reserved for immutable output of a checked inverse FFT.
template <class Scan = motion_detail::ScalarScan, bool SamplesValidated = false>
inline Peak find_peak(span2d::Plane<const float> surface, int mx, int my, float stab, float trust) {
  motion_detail::surface_valid<Scan, SamplesValidated>(surface, mx, my);
  motion_detail::trust_valid(trust);
  finite(stab);
  Peak result;
  float maximum = surface.row(0)[0], sum = 0;
  const int width = surface.width(), height = surface.height();
  // Mapped iteration visits the positive interval before the negative interval.
  for (int j = 0; j < 2 * my + 1; ++j) {
    const int y = j <= my ? j : height - (2 * my + 1 - j);
    const auto scan_interval = [&](int start, int count) {
      const auto index = Scan::scan(surface.row(y).data() + start, static_cast<std::size_t>(count), sum, maximum);
      if (index < static_cast<std::size_t>(count)) {
        result.ix = start + static_cast<int>(index);
        result.iy = y;
      }
    };
    scan_interval(0, mx + 1);
    scan_interval(width - mx, mx);
  }
  result.dx = std::int64_t(result.ix) * 2 < width ? result.ix : result.ix - width;
  result.dy = std::int64_t(result.iy) * 2 < height ? result.iy : result.iy - height;
  const float k = motion_detail::count32(std::uint64_t(width) * height);
  const float j = motion_detail::count32((2 * std::uint64_t(mx) + 1) * (2 * std::uint64_t(my) + 1));
  const float p = div(maximum, k), mean = div(div(sum, j), k);
  const float t0 = div(mul(sub(p, mean), 100), add(p, 0.1f));
  const float ax = f32(mx + 1), ay = f32(my + 1);
  const float denominator_x = add(ax, mul(stab, f32(std::abs(result.dx))));
  const float denominator_y = add(ay, mul(stab, f32(std::abs(result.dy))));
  const float scale = div(mul(div(ax, denominator_x), ay), denominator_y);
  result.confidence = mul(t0, scale);
  result.good = result.confidence >= trust;
  return result;
}

template <class Scan = motion_detail::ScalarScan, bool SamplesValidated = false>
inline WindowMotion refine_motion(span2d::Plane<const float> surface, Peak peak, int mx, int my, float aspect,
                                  bool fields, bool top) {
  motion_detail::surface_valid<Scan, SamplesValidated>(surface, mx, my);
  if (finite(aspect) <= 0)
    throw std::invalid_argument("invalid DepanEstimate aspect");
  finite(peak.confidence);
  const int width = surface.width(), height = surface.height();
  if (peak.ix < 0 || peak.ix >= width || peak.iy < 0 || peak.iy >= height ||
      peak.dx != (std::int64_t(peak.ix) * 2 < width ? peak.ix : peak.ix - width) ||
      peak.dy != (std::int64_t(peak.iy) * 2 < height ? peak.iy : peak.iy - height) || std::abs(peak.dx) > mx ||
      std::abs(peak.dy) > my)
    throw std::invalid_argument("invalid DepanEstimate peak");
  if (!peak.good)
    return {0, 0, peak.confidence, false};
  const int xm = peak.ix == 0 ? width - 1 : peak.ix - 1;
  const int xp = peak.ix == width - 1 ? 0 : peak.ix + 1;
  const int ym = peak.iy == 0 ? height - 1 : peak.iy - 1;
  const int yp = peak.iy == height - 1 ? 0 : peak.iy + 1;
  const float c0 = surface.row(peak.iy)[peak.ix];
  const float ax = motion_detail::fraction(surface.row(peak.iy)[xm], c0, surface.row(peak.iy)[xp], peak.dx, mx);
  float ay = motion_detail::fraction(surface.row(ym)[peak.ix], c0, surface.row(yp)[peak.ix], peak.dy, my);
  std::int64_t dy = peak.dy;
  if (fields) {
    ay = mul(add(ay, top ? 0.5f : -0.5f), 2);
    dy *= 2;
  }
  return {add(f32(peak.dx), ax), div(add(f32(static_cast<double>(dy)), ay), aspect), peak.confidence, true};
}

inline BasicMotion combine(WindowMotion first, std::optional<WindowMotion> second, float zoommax, int separation,
                           std::int64_t n) {
  motion_detail::window_valid(first);
  finite(zoommax);
  if (n < 0)
    throw std::invalid_argument("negative DepanEstimate observation index");
  BasicMotion result{first.dx, first.dy, 1, first.confidence, first.good};
  if (zoommax != 1) {
    if (!second || separation <= 0)
      throw std::invalid_argument("missing DepanEstimate second window or separation");
    motion_detail::window_valid(*second);
    const float zoom = add(1, div(sub(second->dx, first.dx), f32(separation)));
    const float delta = std::abs(sub(zoom, 1)), limit = sub(zoommax, 1);
    result.confidence = second->confidence < first.confidence ? second->confidence : first.confidence;
    result.good = first.good && second->good && delta < limit;
    if (result.good) {
      result.dx = div(add(first.dx, second->dx), 2);
      result.dy = div(add(first.dy, second->dy), 2);
      result.zoom = zoom;
    } else {
      result.dx = result.dy = 0;
      result.zoom = 1;
    }
  }
  if (n == 0) {
    result.good = false;
    result.confidence = 0;
  }
  return result;
}

inline Motion temporal_motion(BasicMotion current, std::optional<float> previous, std::optional<float> next,
                              float trust) {
  motion_detail::trust_valid(trust);
  for (float x : {current.dx, current.dy, current.zoom, current.confidence})
    finite(x);
  if (previous)
    finite(*previous);
  if (next)
    finite(*next);
  bool good = current.good;
  if (current.confidence < mul(trust, 2)) {
    if (previous && current.confidence < mul(0.5f, *previous))
      good = false;
    if (next && current.confidence < mul(0.5f, *next))
      good = false;
  }
  return good ? Motion{current.dx, current.dy, 0, current.zoom, true} : Motion{};
}
inline std::vector<int> required_basic_indices(int n, int frames) {
  motion_detail::index_valid(n, frames);
  std::vector<int> result;
  for (int j = (std::max)(0, n - 1), end = (std::min)(frames - 1, n + 1); j <= end; ++j)
    result.push_back(j);
  return result;
}
inline std::vector<int> required_source_indices(int n, int frames) {
  motion_detail::index_valid(n, frames);
  std::vector<int> result;
  for (int j = (std::max)(0, n - 2), end = (std::min)(frames - 1, n + 1); j <= end; ++j)
    result.push_back(j);
  return result;
}
} // namespace neo_mv::depan::estimate
