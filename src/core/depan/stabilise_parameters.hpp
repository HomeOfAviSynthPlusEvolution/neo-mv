#pragma once
#include "core/depan/transform.hpp"
#include <vector>

namespace neo_mv::depan::stabilise {
// Public floating arguments have already undergone binary64-to-binary32
// conversion; integer arguments have already saturated to int32.
struct Parameters {
  float cutoff = 1, damping = 0.9f, initzoom = 1;
  bool addzoom = false;
  int prev = 0, next = 0, mirror = 0, blur = 0;
  float dxmax = 60, dymax = 30, zoommax = 1.05f, rotmax = 1;
  int subpixel = 2;
  float pixaspect = 1;
  int fitlast = 0;
  float tzoom = 3;
  bool info = false;
  int method = 0;
  bool fields = false;
};
struct Coefficients {
  float fps, aspect, cx, cy, z0, zoom_limit;
  int radius, zoom_radius;
  std::vector<float> weights, zoom_weights;
  // Only method 0 evaluates these coefficients.
  float f = 0, cd = 0, cq = 0, kx = 0, ky = 0, kr = 0;
};
inline int truncate32(float value) {
  const double integer = std::trunc(double(finite(value)));
  if (integer < -2147483648.0 || integer > 2147483647.0)
    throw std::invalid_argument("DepanStabilise integer is unrepresentable");
  return static_cast<int>(integer);
}
inline float positive_integer32(std::int64_t value) {
  if (value <= 0)
    throw std::invalid_argument("nonpositive DepanStabilise rate");
  const auto n = static_cast<std::uint64_t>(value);
  int shift = 0;
  for (auto v = n; v > 0xffffffu; v >>= 1)
    ++shift;
  if (!shift)
    return f32(double(n));
  auto significant = n >> shift;
  const auto remainder = n & ((std::uint64_t(1) << shift) - 1);
  const auto half = std::uint64_t(1) << (shift - 1);
  if (remainder > half || (remainder == half && (significant & 1)))
    ++significant;
  return f32(std::ldexp(double(significant), shift));
}
inline Coefficients normalize(const Parameters& p, int width, int height, std::int64_t numerator,
                              std::int64_t denominator) {
  for (float v : {p.cutoff, p.damping, p.initzoom, p.dxmax, p.dymax, p.zoommax, p.rotmax, p.pixaspect, p.tzoom})
    finite(v);
  if (width <= 0 || height <= 0 || numerator <= 0 || denominator <= 0 || p.cutoff <= 0 || p.initzoom <= 0 ||
      p.pixaspect <= 0 || p.tzoom < 0 || p.prev < 0 || p.next < 0 || p.mirror < 0 || p.mirror > 15 || p.blur < 0 ||
      p.subpixel < 0 || p.subpixel > 2 || p.method < 0 || p.method > 1)
    throw std::invalid_argument("invalid DepanStabilise parameters");
  Coefficients c{};
  c.fps = div(positive_integer32(numerator), positive_integer32(denominator));
  c.aspect = div(p.pixaspect, p.fields ? 2.0f : 1.0f);
  c.cx = div(f32(width), 2);
  c.cy = div(f32(height), 2);
  c.z0 = div(1, p.initzoom);
  if (c.fps <= 0 || c.aspect <= 0 || c.cx <= 0 || c.cy <= 0 || c.z0 <= 0)
    throw std::invalid_argument("nonpositive DepanStabilise geometry");
  c.zoom_limit = p.zoommax > 0 ? (std::max)(p.zoommax, p.initzoom) : -(std::max)(-p.zoommax, p.initzoom);
  c.radius = (std::max)(1, truncate32(div(c.fps, mul(4, p.cutoff))));
  if (c.radius > 2147483646)
    throw std::invalid_argument("DepanStabilise radius is unrepresentable");
  c.zoom_radius = (std::min)(c.radius, truncate32(div(mul(c.fps, p.tzoom), 4)));
  if (p.method == 0) {
    const float b = add(1, mul(mul(6, p.damping), p.damping));
    const float lambda = sqrt32(add(b, sqrt32(add(mul(b, b), 3))));
    c.f = div(p.cutoff, lambda);
    if (c.f <= 0)
      throw std::invalid_argument("nonpositive DepanStabilise response");
    c.cd = div(mul(12.56f, p.damping), c.fps);
    c.cq = div(39.44f, mul(c.fps, c.fps));
    c.kx = p.dxmax != 0 ? div(5, std::abs(p.dxmax)) : 0;
    c.ky = p.dymax != 0 ? div(5, std::abs(p.dymax)) : 0;
    c.kr = p.rotmax != 0 ? div(5, std::abs(p.rotmax)) : 0;
  }
  c.weights.resize(std::size_t(c.radius) + 1, 0.0f);
  c.zoom_weights.resize(std::size_t(c.radius) + 1, 0.0f);
  for (int i = 0; i < c.radius; ++i)
    c.weights[i] = cos32(div(mul(mul(f32(i), 0.5f), pi), f32(c.radius)));
  for (int i = 0; i < c.zoom_radius; ++i)
    c.zoom_weights[i] = cos32(div(mul(mul(f32(i), 0.5f), pi), f32(c.zoom_radius)));
  return c;
}
} // namespace neo_mv::depan::stabilise
