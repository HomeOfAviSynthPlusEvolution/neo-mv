#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace neo_mv::depan::estimate {
struct GeometryParameters {
  std::int64_t winx = 0, winy = 0, wleft = -1, wtop = -1, dxmax = -1, dymax = -1;
  float zoommax = 1;
};
struct WindowGeometry {
  int width = 0, height = 0, left = 0, left2 = 0, top = 0, mx = 0, my = 0;
  bool two = false;
};

inline WindowGeometry make_geometry(int clip_width, int clip_height, GeometryParameters p = {}) {
  const auto require = [](bool valid) {
    if (!valid)
      throw std::invalid_argument("invalid DepanEstimate window geometry");
  };
  const auto saturated = [](std::int64_t value) {
    return std::clamp(value, std::int64_t(std::numeric_limits<std::int32_t>::min()),
                      std::int64_t(std::numeric_limits<std::int32_t>::max()));
  };
  const auto automatic = [](std::int64_t available) {
    const auto limit = std::min(available, std::int64_t(8192));
    std::int64_t result = 0;
    if (limit > 0) {
      result = 1;
      while (result <= limit / 2)
        result *= 2;
    }
    return result;
  };
  require(clip_width > 0 && clip_height > 0 && std::isfinite(p.zoommax));
  const std::int64_t w = clip_width, h = clip_height;
  const auto left0 = saturated(p.wleft), top0 = saturated(p.wtop);
  auto wx = saturated(p.winx), wy = saturated(p.winy);
  const auto available_x = w - std::max(left0, std::int64_t(0));
  const auto available_y = h - std::max(top0, std::int64_t(0));
  require(wx >= 0 && wx <= available_x && wx % 2 == 0);
  require(wy >= 0 && wy <= available_y);
  if (wx == 0)
    wx = automatic(available_x);
  if (wy == 0)
    wy = automatic(available_y);
  const bool two = p.zoommax != 1;
  if (two)
    wx /= 2;
  require(wx > 0 && wx % 2 == 0 && wy > 0);
  const auto left = left0 < 0 ? (two ? (w - 2 * wx) / 4 : (w - wx) / 2) : left0;
  const auto left2 = two ? left + w / 2 : left;
  const auto top = top0 < 0 ? (h - wy) / 2 : top0;
  require(left >= 0 && left + wx <= w && top >= 0 && top + wy <= h);
  require(!two || (left2 > left && left2 + wx <= w));
  auto mx = saturated(p.dxmax), my = saturated(p.dymax);
  if (mx < 0)
    mx = wx / 4;
  if (my < 0)
    my = wy / 4;
  require(mx >= 0 && mx < wx / 2 && my >= 0 && my < wy / 2);
  return {static_cast<int>(wx),    static_cast<int>(wy),  static_cast<int>(left), static_cast<int>(left2),
          static_cast<int>(top),   static_cast<int>(mx),  static_cast<int>(my),   two};
}
} // namespace neo_mv::depan::estimate
