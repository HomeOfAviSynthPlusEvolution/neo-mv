#pragma once
#include "core/render/block_sampling.hpp"

namespace neo_mv::flow_coordinates {
struct Location {
  int phase;
  std::int64_t x, y;
};
inline int shift(int pel) {
  return pel == 4 ? 2 : pel == 2 ? 1 : 0;
}
inline std::int64_t floor_shift(std::int64_t value, int bits) {
  return value >= 0 ? value >> bits : -1 - ((-1 - value) >> bits);
}
// Decompose displacement alone: the integer pixel origin cannot change phase.
inline Location locate(const RenderPhaseGeometry& g, int x, int y, std::int64_t dx, std::int64_t dy) {
  const int bits = shift(g.pel);
  const auto qx = floor_shift(dx, bits), qy = floor_shift(dy, bits);
  return {static_cast<int>((dy - qy * g.pel) * g.pel + dx - qx * g.pel), std::int64_t(g.pad_x) + x + qx,
          std::int64_t(g.pad_y) + y + qy};
}
// A sufficient certificate only. Outside this rectangle, exact phase admission
// is still required; irregular external phases must never be rejected early.
class CommonDomain {
  std::int64_t width_, height_;
  int pad_x_, pad_y_, pel_;

public:
  explicit CommonDomain(const RenderPhaseGeometry& g)
      : width_(g.phases[0].width), height_(g.phases[0].height), pad_x_(g.pad_x), pad_y_(g.pad_y), pel_(g.pel) {
    for (int a = 1; a < g.pel * g.pel; ++a) {
      width_ = (std::min)(width_, std::int64_t(g.phases[a].width));
      height_ = (std::min)(height_, std::int64_t(g.phases[a].height));
    }
  }
  bool contains_scaled(int x, int y, std::int64_t dx256, std::int64_t dy256) const {
    const auto ax = (std::int64_t(pad_x_) + x) * pel_ * 256 + dx256;
    const auto ay = (std::int64_t(pad_y_) + y) * pel_ * 256 + dy256;
    return ax >= 0 && ay >= 0 && ax < width_ * pel_ * 256 && ay < height_ * pel_ * 256;
  }
};
} // namespace neo_mv::flow_coordinates
