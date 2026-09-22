#pragma once

#include "core/flow/dense.hpp"
#include "core/render/block_sampling.hpp"

#include <cstring>

namespace neo_mv {

class FlowSamplingPlan {
  RenderPhaseGeometry geometry_;
  int width_, height_, time_;
  struct Location {
    int phase, x, y;
  };

  std::int64_t displacement(std::int16_t value) const {
    const auto scaled = std::int64_t(value) * time_ + 128;
    // Nonnegative division also avoids a compiler misoptimization of signed
    // remainder correction at exact negative multiples of 256. The int16
    // component and [0,256] time bounds make both numerators representable.
    return scaled >= 0 ? scaled / 256 : -((255 - scaled) / 256);
  }

protected:
  void validate_dense(const DenseFlowField& field) const {
    const auto count = std::uint64_t(width_) * height_;
    if (field.width != width_ || field.height != height_ || field.x.size() != count || field.y.size() != count)
      throw std::invalid_argument("inconsistent dense Flow field");
  }
  Location location(int x, int y, std::int16_t vx, std::int16_t vy) const {
    const auto& g = geometry_;
    const auto ax = std::int64_t(g.pel) * x + displacement(vx);
    const auto ay = std::int64_t(g.pel) * y + displacement(vy);
    const auto qx = sampling_detail::floor_div(ax, g.pel), qy = sampling_detail::floor_div(ay, g.pel);
    const int phase = static_cast<int>((ay - qy * g.pel) * g.pel + ax - qx * g.pel);
    const auto sx = std::int64_t(g.pad_x) + qx, sy = std::int64_t(g.pad_y) + qy;
    if (sx < 0 || sy < 0 || sx >= g.phases[phase].width || sy >= g.phases[phase].height)
      throw std::invalid_argument("Flow sample exceeds its logical phase domain");
    return {phase, static_cast<int>(sx), static_cast<int>(sy)};
  }

public:
  FlowSamplingPlan(RenderPhaseGeometry geometry, int width, int height, int time256)
      : geometry_(geometry), width_(width), height_(height), time_(time256) {
    const auto& g = geometry_;
    if (width <= 0 || height <= 0 || time256 < 0 || time256 > 256 || (g.pel != 1 && g.pel != 2 && g.pel != 4) ||
        (g.ratio_x != 1 && g.ratio_x != 2) || (g.ratio_y != 1 && g.ratio_y != 2) || g.pad_x < 0 || g.pad_y < 0)
      throw std::invalid_argument("invalid Flow sampling geometry or time");
    for (int a = 0; a < g.pel * g.pel; ++a)
      if (g.phases[a].width <= 0 || g.phases[a].height <= 0)
        throw std::invalid_argument("missing Flow logical phase domain");
    // Creation admits only zero-displacement integer-phase coordinates.
    if (std::int64_t(g.pad_x) + width > g.phases[0].width || std::int64_t(g.pad_y) + height > g.phases[0].height)
      throw std::invalid_argument("Flow zero displacement exceeds its logical phase domain");
  }
  const RenderPhaseGeometry& geometry() const { return geometry_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int time_coefficient() const { return time_; }

  void preflight(const DenseFlowField& field) const {
    validate_dense(field);
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        const auto i = std::size_t(y) * width_ + x;
        (void)location(x, y, field.x[i], field.y[i]);
      }
  }

protected:
  // Shared storage checks; callers admit every coordinate before reading pixels.
  template <class T>
  void validate_storage(const DenseFlowField& field, const SubpixelPhases<T>& source,
                        span2d::Plane<T> output) const {
    static_assert(supported_sample<T>);
    validate_plane(output);
    const auto& g = geometry_;
    if (output.width() != width_ || output.height() != height_ || source.pel != g.pel)
      throw std::invalid_argument("Flow sampling storage geometry mismatch");
    for (int a = 0; a < g.pel * g.pel; ++a) {
      const auto plane = source.planes[a];
      validate_plane(plane);
      if (plane.width() != g.phases[a].width || plane.height() != g.phases[a].height ||
          active_rows_overlap(plane, output))
        throw std::invalid_argument("Flow phase storage mismatch or output alias");
    }
    if (std::uint64_t(width_) > std::uint64_t(PTRDIFF_MAX) / sizeof(std::int16_t))
      throw std::overflow_error("dense Flow row stride is unrepresentable");
    for (const auto* component : {&field.x, &field.y}) {
      const auto view = checked_plane(component->data(), width_, height_, std::ptrdiff_t(width_) * sizeof(std::int16_t),
                                      component->size() * sizeof(std::int16_t));
      if (active_rows_overlap(view, output))
        throw std::invalid_argument("Flow output aliases dense input");
    }
  }
public:
  template <class T>
  void sample(const DenseFlowField& field, const SubpixelPhases<T>& source, span2d::Plane<T> output) const {
    preflight(field);
    validate_storage(field, source, output);
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        const auto i = std::size_t(y) * width_ + x;
        const auto at = location(x, y, field.x[i], field.y[i]);
        // Copy object representation: NaNs, infinities and signed zero are
        // image data, not numeric scores. No conversion or clamping applies.
        std::memcpy(output.row(y).data() + x, source.planes[at.phase].row(at.y).data() + at.x, sizeof(T));
      }
  }
};
} // namespace neo_mv
