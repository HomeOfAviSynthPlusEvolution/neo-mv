#pragma once

#include "core/motion/block_sampling.hpp"
#include "core/super/geometry.hpp"

#include <map>

namespace neo_mv {

struct MotionTriple {
  MotionVector vector{};
  std::int64_t error = 0;
};
// All entries initially contain parent predictions (or zero at the coarsest
// level). The analysis caller replaces each entry with its final result in
// scan order, so spatial queries see defined initial/final values only.
struct MotionGrid {
  std::int32_t width, height;
  std::vector<MotionTriple> values;
};
struct PredictionGeometry {
  std::int32_t block_width, block_height, overlap_x, overlap_y;
  std::int32_t parent_pel, child_pel;
};
struct SpatialPredictors {
  std::array<MotionTriple, 4> p;
  MotionVector global;
};

namespace prediction_detail {
inline std::int64_t add(std::int64_t a, std::int64_t b) {
  if ((b > 0 && a > INT64_MAX - b) || (b < 0 && a < INT64_MIN - b))
    throw std::overflow_error("predictor sum exceeds int64 range");
  return a + b;
}
// All weights are nonnegative, including when one overlap is zero.
inline std::int64_t weight(std::int64_t value, std::int64_t factor) {
  if (factor != 0 && (value > INT64_MAX / factor || value < INT64_MIN / factor))
    throw std::overflow_error("predictor product exceeds int64 range");
  return value * factor;
}
inline std::int32_t coordinate(std::int64_t value) {
  if (value < INT32_MIN || value > INT32_MAX)
    throw std::overflow_error("predictor coordinate exceeds int32 range");
  return static_cast<std::int32_t>(value);
}
inline int log_pel(int pel) {
  if (pel != 1 && pel != 2 && pel != 4)
    throw std::invalid_argument("invalid predictor pel");
  return pel == 1 ? 0 : pel == 2 ? 1 : 2;
}
inline void validate(const MotionGrid& grid) {
  if (grid.width <= 0 || grid.height <= 0 || std::uint64_t(grid.width) * grid.height != grid.values.size())
    throw std::invalid_argument("invalid motion grid dimensions or element count");
}
inline MotionTriple at(const MotionGrid& grid, int x, int y) {
  const auto value = grid.values[std::size_t(y) * grid.width + x];
  if (value.error < 0)
    throw std::invalid_argument("negative predictor confidence error");
  return value;
}
inline std::int64_t truncate(double value) {
  if (!std::isfinite(value) || value < -0x1p63 || value >= 0x1p63)
    throw std::overflow_error("floating predictor numerator exceeds int64 range");
  return static_cast<std::int64_t>(value);
}
inline MotionVector clamp(MotionVector v, CandidateDomain omega) {
  return {static_cast<std::int32_t>(std::clamp(std::int64_t(v.x), omega.left, omega.right - 1)),
          static_cast<std::int32_t>(std::clamp(std::int64_t(v.y), omega.top, omega.bottom - 1))};
}
} // namespace prediction_detail

// Returns a value, so a query cannot overwrite or alias its parent grid.
inline MotionTriple interpolate_predictor(const MotionGrid& parent, std::int32_t child_x, std::int32_t child_y,
                                          PredictionGeometry g) {
  using namespace prediction_detail;
  validate(parent);
  const int r = 3 - log_pel(g.child_pel) + log_pel(g.parent_pel);
  if (!geometry_detail::block_pair(g.block_width, g.block_height) || g.overlap_x < 0 || g.overlap_y < 0 ||
      g.overlap_x > g.block_width / 2 || g.overlap_y > g.block_height / 2)
    throw std::invalid_argument("invalid parent interpolation geometry");
  const auto xmax = 2 * std::int64_t(parent.width) - 1, ymax = 2 * std::int64_t(parent.height) - 1;
  const auto i = std::clamp(std::int64_t(child_x), std::int64_t{0}, xmax);
  const auto t = std::clamp(std::int64_t(child_y), std::int64_t{0}, ymax);
  const int x = static_cast<int>(i / 2), y = static_cast<int>(t / 2);
  const int dx = 2 * int(i % 2) - 1, dy = 2 * int(t % 2) - 1;
  const bool edge_x = i == 0 || i == xmax, edge_y = t == 0 || t == ymax;
  const auto a = at(parent, x, y);
  std::array<MotionTriple, 4> values;
  if (edge_x && edge_y)
    values = {a, a, a, a};
  else if (edge_x) {
    const auto b = at(parent, x, y + dy);
    values = {a, a, b, b};
  } else if (edge_y) {
    const auto b = at(parent, x + dx, y);
    values = {a, a, b, b};
  } else
    values = {a, at(parent, x + dx, y), at(parent, x, y + dy), at(parent, x + dx, y + dy)};
  std::array<std::int64_t, 4> weights{9, 3, 3, 1};
  const bool overlap = g.overlap_x != 0 || g.overlap_y != 0;
  double reciprocal = 1.0;
  if (overlap) {
    const std::int64_t sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
    const std::int64_t ax = 3 * g.block_width - (dx > 0 ? 2 : 4) * g.overlap_x;
    const std::int64_t ay = 3 * g.block_height - (dy > 0 ? 2 : 4) * g.overlap_y;
    const auto bx = 4 * sx - ax, by = 4 * sy - ay;
    weights = {ax * ay, bx * ay, ax * by, bx * by};
    reciprocal = 1.0 / double(sx * sy);
  }
  const auto numerator = [&](int component) {
    std::int64_t sum = 0;
    for (int n = 0; n < 4; ++n) {
      const auto value = component == 0   ? std::int64_t(values[n].vector.x)
                         : component == 1 ? std::int64_t(values[n].vector.y)
                                          : values[n].error;
      sum = add(sum, weight(value, weights[n]));
    }
    if (overlap)
      return truncate(double(sum) * reciprocal);
    return component == 2 ? add(sum, 8) : sum;
  };
  // Supported pel ratios make r range from 1 through 5.
  return {{coordinate(sampling_detail::floor_div(numerator(0), 1 << r)),
           coordinate(sampling_detail::floor_div(numerator(1), 1 << r))},
          numerator(2) / 16};
}

inline MotionVector global_predictor(const MotionGrid& parent, bool enabled = true) {
  using namespace prediction_detail;
  if (!enabled)
    return {0, 0};
  validate(parent);
  std::map<std::int32_t, std::size_t> xs, ys;
  for (const auto& value : parent.values) {
    ++xs[value.vector.x];
    ++ys[value.vector.y];
  }
  const auto mode = [](const auto& counts) {
    std::int32_t selected = 0;
    std::size_t frequency = 0;
    for (auto item : counts)
      if (item.second > frequency) {
        selected = item.first;
        frequency = item.second;
      }
    return selected;
  };
  const auto mx = mode(xs), my = mode(ys);
  std::int64_t x = 0, y = 0, count = 0;
  for (const auto& value : parent.values) {
    const auto dx = std::int64_t(value.vector.x) - mx, dy = std::int64_t(value.vector.y) - my;
    if (dx > -6 && dx < 6 && dy > -6 && dy < 6) {
      x = add(x, value.vector.x);
      y = add(y, value.vector.y);
      count = add(count, 1);
    }
  }
  if (count == 0)
    return {coordinate(2 * std::int64_t(mx)), coordinate(2 * std::int64_t(my))};
  return {coordinate(weight(x, 2) / count), coordinate(weight(y, 2) / count)};
}

inline MotionVector enter_global_level(MotionVector global, int pel, int field_shift) {
  prediction_detail::log_pel(pel);
  return {prediction_detail::coordinate(std::int64_t(global.x) * pel),
          prediction_detail::coordinate(std::int64_t(global.y) * pel + field_shift)};
}

inline SpatialPredictors spatial_predictors(const MotionGrid& grid, int bx, int by, int direction, int field_shift,
                                            MotionVector layer_global, CandidateDomain omega) {
  using namespace prediction_detail;
  validate(grid);
  if (bx < 0 || bx >= grid.width || by < 0 || by >= grid.height || (direction != 1 && direction != -1) ||
      omega.left < INT32_MIN || omega.top < INT32_MIN || omega.right > std::int64_t(INT32_MAX) + 1 ||
      omega.bottom > std::int64_t(INT32_MAX) + 1 || omega.left >= omega.right || omega.top >= omega.bottom)
    throw std::invalid_argument("invalid spatial predictor query");
  const MotionTriple missing{{0, field_shift}, 0};
  const auto present = [&](int x, int y) {
    return x >= 0 && x < grid.width && y >= 0 && y < grid.height;
  };
  SpatialPredictors result;
  result.p[1] = present(bx - direction, by) ? at(grid, bx - direction, by) : missing;
  result.p[2] = present(bx, by - 1) ? at(grid, bx, by - 1) : missing;
  result.p[3] = present(bx + direction, by + 1)   ? at(grid, bx + direction, by + 1)
                : present(bx + direction, by - 1) ? at(grid, bx + direction, by - 1)
                                                  : missing;
  for (int i = 1; i < 4; ++i)
    result.p[i].vector = clamp(result.p[i].vector, omega);
  result.p[0] = result.p[1];
  if (by > 0) {
    std::array<std::int32_t, 3> xs{}, ys{};
    for (int i = 0; i < 3; ++i) {
      xs[i] = result.p[i + 1].vector.x;
      ys[i] = result.p[i + 1].vector.y;
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    result.p[0] = {{xs[1], ys[1]}, std::max({result.p[1].error, result.p[2].error, result.p[3].error})};
  }
  result.global = clamp(layer_global, omega);
  return result;
}

} // namespace neo_mv
