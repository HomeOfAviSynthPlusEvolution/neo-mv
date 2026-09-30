#pragma once
#include "core/motion/analysis_field.hpp"

namespace neo_mv::staggered {
// Infinite triangular lattice with duplicated boundary vertices. Outside the
// finite center hull this extends edge values, never extrapolates motion.
struct Triangle {
  std::array<std::size_t, 3> index;
  std::array<double, 3> weight;
  template <class Read>
  double sample(Read read) const {
    return weight[0] * read(index[0]) + weight[1] * read(index[1]) + weight[2] * read(index[2]);
  }
};
inline Triangle triangle(const AnalysisMetadata& m, double x, double y) {
  const double row = std::clamp((y - m.block_height / 2.0) / (m.block_height * 0.75), 0.0, double(m.blocks_y - 1));
  const int a = int(std::floor(row)), b = std::min(a + 1, m.blocks_y - 1);
  const double t = row - a, d = (a & 1) ? -0.5 : 0.5;
  const double u = (x - m.block_width / 2.0) / m.block_width - (a & 1 ? 0.5 : 0.0) - d * t;
  const int i = int(std::floor(u));
  const double f = u - i;
  const auto at = [&](int col, int r) {
    return std::size_t(r) * m.blocks_x + std::clamp(col, 0, m.blocks_x - 1);
  };
  if (d > 0) {
    if (f + t <= 1)
      return {{at(i, a), at(i + 1, a), at(i, b)}, {1 - f - t, f, t}};
    return {{at(i + 1, a), at(i, b), at(i + 1, b)}, {1 - t, 1 - f, f + t - 1}};
  }
  if (f >= t)
    return {{at(i, a), at(i + 1, a), at(i + 1, b)}, {1 - f, f - t, t}};
  return {{at(i, a), at(i, b), at(i + 1, b)}, {1 - t, t - f, f}};
}
template <class T>
void resize(const AnalysisMetadata& m, span2d::Plane<const T> input, span2d::Plane<T> output, int ratio_x,
            int ratio_y) {
  for (int y = 0; y < output.height(); ++y)
    for (int x = 0; x < output.width(); ++x) {
      const auto p = triangle(m, (x + 0.5) * ratio_x, (y + 0.5) * ratio_y);
      const auto value =
          p.sample([&](std::size_t i) { return double(input.row(int(i / m.blocks_x))[i % m.blocks_x]); });
      output.row(y)[x] = static_cast<T>(std::lround(value));
    }
}
inline SpatialPredictors spatial(const MotionGrid& grid, int x, int y, int direction, int shift, MotionVector global,
                                 CandidateDomain domain) {
  const MotionTriple missing{{0, shift}, 0};
  const auto read = [&](int bx, int by) {
    auto v = bx < 0 || bx >= grid.width || by < 0 ? missing : grid.values[std::size_t(by) * grid.width + bx];
    v.vector = prediction_detail::clamp(v.vector, domain);
    return v;
  };
  SpatialPredictors result;
  result.p[1] = read(x - direction, y);
  result.p[2] = read(x - (y % 2 == 0), y - 1);
  result.p[3] = read(x + (y % 2 != 0), y - 1);
  const auto median = [](int a, int b, int c) {
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
  };
  result.p[0] = y == 0 ? result.p[1]
                       : MotionTriple{{median(result.p[1].vector.x, result.p[2].vector.x, result.p[3].vector.x),
                                       median(result.p[1].vector.y, result.p[2].vector.y, result.p[3].vector.y)},
                                      std::max({result.p[1].error, result.p[2].error, result.p[3].error})};
  result.global = prediction_detail::clamp(global, domain);
  return result;
}
// Pointy-top hexagon: (W/2,0), (W,H/4), (W,3H/4),
// (W/2,H), (0,3H/4), (0,H/4), evaluated at pixel centers.
inline bool inside(int x, int y, int width, int height) {
  const int dx = std::abs(2 * x + 1 - width), dy = std::abs(2 * y + 1 - height);
  return dx * height + 2 * dy * width <= 2 * width * height;
}
template <class T>
std::int64_t hex_sad(span2d::Plane<const T> source, span2d::Plane<const T> reference) {
  using A = std::conditional_t<std::is_same_v<T, float>, float, std::int64_t>;
  A total = 0;
  int count = 0;
  for (int y = 0; y < source.height(); ++y)
    for (int x = 0; x < source.width(); ++x)
      if (inside(x, y, source.width(), source.height())) {
        const A difference = metric_detail::subtract(metric_detail::finite(A(source.row(y)[x])),
                                                     metric_detail::finite(A(reference.row(y)[x])));
        total = metric_detail::accumulate(total, metric_detail::magnitude(difference));
        ++count;
      }
  const int area = source.width() * source.height();
  if constexpr (std::is_same_v<T, float>)
    return encode_float_error(total * (float(area) / count));
  else
    return (total * area + count / 2) / count;
}
template <class T>
BlockError error(const SamplingGeometry& g, BlockRegion b, const SamplingFrames<T>& frames, MotionVector v, bool hex) {
  std::array<std::int64_t, 3> sums{};
  for (int k = 0; k < (g.chroma ? 3 : 1); ++k) {
    const auto a = source_block(g, b, frames, k), r = reference_block(g, b, frames, k, v);
    sums[k] = hex ? hex_sad(a, r) : block_metric<T, true>(a, r, BlockMetric::sad);
  }
  const auto chroma = metric_detail::accumulate(sums[1], sums[2]);
  return {sums[0], chroma, metric_detail::accumulate(sums[0], chroma)};
}
} // namespace neo_mv::staggered
