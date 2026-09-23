#pragma once
#include "core/depan/analysis.hpp"
#include "core/depan/sampling.hpp"
#include "highway/depan_rows.hpp"
#include "highway/depan_sampling.hpp"
#include "highway/rows.hpp"
#include <cstring>

namespace neo_mv::depan {
struct HighwayResiduals {
  static std::array<float, 4> adjust(std::array<float, 4> values, const std::array<float, 4>& scales,
                                     const std::array<float, 4>& gradients, std::size_t count) {
    simd::depan_rows::adjust(values.data(), scales.data(), gradients.data(), count, values.data());
    return values;
  }
  struct Rows {
    std::vector<float> ex, ey;
    auto operator()(std::size_t i) const { return std::array<float, 2>{ex[i], ey[i]}; }
  };
  static Rows prepare(const Observations& observations, Transform map) {
    const auto count = observations.values.size();
    std::array<std::vector<float>, 4> rows;
    for (auto& r : rows)
      r.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      const auto& o = observations.values[i];
      rows[0][i] = analysis_detail::integer32(static_cast<std::uint64_t>(o.x));
      rows[1][i] = analysis_detail::integer32(static_cast<std::uint64_t>(o.y));
      rows[2][i] = o.dx;
      rows[3][i] = o.dy;
    }
    Rows result{std::vector<float>(count), std::vector<float>(count)};
    simd::depan_rows::residuals(rows[0].data(), rows[1].data(), rows[2].data(), rows[3].data(), count, map,
                                result.ex.data(), result.ey.data());
    return result;
  }
  static FitSums accumulate(const Observations& observations, const std::vector<float>& weights, Transform map,
                            bool zoom, bool rotation) {
    const auto rows = prepare(observations, map);
    return simd::depan_rows::accumulate(observations, weights, rows.ex.data(), rows.ey.data(), zoom, rotation);
  }
};
template <>
struct FitWorkspace<HighwayResiduals> {
  const Observations& observations;
  std::array<std::vector<float>, 6> rows;
  std::vector<FitGeometry> geometry;
  explicit FitWorkspace(const Observations& o) : observations(o), geometry(o.values.size()) {
    for (auto& row : rows)
      row.resize(o.values.size());
    for (std::size_t i = 0; i < o.values.size(); ++i) {
      const auto& v = o.values[i];
      const auto x = static_cast<std::uint64_t>(v.x), y = static_cast<std::uint64_t>(v.y);
      rows[0][i] = analysis_detail::integer32(x);
      rows[1][i] = analysis_detail::integer32(y);
      rows[2][i] = v.dx;
      rows[3][i] = v.dy;
      geometry[i] = {analysis_detail::square32(x), analysis_detail::square32(y), analysis_detail::integer32(2 * x),
                     analysis_detail::integer32(2 * y)};
    }
  }
  FitSums accumulate(const std::vector<float>& weights, Transform map, bool zoom, bool rotation) {
    simd::depan_rows::residuals(rows[0].data(), rows[1].data(), rows[2].data(), rows[3].data(),
                                observations.values.size(), map, rows[4].data(), rows[5].data());
    return simd::depan_rows::accumulate(observations, weights, rows[4].data(), rows[5].data(), zoom, rotation,
                                        geometry.data());
  }
};
class HighwaySamplingPlan : public SamplingPlan {
public:
  using SamplingPlan::SamplingPlan;
  template <class T>
  void validate(span2d::Plane<const T> source, span2d::Plane<T> output) const {
    validate_storage(source, output);
    for (int y = 0; y < height(); ++y)
      simd::detail::scan(source.row(y).data(), width(), (1u << bits()) - 1);
  }
  template <class T>
  void render(span2d::Plane<const T> source, span2d::Plane<T> output, bool preserve = false) const {
    validate(source, output);
    if (mode() == 1) {
      simd::depan_rows::linear_render(*this, source, output, preserve);
      return;
    }
    std::vector<SamplingCoordinates> coordinates(width());
    if (mode() == 0) {
      for (int y = 0; y < height(); ++y) {
        simd::depan_rows::coordinates(*this, y, coordinates.data());
        for (int x = 0; x < width(); ++x)
          write_sample(source, output.row(y)[x], coordinates[x], preserve);
      }
      return;
    }
    const auto count = static_cast<std::size_t>(width());
    constexpr int taps = 16;
    const bool translation = sampling_class() == SamplingClass::translation;
    std::vector<std::int64_t> samples, weights, results;
    if (count > samples.max_size() / static_cast<std::size_t>(taps))
      throw std::length_error("Depan SIMD tap storage is unrepresentable");
    samples.resize(count * taps);
    weights.resize(count * taps);
    results.resize(count);
    std::vector<int> columns;
    for (int y = 0; y < height(); ++y) {
      columns.clear();
      auto visit = [&](int x, SamplingCoordinates q) {
        const bool complete = q.i >= 1 && q.i < width() - 2 && q.j >= 1 && q.j < height() - 2;
        if (!complete) {
          write_sample(source, output.row(y)[x], q, preserve);
          return;
        }
        const auto index = columns.size();
        columns.push_back(x);
        const int ix = static_cast<int>(q.i), iy = static_cast<int>(q.j);
        const auto cx = cubic_coefficients(static_cast<int>(mul(256, q.fx))),
                   cy = cubic_coefficients(static_cast<int>(mul(256, q.fy)));
        for (int f = 0; f < 4; ++f)
          for (int e = 0; e < 4; ++e) {
            const auto pos = std::size_t(f * 4 + e) * count + index;
            samples[pos] = source.row(iy + f - 1)[ix + e - 1];
            weights[pos] = translation ? cx[e] * cy[f] / 2048 : cx[e] * cy[f];
          }
      };
      simd::depan_rows::coordinates(*this, y, coordinates.data());
      for (int x = 0; x < width(); ++x)
        visit(x, coordinates[x]);
      const auto used = columns.size();
      if (!used)
        continue;
      // The admitted footprints already occupy the start of each tap row.
      // Read with the original row spacing instead of compacting every row.
      simd::depan_rows::weighted(samples.data(), weights.data(), used, taps, translation ? 11 : 22, translation,
                                 (1 << bits()) - 1, results.data(), count);
      for (std::size_t i = 0; i < used; ++i)
        output.row(y)[columns[i]] = static_cast<T>(results[i]);
    }
  }
};
} // namespace neo_mv::depan
