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
    const int taps = mode() == 2 ? 16 : 4;
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
        const bool complete = mode() == 1 ? (q.i >= 0 && q.i < width() - 1 && q.j >= 0 && q.j < height() - 1)
                                          : (q.i >= 1 && q.i < width() - 2 && q.j >= 1 && q.j < height() - 2);
        if (mode() == 0 || !complete) {
          write_sample(source, output.row(y)[x], q, preserve);
          return;
        }
        const auto index = columns.size();
        columns.push_back(x);
        const int ix = static_cast<int>(q.i), iy = static_cast<int>(q.j);
        if (mode() == 1) {
          const int ax = static_cast<int>(mul(32, q.fx)), ay = static_cast<int>(mul(32, q.fy));
          for (int f = 0; f < 2; ++f)
            for (int e = 0; e < 2; ++e) {
              const auto pos = std::size_t(f * 2 + e) * count + index;
              samples[pos] = source.row(iy + f)[ix + e];
              weights[pos] = (e ? ax : 32 - ax) * (f ? ay : 32 - ay);
            }
        } else {
          const auto cx = cubic_coefficients(static_cast<int>(mul(256, q.fx))),
                     cy = cubic_coefficients(static_cast<int>(mul(256, q.fy)));
          for (int f = 0; f < 4; ++f)
            for (int e = 0; e < 4; ++e) {
              const auto pos = std::size_t(f * 4 + e) * count + index;
              samples[pos] = source.row(iy + f - 1)[ix + e - 1];
              weights[pos] = translation ? cx[e] * cy[f] / 2048 : cx[e] * cy[f];
            }
        }
      };
      simd::depan_rows::coordinates(*this, y, coordinates.data());
      for (int x = 0; x < width(); ++x)
        visit(x, coordinates[x]);
      // Compact the per-tap arrays to the number of admitted footprints.
      const auto used = columns.size();
      if (!used)
        continue;
      for (int k = 1; k < taps; ++k) {
        std::memmove(samples.data() + std::size_t(k) * used, samples.data() + std::size_t(k) * count,
                     used * sizeof(std::int64_t));
        std::memmove(weights.data() + std::size_t(k) * used, weights.data() + std::size_t(k) * count,
                     used * sizeof(std::int64_t));
      }
      simd::depan_rows::weighted(samples.data(), weights.data(), used, taps,
                                 mode() == 1   ? 10
                                 : translation ? 11
                                               : 22,
                                 mode() == 2 && translation, (1 << bits()) - 1, results.data());
      for (std::size_t i = 0; i < used; ++i)
        output.row(y)[columns[i]] = static_cast<T>(results[i]);
    }
  }
};
} // namespace neo_mv::depan
