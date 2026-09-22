#pragma once

#include "highway/grid_resampling.hpp"
#include "highway/interpolation_rows.hpp"
#include "highway/scene.hpp"
#include "highway/rows.hpp"
#include "highway/interpolation_sampling.hpp"
#include "kernels/interpolation_scalar.hpp"

namespace neo_mv {
struct HighwayBlurAverage {
  template <class T>
  void operator()(const T* samples, std::size_t count, int bits, T* output) const {
    mask_detail::validate_storage<T>(bits);
    if (!samples || !output || count < 1 || count > 65537)
      throw std::invalid_argument("invalid blur sample list");
    if (count == 1) {
      std::memmove(output, samples, sizeof(T));
      return;
    }
    if constexpr (std::is_same_v<T, float>) {
      // The float contract is an ordered binary64 accumulation. Reassociation
      // into a SIMD tree would change it, so retain that exact sequence.
      ordered_blur_average(samples, count, bits, output);
    } else {
      const auto maximum = (std::uint32_t{1} << bits) - 1;
      simd::detail::scan(samples, static_cast<int>(count), maximum);
      *output = static_cast<T>(simd::interpolation_rows::sum(samples, count) / count);
    }
  }
};

template <class T>
struct HighwayInterpolationKernels {
  static constexpr auto scene_count = &simd::scene_count;
  using Sampling = simd::InterpolationSamplingPlan;
  using Dense = DenseInterpolationPlan<simd::GridResamplingPlan>;
  using DenseFlow = DenseFlowPlan<simd::GridResamplingPlan>;
  using BlurAverage = HighwayBlurAverage;
  using Lane = std::conditional_t<std::is_same_v<T, float>, float, std::uint32_t>;

  static void compose(const std::vector<InterpolationSamples<T>>& samples, const std::vector<std::uint8_t>& mF,
                      const std::vector<std::uint8_t>& mB, bool extra, int time, int bits, span2d::Plane<T> output) {
    interpolation_detail::controls(time);
    mask_detail::validate_storage<T>(bits);
    validate_plane(output);
    const auto count = std::size_t(output.width()) * output.height();
    if (samples.size() != count || mF.size() != count || mB.size() != count)
      throw std::invalid_argument("interpolation composition size mismatch");
    std::array<std::vector<Lane>, 7> rows;
    for (auto& row : rows)
      row.resize(output.width());
    for (int y = 0; y < output.height(); ++y) {
      for (int x = 0; x < output.width(); ++x) {
        const auto i = std::size_t(y) * output.width() + x;
        const auto& s = samples[i];
        const T values[] = {s.A, s.C, extra ? s.E : s.A0, extra ? s.K : s.C0};
        for (int k = 0; k < 4; ++k) {
          interpolation_detail::sample(values[k], bits);
          rows[k][x] = values[k];
        }
        rows[4][x] = mF[i];
        rows[5][x] = mB[i];
      }
      simd::interpolation_rows::compose(rows[0].data(), rows[1].data(), rows[2].data(), rows[3].data(), rows[4].data(),
                                        rows[5].data(), output.width(), extra, time, rows[6].data());
      for (int x = 0; x < output.width(); ++x)
        output.row(y)[x] = static_cast<T>(rows[6][x]);
    }
  }
  static void blend(span2d::Plane<const T> a, span2d::Plane<const T> b, span2d::Plane<T> out, int time, int bits) {
    interpolation_detail::controls(time);
    mask_detail::validate_storage<T>(bits);
    validate_plane(a);
    validate_plane(b);
    validate_plane(out);
    if (a.width() != out.width() || b.width() != out.width() || a.height() != out.height() ||
        b.height() != out.height() || active_rows_overlap(a, out) || active_rows_overlap(b, out))
      throw std::invalid_argument("fallback blend storage mismatch or output alias");
    super_detail::PlaneBuffer<T> result(out.width(), out.height());
    std::array<std::vector<Lane>, 3> rows;
    for (auto& row : rows)
      row.resize(out.width());
    for (int y = 0; y < out.height(); ++y) {
      for (int x = 0; x < out.width(); ++x) {
        interpolation_detail::sample(a.row(y)[x], bits);
        interpolation_detail::sample(b.row(y)[x], bits);
        rows[0][x] = a.row(y)[x];
        rows[1][x] = b.row(y)[x];
      }
      simd::interpolation_rows::blend(rows[0].data(), rows[1].data(), out.width(), time, rows[2].data());
      for (int x = 0; x < out.width(); ++x)
        result.view().row(y)[x] = static_cast<T>(rows[2][x]);
    }
    for (int y = 0; y < out.height(); ++y)
      std::memcpy(out.row(y).data(), result.view().row(y).data(), std::size_t(out.width()) * sizeof(T));
  }
};
} // namespace neo_mv
