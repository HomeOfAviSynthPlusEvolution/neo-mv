#pragma once

#include "core/interpolation/dense.hpp"
#include "core/interpolation/sampling.hpp"
#include "core/interpolation/blur.hpp"

namespace neo_mv {
template <class T>
struct ScalarInterpolationKernels {
  using Dense = DenseInterpolationPlan<>;
  using DenseFlow = DenseFlowPlan<>;
  using BlurAverage = ScalarBlurAverage;

  static void compose(const std::vector<InterpolationSamples<T>>& samples, const std::vector<std::uint8_t>& mF,
                      const std::vector<std::uint8_t>& mB, bool extra, int time, int bits, span2d::Plane<T> output) {
    validate_plane(output);
    const auto count = std::size_t(output.width()) * output.height();
    if (samples.size() != count || mF.size() != count || mB.size() != count)
      throw std::invalid_argument("interpolation composition size mismatch");
    for (int y = 0; y < output.height(); ++y)
      for (int x = 0; x < output.width(); ++x) {
        const auto i = std::size_t(y) * output.width() + x;
        const auto& s = samples[i];
        output.row(y)[x] = extra ? interpolation_extra(s.A, s.C, s.E, s.K, mF[i], mB[i], time, bits)
                                 : interpolation_basic(s.A, s.C, s.A0, s.C0, mF[i], mB[i], time, bits);
      }
  }
  static void blend(span2d::Plane<const T> a, span2d::Plane<const T> b, span2d::Plane<T> out, int time, int bits) {
    interpolation_blend_planes(a, b, out, time, bits);
  }
};
} // namespace neo_mv
