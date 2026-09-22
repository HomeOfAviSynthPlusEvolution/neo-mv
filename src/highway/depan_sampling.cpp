#include "highway/depan_sampling.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/depan_sampling.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::depan_rows {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class D, class V>
V FiniteCoordinate(D d, V v) {
  if (!hn::AllTrue(d, hn::IsFinite(v)))
    throw std::invalid_argument("non-finite Depan sampling coordinate");
  return v;
}
void Coordinates(const depan::SamplingPlan& plan, int y, depan::SamplingCoordinates* output) {
  if (y < 0 || y >= plan.height())
    throw std::invalid_argument("Depan sampling row is outside output");
  const auto kind = plan.sampling_class();
  if (kind == depan::SamplingClass::translation) {
    // The horizontal origin is rounded once, then incremented as binary64
    // integers. Forming float coordinates per pixel would change this rule.
    plan.row_coordinates(y, [&](int x, depan::SamplingCoordinates q) { output[x] = q; });
    return;
  }
  const hn::ScalableTag<float> d;
  const hn::Rebind<std::int32_t, decltype(d)> di;
  const int lanes = static_cast<int>(hn::Lanes(d));
  HWY_ALIGN std::int32_t xs[hn::MaxLanes(di)]{};
  HWY_ALIGN float px[hn::MaxLanes(d)]{}, py[hn::MaxLanes(d)]{};
  HWY_ALIGN float ix[hn::MaxLanes(d)], iy[hn::MaxLanes(d)], fx[hn::MaxLanes(d)], fy[hn::MaxLanes(d)];
  const auto m = plan.map();
  const auto zero = hn::Zero(d), half = hn::Set(d, 0.5f);
  const float row = depan::f32(y);
  const float vertical = depan::mul(m.h, row);
  const float shear = kind == depan::SamplingClass::affine ? depan::mul(m.v, row) : 0;
  const bool sequential = kind == depan::SamplingClass::affine && plan.mode() != 2;
  float next_x = sequential ? depan::add(m.tx, shear) : 0;
  float next_y = sequential ? depan::add(m.ty, vertical) : 0;
  for (int x = 0; x < plan.width();) {
    const int used = std::min(lanes, plan.width() - x);
    auto X = zero, Y = zero;
    if (sequential) {
      // Preserve the recurrence across vector groups; only splitting is SIMD.
      for (int i = 0; i < used; ++i) {
        px[i] = next_x;
        py[i] = next_y;
        if (x + i + 1 < plan.width()) {
          next_x = depan::add(next_x, m.u);
          next_y = depan::add(next_y, m.w);
        }
      }
      X = hn::Load(d, px);
      Y = hn::Load(d, py);
    } else {
      for (int i = 0; i < used; ++i)
        xs[i] = x + i;
      const auto column = hn::ConvertTo(d, hn::Load(di, xs));
      X = FiniteCoordinate(d, hn::Mul(hn::Set(d, m.u), column));
      X = FiniteCoordinate(d, hn::Add(hn::Set(d, m.tx), X));
      if (kind == depan::SamplingClass::scale) {
        Y = hn::Set(d, depan::add(m.ty, vertical));
      } else {
        X = FiniteCoordinate(d, hn::Add(X, hn::Set(d, shear)));
        Y = FiniteCoordinate(d, hn::Mul(hn::Set(d, m.w), column));
        Y = FiniteCoordinate(d, hn::Add(hn::Set(d, m.ty), Y));
        Y = FiniteCoordinate(d, hn::Add(Y, hn::Set(d, vertical)));
      }
    }
    auto I = zero, J = zero, FX = zero, FY = zero;
    if (plan.mode() == 0) {
      X = FiniteCoordinate(d, hn::Add(X, half));
      Y = FiniteCoordinate(d, hn::Add(Y, half));
      I = sequential ? hn::Trunc(X) : hn::Floor(X);
      J = sequential ? hn::Trunc(Y) : hn::Floor(Y);
    } else {
      I = hn::Floor(X);
      J = hn::Floor(Y);
      FX = hn::Sub(X, I);
      FY = hn::Sub(Y, J);
    }
    hn::Store(I, d, ix);
    hn::Store(J, d, iy);
    hn::Store(FX, d, fx);
    hn::Store(FY, d, fy);
    for (int i = 0; i < used; ++i)
      output[x + i] = {double(ix[i]), double(iy[i]), fx[i], fy[i]};
    x += used;
  }
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::depan_rows
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::depan_rows {
HWY_EXPORT(Coordinates);
void coordinates(const depan::SamplingPlan& plan, int y, depan::SamplingCoordinates* output) {
  HWY_DYNAMIC_DISPATCH(Coordinates)(plan, y, output);
}
} // namespace neo_mv::simd::depan_rows
#endif
