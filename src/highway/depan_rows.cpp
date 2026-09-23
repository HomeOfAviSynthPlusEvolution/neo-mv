#include "highway/depan_rows.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/depan_rows.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::depan_rows {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class D, class V>
auto Check(D d, V value) {
  if (!hn::AllTrue(d, hn::IsFinite(value)))
    throw std::invalid_argument("non-finite Depan SIMD residual");
  return value;
}
template <class D, class V>
auto Add(D d, V a, V b) {
  return Check(d, hn::Add(a, b));
}
template <class D, class V>
auto Sub(D d, V a, V b) {
  return Check(d, hn::Sub(a, b));
}
template <class D, class V>
auto Mul(D d, V a, V b) {
  return Check(d, hn::Mul(a, b));
}
template <class D, class V>
auto MulAdd(D d, V a, V b, V c) {
  const auto product = Mul(d, a, b);
#if HWY_NATIVE_FMA
  (void)product; // Keep the separate product overflow check even when the sum can cancel it.
  return Check(d, hn::MulAdd(a, b, c));
#else
  return Add(d, product, c);
#endif
}
bool NativeFma() {
  return HWY_NATIVE_FMA != 0;
}
bool NativeArithmeticEnvironment() {
#if HWY_ARCH_X86 && HWY_TARGET != HWY_SCALAR && HWY_TARGET != HWY_EMU128
  return (_mm_getcsr() & 0xffc0u) == 0x1f80u;
#else
  return false;
#endif
}

template <class D>
auto NeighborDifference(D d, const float* center, std::size_t stride) {
  auto total = hn::LoadU(d, center - stride - 1);
  total = hn::Add(total, hn::LoadU(d, center - stride));
  total = hn::Add(total, hn::LoadU(d, center - stride + 1));
  total = hn::Add(total, hn::LoadU(d, center - 1));
  total = hn::Add(total, hn::LoadU(d, center + 1));
  total = hn::Add(total, hn::LoadU(d, center + stride - 1));
  total = hn::Add(total, hn::LoadU(d, center + stride));
  total = hn::Add(total, hn::LoadU(d, center + stride + 1));
  return hn::Sub(hn::LoadU(d, center), hn::Mul(total, hn::Set(d, 0.125f)));
}
template <class D>
bool AdmissionChunk(D d, const float* dx, const float* dy, std::size_t stride, float wrong, std::int8_t* eligibility) {
  const auto x = NeighborDifference(d, dx, stride), y = NeighborDifference(d, dy, stride);
  // Nonfinite intermediates cannot become finite through these additions,
  // power-of-two scaling and subtraction; preserve lazy errors via fallback.
  if (!hn::AllTrue(d, hn::And(hn::IsFinite(x), hn::IsFinite(y))))
    return false;
  const auto limit = hn::Set(d, wrong);
  const auto accepted = hn::And(hn::Le(hn::Abs(x), limit), hn::Le(hn::Abs(y), limit));
  HWY_ALIGN float flags[hn::MaxLanes(d)];
  hn::Store(hn::IfThenElse(accepted, hn::Set(d, 1), hn::Zero(d)), d, flags);
  for (std::size_t i = 0; i < hn::Lanes(d); ++i)
    eligibility[i] &= static_cast<std::int8_t>(flags[i]);
  return true;
}
bool WeightAdmission(const depan::Observations& observations, const float* dx, const float* dy, float wrong,
                     std::int8_t* eligibility) {
  if (!NativeArithmeticEnvironment() || !std::isfinite(wrong))
    return false;
  const int nx = observations.nx, ny = observations.ny;
  const int border = observations.masked ? 0 : 4;
  for (int y = 0; y < ny; ++y)
    for (int x = 0; x < nx; ++x) {
      const auto i = std::size_t(y) * nx + x;
      eligibility[i] = x >= border && x < nx - border && y >= border && y < ny - border &&
                       observations.values[i].sad <= observations.sad_threshold;
    }
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  const hn::CappedTag<float, 1> one;
  for (int y = 1; y + 1 < ny; ++y) {
    std::size_t x = 1;
    for (; x + lanes < static_cast<std::size_t>(nx); x += lanes) {
      const auto i = std::size_t(y) * nx + x;
      if (!AdmissionChunk(d, dx + i, dy + i, nx, wrong, eligibility + i))
        return false;
    }
    for (; x + 1 < static_cast<std::size_t>(nx); ++x) {
      const auto i = std::size_t(y) * nx + x;
      if (!AdmissionChunk(one, dx + i, dy + i, nx, wrong, eligibility + i))
        return false;
    }
  }
  return true;
}

depan::FitSums AccumulateComponents(const std::vector<float>& weights, const float* ex, const float* ey, bool zoom,
                                    bool rotation, const depan::FitGeometry* geometry) {
  // Lanes hold independent statistics, not partial observation sums. Each
  // statistic sees the original observation order with the same rounding.
  const hn::CappedTag<float, 4> d;
  const hn::CappedTag<float, 1> one;
  auto a = hn::Set(d, 0.1f), b = hn::Zero(d), c = hn::Zero(d);
  HWY_ALIGN float factors[4], values[4], inner[4], terms_a[4], terms_b[4], terms_c[4];
  for (std::size_t i = 0; i < weights.size(); ++i) {
    const float x = ex[i], y = ey[i];
    values[0] = values[2] = x;
    values[1] = values[3] = y;
    factors[0] = x;
    factors[1] = y;
    factors[2] = factors[3] = 2;
    hn::Store(Mul(d, hn::Load(d, values), hn::Load(d, factors)), d, inner);
    terms_a[0] = geometry[i].x2;
    terms_a[1] = geometry[i].y2;
    terms_a[2] = hn::GetLane(Add(one, hn::Set(one, inner[0]), hn::Set(one, inner[1])));
    terms_a[3] = 1;
    terms_b[0] = inner[2];
    terms_b[1] = inner[3];
    terms_b[2] = terms_b[3] = 0;
    if (zoom || rotation) {
      factors[0] = zoom ? geometry[i].twice_x : 0;
      factors[1] = zoom ? geometry[i].twice_y : 0;
      factors[2] = rotation ? geometry[i].twice_y : 0;
      factors[3] = rotation ? geometry[i].twice_x : 0;
      hn::Store(Mul(d, hn::Load(d, values), hn::Load(d, factors)), d, inner);
      terms_b[2] = inner[0];
      terms_b[3] = inner[1];
      terms_c[0] = inner[2];
      terms_c[1] = inner[3];
      terms_c[2] = terms_c[3] = 0;
    }
    const auto weight = hn::Set(d, weights[i]);
    a = MulAdd(d, hn::Load(d, terms_a), weight, a);
    b = MulAdd(d, hn::Load(d, terms_b), weight, b);
    if (rotation)
      c = MulAdd(d, hn::Load(d, terms_c), weight, c);
  }
  hn::Store(a, d, terms_a);
  hn::Store(b, d, terms_b);
  hn::Store(c, d, terms_c);
  return {terms_a[3], terms_a[0], terms_a[1], terms_a[2], terms_b[0],
          terms_b[1], terms_b[2], terms_b[3], terms_c[0], terms_c[1]};
}

struct FitArithmetic {
  static float multiply_add(float a, float b, float c) {
    const hn::CappedTag<float, 1> d;
    // One lane preserves the serial reduction order on every target.
    return hn::GetLane(MulAdd(d, hn::Set(d, a), hn::Set(d, b), hn::Set(d, c)));
  }
};
depan::FitSums Accumulate(const depan::Observations& observations, const std::vector<float>& weights, const float* ex,
                          const float* ey, bool zoom, bool rotation, const depan::FitGeometry* geometry) {
  if (geometry && NativeArithmeticEnvironment())
    return AccumulateComponents(weights, ex, ey, zoom, rotation, geometry);
  const depan::ArithmeticContext arithmetic;
  return depan::accumulate_fit<FitArithmetic>(
      observations, weights, [ex, ey](std::size_t i) { return std::array<float, 2>{ex[i], ey[i]}; }, zoom, rotation,
      &arithmetic, geometry);
}
template <class D>
void AdjustChunk(D d, const float* values, const float* scales, const float* gradients, float* output) {
  const auto value = Check(d, hn::LoadU(d, values));
  const auto scale = Check(d, hn::LoadU(d, scales));
  const auto gradient = Check(d, hn::LoadU(d, gradients));
  const auto product = Mul(d, scale, gradient);
#if HWY_NATIVE_FMA
  (void)product;
  hn::StoreU(Check(d, hn::NegMulAdd(scale, gradient, value)), d, output);
#else
  hn::StoreU(Sub(d, value, product), d, output);
#endif
}
void Adjust(const float* values, const float* scales, const float* gradients, std::size_t count, float* output) {
  const hn::CappedTag<float, 4> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; i + lanes <= count; i += lanes)
    AdjustChunk(d, values + i, scales + i, gradients + i, output + i);
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    AdjustChunk(one, values + i, scales + i, gradients + i, output + i);
}
template <class D>
void ResidualChunk(D d, const float* x, const float* y, const float* dx, const float* dy, depan::Transform t, float* ex,
                   float* ey) {
  const auto X = hn::LoadU(d, x), Y = hn::LoadU(d, y);
  const auto a = Sub(d, Sub(d, MulAdd(d, hn::Set(d, t.v), Y, MulAdd(d, hn::Set(d, t.u), X, hn::Set(d, t.tx))), X),
                     hn::LoadU(d, dx));
  const auto b = Sub(d, Sub(d, MulAdd(d, hn::Set(d, t.h), Y, MulAdd(d, hn::Set(d, t.w), X, hn::Set(d, t.ty))), Y),
                     hn::LoadU(d, dy));
  hn::StoreU(a, d, ex);
  hn::StoreU(b, d, ey);
}
void Residuals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count, depan::Transform t,
               float* ex, float* ey) {
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; i + lanes <= count; i += lanes)
    ResidualChunk(d, x + i, y + i, dx + i, dy + i, t, ex + i, ey + i);
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    ResidualChunk(one, x + i, y + i, dx + i, dy + i, t, ex + i, ey + i);
}
template <class D>
bool StrictResidualChunk(D d, const float* x, const float* y, const float* dx, const float* dy, depan::Transform t,
                         float* ex, float* ey) {
  const auto X = hn::LoadU(d, x), Y = hn::LoadU(d, y);
  // These operations must not contract: weight admission uses separate
  // rounding even on native-FMA targets (-ffp-contract=off / fp:strict).
  const auto a =
      hn::Sub(hn::Sub(hn::Add(hn::Add(hn::Set(d, t.tx), hn::Mul(hn::Set(d, t.u), X)), hn::Mul(hn::Set(d, t.v), Y)), X),
              hn::LoadU(d, dx));
  const auto b =
      hn::Sub(hn::Sub(hn::Add(hn::Add(hn::Set(d, t.ty), hn::Mul(hn::Set(d, t.w), X)), hn::Mul(hn::Set(d, t.h), Y)), Y),
              hn::LoadU(d, dy));
  // With only additions/subtractions after each product, a nonfinite
  // intermediate cannot become finite again. Check the batch once at the end.
  if (!hn::AllTrue(d, hn::And(hn::IsFinite(a), hn::IsFinite(b))))
    return false;
  hn::StoreU(a, d, ex);
  hn::StoreU(b, d, ey);
  return true;
}
bool StrictResiduals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count,
                     depan::Transform t, float* ex, float* ey) {
#if HWY_ARCH_X86 && HWY_TARGET != HWY_SCALAR && HWY_TARGET != HWY_EMU128
  // Nearest rounding, gradual underflow, and masked FP exceptions are needed
  // for speculative evaluation of blocks that scalar admission may skip.
  if ((_mm_getcsr() & 0xffc0u) != 0x1f80u)
    return false;
#else
  // Until a target-specific environment check is available, preserve the
  // scalar semantics rather than assume the host's underflow mode.
  return false;
#endif
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; i + lanes <= count; i += lanes)
    if (!StrictResidualChunk(d, x + i, y + i, dx + i, dy + i, t, ex + i, ey + i))
      return false;
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    if (!StrictResidualChunk(one, x + i, y + i, dx + i, dy + i, t, ex + i, ey + i))
      return false;
  return true;
}
template <class D>
void WeightedChunk(D d, const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps,
                   int shift, bool round, std::int64_t maximum, std::int64_t* out) {
  auto total = hn::Zero(d);
  for (int k = 0; k < taps; ++k)
    total = hn::Add(
        total, hn::Mul(hn::LoadU(d, samples + std::size_t(k) * count), hn::LoadU(d, weights + std::size_t(k) * count)));
  if (round)
    total = hn::Add(total, hn::Set(d, std::int64_t{1024}));
  if (shift == 10)
    total = hn::ShiftRight<10>(total);
  else if (shift == 11)
    total = hn::ShiftRight<11>(total);
  else
    total = hn::ShiftRight<22>(total);
  hn::StoreU(hn::Min(hn::Max(total, hn::Zero(d)), hn::Set(d, maximum)), d, out);
}
template <int FixedTaps>
void WeightedRows(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
                  bool round, std::int64_t maximum, std::int64_t* out, std::size_t tap_stride) {
  const hn::ScalableTag<std::int64_t> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; i + lanes <= count; i += lanes)
    WeightedChunk(d, samples + i, weights + i, tap_stride, FixedTaps ? FixedTaps : taps, shift, round, maximum,
                  out + i);
  const hn::CappedTag<std::int64_t, 1> one;
  for (; i < count; ++i)
    WeightedChunk(one, samples + i, weights + i, tap_stride, FixedTaps ? FixedTaps : taps, shift, round, maximum,
                  out + i);
}
void Weighted(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
              bool round, std::int64_t maximum, std::int64_t* out, std::size_t tap_stride) {
  if (taps == 4)
    WeightedRows<4>(samples, weights, count, taps, shift, round, maximum, out, tap_stride);
  else if (taps == 16)
    WeightedRows<16>(samples, weights, count, taps, shift, round, maximum, out, tap_stride);
  else
    WeightedRows<0>(samples, weights, count, taps, shift, round, maximum, out, tap_stride);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::depan_rows
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::depan_rows {
HWY_EXPORT(Residuals);
HWY_EXPORT(Weighted);
HWY_EXPORT(NativeFma);
HWY_EXPORT(Adjust);
HWY_EXPORT(Accumulate);
HWY_EXPORT(StrictResiduals);
HWY_EXPORT(WeightAdmission);
bool weight_admission(const depan::Observations& observations, const float* dx, const float* dy, float wrong,
                      std::int8_t* eligibility) {
  return HWY_DYNAMIC_DISPATCH(WeightAdmission)(observations, dx, dy, wrong, eligibility);
}
bool strict_residuals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count,
                      depan::Transform map, float* ex, float* ey) {
  return HWY_DYNAMIC_DISPATCH(StrictResiduals)(x, y, dx, dy, count, map, ex, ey);
}
depan::FitSums accumulate(const depan::Observations& observations, const std::vector<float>& weights, const float* ex,
                          const float* ey, bool zoom, bool rotation, const depan::FitGeometry* geometry) {
  return HWY_DYNAMIC_DISPATCH(Accumulate)(observations, weights, ex, ey, zoom, rotation, geometry);
}
bool native_fma() {
  return HWY_DYNAMIC_DISPATCH(NativeFma)();
}
void adjust(const float* values, const float* scales, const float* gradients, std::size_t count, float* output) {
  HWY_DYNAMIC_DISPATCH(Adjust)(values, scales, gradients, count, output);
}
void residuals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count,
               depan::Transform map, float* ex, float* ey) {
  HWY_DYNAMIC_DISPATCH(Residuals)(x, y, dx, dy, count, map, ex, ey);
}
void weighted(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
              bool round, std::int64_t maximum, std::int64_t* out, std::size_t tap_stride) {
  HWY_DYNAMIC_DISPATCH(Weighted)(samples, weights, count, taps, shift, round, maximum, out,
                                 tap_stride ? tap_stride : count);
}
} // namespace neo_mv::simd::depan_rows
#endif
