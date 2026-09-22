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
bool NativeFma() { return HWY_NATIVE_FMA != 0; }
struct FitArithmetic {
  static float multiply_add(float a, float b, float c) {
    const hn::CappedTag<float, 1> d;
    // One lane preserves the serial reduction order on every target.
    return hn::GetLane(MulAdd(d, hn::Set(d, a), hn::Set(d, b), hn::Set(d, c)));
  }
};
depan::FitSums Accumulate(const depan::Observations& observations, const std::vector<float>& weights,
                          const float* ex, const float* ey, bool zoom, bool rotation) {
  return depan::accumulate_fit<FitArithmetic>(observations, weights,
      [ex, ey](std::size_t i) { return std::array<float, 2>{ex[i], ey[i]}; }, zoom, rotation);
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
  const auto a =
      Sub(d, Sub(d, MulAdd(d, hn::Set(d, t.v), Y, MulAdd(d, hn::Set(d, t.u), X, hn::Set(d, t.tx))), X),
          hn::LoadU(d, dx));
  const auto b =
      Sub(d, Sub(d, MulAdd(d, hn::Set(d, t.h), Y, MulAdd(d, hn::Set(d, t.w), X, hn::Set(d, t.ty))), Y),
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
    WeightedChunk(d, samples + i, weights + i, tap_stride, FixedTaps ? FixedTaps : taps, shift, round, maximum, out + i);
  const hn::CappedTag<std::int64_t, 1> one;
  for (; i < count; ++i)
    WeightedChunk(one, samples + i, weights + i, tap_stride, FixedTaps ? FixedTaps : taps, shift, round, maximum, out + i);
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
depan::FitSums accumulate(const depan::Observations& observations, const std::vector<float>& weights,
                          const float* ex, const float* ey, bool zoom, bool rotation) {
  return HWY_DYNAMIC_DISPATCH(Accumulate)(observations, weights, ex, ey, zoom, rotation);
}
bool native_fma() { return HWY_DYNAMIC_DISPATCH(NativeFma)(); }
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
