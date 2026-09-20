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
template <class D>
void ResidualChunk(D d, const float* x, const float* y, const float* dx, const float* dy, depan::Transform t, float* ex,
                   float* ey) {
  const auto X = hn::LoadU(d, x), Y = hn::LoadU(d, y);
  const auto a =
      Sub(d, Sub(d, Add(d, Add(d, hn::Set(d, t.tx), Mul(d, hn::Set(d, t.u), X)), Mul(d, hn::Set(d, t.v), Y)), X),
          hn::LoadU(d, dx));
  const auto b =
      Sub(d, Sub(d, Add(d, Add(d, hn::Set(d, t.ty), Mul(d, hn::Set(d, t.w), X)), Mul(d, hn::Set(d, t.h), Y)), Y),
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
void Weighted(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
              bool round, std::int64_t maximum, std::int64_t* out) {
  const hn::ScalableTag<std::int64_t> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; i + lanes <= count; i += lanes)
    WeightedChunk(d, samples + i, weights + i, count, taps, shift, round, maximum, out + i);
  const hn::CappedTag<std::int64_t, 1> one;
  for (; i < count; ++i)
    WeightedChunk(one, samples + i, weights + i, count, taps, shift, round, maximum, out + i);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::depan_rows
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::depan_rows {
HWY_EXPORT(Residuals);
HWY_EXPORT(Weighted);
void residuals(const float* x, const float* y, const float* dx, const float* dy, std::size_t count,
               depan::Transform map, float* ex, float* ey) {
  HWY_DYNAMIC_DISPATCH(Residuals)(x, y, dx, dy, count, map, ex, ey);
}
void weighted(const std::int64_t* samples, const std::int64_t* weights, std::size_t count, int taps, int shift,
              bool round, std::int64_t maximum, std::int64_t* out) {
  HWY_DYNAMIC_DISPATCH(Weighted)(samples, weights, count, taps, shift, round, maximum, out);
}
} // namespace neo_mv::simd::depan_rows
#endif
