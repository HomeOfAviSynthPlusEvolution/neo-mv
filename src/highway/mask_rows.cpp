#include "highway/mask_rows.hpp"
#include "core/mask/numeric.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/mask_rows.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"
HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::mask_rows {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class D, class V>
void Check(D d, V v) {
  if (!hn::AllTrue(d, hn::IsFinite(v)))
    throw std::overflow_error("non-finite SIMD mask score intermediate");
}
template <class D, class V>
auto Power(D d, V value, hn::TFromD<D> exponent) {
  // Preserve the approved scalar libm result for general exponents. The common
  // exact exponents need no transcendental approximation.
  if (exponent == 0)
    return hn::Set(d, hn::TFromD<D>(1));
  if (exponent == 1)
    return hn::IfThenElse(hn::Eq(value, hn::Zero(d)), hn::Zero(d), value);
  HWY_ALIGN hn::TFromD<D> lanes[hn::MaxLanes(d)];
  hn::StoreU(value, d, lanes);
  for (std::size_t i = 0; i < hn::Lanes(d); ++i)
    lanes[i] = mask_detail::power(lanes[i], exponent);
  return hn::LoadU(d, lanes);
}
#if HWY_HAVE_FLOAT64
template <class D>
void MagnitudeChunk(D d, const double* x, const double* y, int pel, float f2, float exponent, float maximum,
                    double* out) {
  const auto inv = hn::Set(d, 1.0 / double(pel));
  const auto vx = hn::Mul(hn::LoadU(d, x), inv), vy = hn::Mul(hn::LoadU(d, y), inv);
  const auto xx = hn::Mul(vx, vx), yy = hn::Mul(vy, vy);
  Check(d, xx);
  Check(d, yy);
  const auto q = hn::Add(xx, yy);
  Check(d, q);
  const auto a = hn::Mul(q, hn::Set(d, double(f2)));
  Check(d, a);
  const auto score = hn::Mul(hn::Set(d, double(maximum)), Power(d, a, double(exponent)));
  Check(d, score);
  hn::StoreU(score, d, out);
}
#endif
void Magnitude(const double* x, const double* y, std::size_t count, int pel, float f2, float exponent, float maximum,
               double* out) {
#if HWY_HAVE_FLOAT64
  const hn::ScalableTag<double> d;
  const auto n = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= n; i += n)
    MagnitudeChunk(d, x + i, y + i, pel, f2, exponent, maximum, out + i);
  const hn::CappedTag<double, 1> one;
  for (; i < count; ++i)
    MagnitudeChunk(one, x + i, y + i, pel, f2, exponent, maximum, out + i);
#else
  // Some SIMD targets have no binary64 lanes. Keep the specified binary64
  // arithmetic here; float32 vector arithmetic would change the mask values.
  const auto checked = [](double value) {
    if (!std::isfinite(value))
      throw std::overflow_error("non-finite mask score intermediate");
    return value;
  };
  for (std::size_t i = 0; i < count; ++i) {
    const double vx = x[i] / double(pel), vy = y[i] / double(pel);
    const double xx = checked(vx * vx), yy = checked(vy * vy);
    const double q = checked(xx + yy);
    const double a = checked(q * double(f2));
    out[i] = checked(double(maximum) * mask_detail::power(a, double(exponent)));
  }
#endif
}
template <class D>
void SadChunk(D d, const float* samples, float scale, float exponent, float maximum, float* out) {
  const auto z = hn::Mul(hn::LoadU(d, samples), hn::Set(d, scale));
  Check(d, z);
  const auto score = hn::Mul(hn::Set(d, maximum), Power(d, z, exponent));
  Check(d, score);
  hn::StoreU(score, d, out);
}
void Sad(const float* samples, std::size_t count, float scale, float exponent, float maximum, float* out) {
  const hn::ScalableTag<float> d;
  const auto n = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= n; i += n)
    SadChunk(d, samples + i, scale, exponent, maximum, out + i);
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    SadChunk(one, samples + i, scale, exponent, maximum, out + i);
}
#if HWY_HAVE_FLOAT64
template <class D>
void ResizeFloatVerticalChunk(D d, const double* top, const double* bottom, double coefficient, float* output) {
  const auto a = hn::Set(d, 1.0 - coefficient), b = hn::Set(d, coefficient);
  auto value = hn::Add(hn::Mul(a, hn::LoadU(d, top)), hn::Mul(b, hn::LoadU(d, bottom)));
  if (!hn::AllTrue(d, hn::Lt(hn::Abs(value), hn::Set(d, 0x1.ffffffp127))))
    throw std::invalid_argument("non-finite resized float");
  const auto max = hn::Set(d, double(std::numeric_limits<float>::max()));
  value = hn::Min(max, hn::Max(hn::Neg(max), value));
  const hn::Rebind<float, D> df;
  hn::StoreU(hn::DemoteTo(df, value), df, output);
}
#endif
void ResizeFloatVertical(const double* top, const double* bottom, int width, double coefficient, float* output) {
#if HWY_HAVE_FLOAT64
  const hn::ScalableTag<double> d;
  const int lanes = int(hn::Lanes(d));
  int x = 0;
  for (; x <= width - lanes; x += lanes)
    ResizeFloatVerticalChunk(d, top + x, bottom + x, coefficient, output + x);
  const hn::CappedTag<double, 1> one;
  for (; x < width; ++x)
    ResizeFloatVerticalChunk(one, top + x, bottom + x, coefficient, output + x);
#else
  for (int x = 0; x < width; ++x)
    output[x] = mask_detail::binary32((1.0 - coefficient) * top[x] + coefficient * bottom[x]);
#endif
}

template <class D, class V>
auto Interpolate(D d, V s0, V s1, V coefficient) {
  // Biased samples are at most 65535 and complementary Q14 coefficients sum
  // to 16384. The product sum plus 8192 fits signed int32 at either stage.
  const auto complement = hn::Sub(hn::Set(d, 16384), coefficient);
  const auto sum = hn::Add(hn::Mul(complement, s0), hn::Mul(coefficient, s1));
  return hn::ShiftRight<14>(hn::Add(sum, hn::Set(d, 8192)));
}
template <bool Horizontal, class D, class T>
void ResizePassChunk(D d, const std::int32_t* top, const std::int32_t* bottom, const std::int32_t* left,
                     const std::int32_t* right, const std::int32_t* weights, int x, std::int32_t vertical, T* output) {
  auto a = hn::Zero(d), b = a, c = a;
  if constexpr (Horizontal) {
    a = hn::GatherIndex(d, top, hn::LoadU(d, left + x));
    b = hn::GatherIndex(d, top, hn::LoadU(d, right + x));
    c = hn::LoadU(d, weights + x);
  } else {
    a = hn::LoadU(d, top + x);
    b = hn::LoadU(d, bottom + x);
    c = hn::Set(d, vertical);
  }
  auto value = Interpolate(d, a, b, c);
  if constexpr (std::is_same_v<T, std::int32_t>)
    hn::StoreU(value, d, output + x);
  else {
    if constexpr (std::is_same_v<T, std::int16_t>)
      value = hn::Sub(value, hn::Set(d, 32768));
    const hn::Rebind<T, D> narrow;
    hn::StoreU(hn::DemoteTo(narrow, value), narrow, output + x);
  }
}
template <bool Horizontal, class T>
void ResizePass(const std::int32_t* top, const std::int32_t* bottom, const std::int32_t* left,
                const std::int32_t* right, const std::int32_t* weights, int width, std::int32_t vertical, T* output) {
  const hn::ScalableTag<std::int32_t> d;
  const int lanes = int(hn::Lanes(d));
  int x = 0;
  for (; x <= width - lanes; x += lanes)
    ResizePassChunk<Horizontal>(d, top, bottom, left, right, weights, x, vertical, output);
  const hn::CappedTag<std::int32_t, 1> one;
  for (; x < width; ++x)
    ResizePassChunk<Horizontal>(one, top, bottom, left, right, weights, x, vertical, output);
}
#define NEO_PASS(T, S)                                                                                                 \
  void ResizePass##S(const std::int32_t* a, const std::int32_t* b, const std::int32_t* l, const std::int32_t* r,       \
                     const std::int32_t* w, int n, std::int32_t v, T* o) {                                             \
    if (l)                                                                                                             \
      ResizePass<true>(a, b, l, r, w, n, v, o);                                                                        \
    else                                                                                                               \
      ResizePass<false>(a, b, l, r, w, n, v, o);                                                                       \
  }
NEO_PASS(std::int32_t, I32)
NEO_PASS(std::int16_t, I16)
NEO_PASS(std::uint16_t, U16)
NEO_PASS(std::uint8_t, U8)
#undef NEO_PASS
template <class T>
void Max(T* samples, std::size_t count, T value) {
  const hn::ScalableTag<T> d;
  const auto n = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= n; i += n) {
    const auto old = hn::LoadU(d, samples + i), v = hn::Set(d, value);
    hn::StoreU(hn::IfThenElse(hn::Lt(old, v), v, old), d, samples + i);
  }
  for (; i < count; ++i)
    samples[i] = std::max(samples[i], value);
}

void MaxU8(std::uint8_t* p, std::size_t n, std::uint8_t v) {
  Max(p, n, v);
}
void MaxU16(std::uint16_t* p, std::size_t n, std::uint16_t v) {
  Max(p, n, v);
}
void MaxF32(float* p, std::size_t n, float v) {
  Max(p, n, v);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::mask_rows
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::mask_rows {
#define NEO_PASS_EXPORT(T, S)                                                                                          \
  HWY_EXPORT(ResizePass##S);                                                                                           \
  void resize_pass(const std::int32_t* a, const std::int32_t* b, const std::int32_t* l, const std::int32_t* r,         \
                   const std::int32_t* w, int n, std::int32_t v, T* o) {                                               \
    HWY_DYNAMIC_DISPATCH(ResizePass##S)(a, b, l, r, w, n, v, o);                                                       \
  }
NEO_PASS_EXPORT(std::int32_t, I32)
NEO_PASS_EXPORT(std::int16_t, I16)
NEO_PASS_EXPORT(std::uint16_t, U16)
NEO_PASS_EXPORT(std::uint8_t, U8)
#undef NEO_PASS_EXPORT
HWY_EXPORT(Magnitude);
HWY_EXPORT(Sad);
HWY_EXPORT(MaxU8);
HWY_EXPORT(MaxU16);
HWY_EXPORT(MaxF32);
void magnitude(const double* x, const double* y, std::size_t n, int p, float f, float e, float m, double* o) {
  HWY_DYNAMIC_DISPATCH(Magnitude)(x, y, n, p, f, e, m, o);
}
void sad(const float* p, std::size_t n, float s, float e, float m, float* o) {
  HWY_DYNAMIC_DISPATCH(Sad)(p, n, s, e, m, o);
}
void max_span(std::uint8_t* p, std::size_t n, std::uint8_t v) {
  HWY_DYNAMIC_DISPATCH(MaxU8)(p, n, v);
}
void max_span(std::uint16_t* p, std::size_t n, std::uint16_t v) {
  HWY_DYNAMIC_DISPATCH(MaxU16)(p, n, v);
}
void max_span(float* p, std::size_t n, float v) {
  HWY_DYNAMIC_DISPATCH(MaxF32)(p, n, v);
}
HWY_EXPORT(ResizeFloatVertical);
void resize_float_vertical(const double* top, const double* bottom, int width, double coefficient, float* output) {
  HWY_DYNAMIC_DISPATCH(ResizeFloatVertical)(top, bottom, width, coefficient, output);
}
} // namespace neo_mv::simd::mask_rows
#endif
