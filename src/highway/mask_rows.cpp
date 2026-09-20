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
void ResizeChunk(D d, const double* top, const double* bottom, const std::int64_t* left, const std::int64_t* right,
                 const double* rx, double dx, double dy, double ry, float* out) {
  const hn::Rebind<std::int64_t, D> di;
  const auto ix = hn::LoadU(di, left), jx = hn::LoadU(di, right);
  const auto s0 = hn::GatherIndex(d, top, ix), s1 = hn::GatherIndex(d, top, jx);
  const auto s2 = hn::GatherIndex(d, bottom, ix), s3 = hn::GatherIndex(d, bottom, jx);
  const auto a = hn::Div(hn::LoadU(d, rx), hn::Set(d, dx)), b = hn::Set(d, ry / dy);
  const auto c = hn::Sub(hn::Set(d, 1.0), a), e = hn::Sub(hn::Set(d, 1.0), b);
  const auto h0 = hn::Add(hn::Mul(c, s0), hn::Mul(a, s1)), h1 = hn::Add(hn::Mul(c, s2), hn::Mul(a, s3));
  auto value = hn::Add(hn::Mul(e, h0), hn::Mul(b, h1));
  // Reject outside the finite round-to-nearest interval before demotion.
  if (!hn::AllTrue(d, hn::Lt(hn::Abs(value), hn::Set(d, 0x1.ffffffp127))))
    throw std::invalid_argument("non-finite resized float");
  const auto max = hn::Set(d, double(std::numeric_limits<float>::max()));
  value = hn::Min(max, hn::Max(hn::Neg(max), value));
  const hn::Rebind<float, D> df;
  hn::StoreU(hn::DemoteTo(df, value), df, out);
}
#endif
void ResizeF32(const double* top, const double* bottom, const std::int64_t* left, const std::int64_t* right,
               const double* rx, int width, double dx, double dy, double ry, float* out) {
#if HWY_HAVE_FLOAT64
  const hn::ScalableTag<double> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= width - n; x += n)
    ResizeChunk(d, top, bottom, left + x, right + x, rx + x, dx, dy, ry, out + x);
  const hn::CappedTag<double, 1> one;
  for (; x < width; ++x)
    ResizeChunk(one, top, bottom, left + x, right + x, rx + x, dx, dy, ry, out + x);
#else
  for (int x = 0; x < width; ++x) {
    const double s0 = top[left[x]], s1 = top[right[x]];
    const double s2 = bottom[left[x]], s3 = bottom[right[x]];
    const double a = rx[x] / dx, b = ry / dy;
    const double c = 1.0 - a, e = 1.0 - b;
    const double h0 = c * s0 + a * s1;
    const double h1 = c * s2 + a * s3;
    out[x] = mask_detail::binary32(e * h0 + b * h1);
  }
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
template <class D, class T>
void ResizeIntegerChunk(D d, const std::int32_t* top, const std::int32_t* bottom, const std::int32_t* left,
                        const std::int32_t* right, const std::int32_t* coefficients, std::int32_t vertical,
                        bool horizontal_first, T* out) {
  const auto ix = hn::LoadU(d, left), jx = hn::LoadU(d, right);
  const auto s0 = hn::GatherIndex(d, top, ix), s1 = hn::GatherIndex(d, top, jx);
  const auto s2 = hn::GatherIndex(d, bottom, ix), s3 = hn::GatherIndex(d, bottom, jx);
  const auto a = hn::LoadU(d, coefficients), b = hn::Set(d, vertical);
  auto value = horizontal_first ? Interpolate(d, Interpolate(d, s0, s1, a), Interpolate(d, s2, s3, a), b)
                                : Interpolate(d, Interpolate(d, s0, s2, b), Interpolate(d, s1, s3, b), a);
  if constexpr (std::is_same_v<T, std::int16_t>)
    value = hn::Sub(value, hn::Set(d, 32768));
  const hn::Rebind<T, D> narrow;
  hn::StoreU(hn::DemoteTo(narrow, value), narrow, out);
}
template <class T>
void ResizeInteger(const std::int32_t* top, const std::int32_t* bottom, const std::int32_t* left,
                   const std::int32_t* right, const std::int32_t* coefficients, int width, std::int32_t vertical,
                   bool horizontal_first, T* out) {
  const hn::ScalableTag<std::int32_t> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= width - n; x += n)
    ResizeIntegerChunk(d, top, bottom, left + x, right + x, coefficients + x, vertical, horizontal_first, out + x);
  const hn::CappedTag<std::int32_t, 1> one;
  for (; x < width; ++x)
    ResizeIntegerChunk(one, top, bottom, left + x, right + x, coefficients + x, vertical, horizontal_first, out + x);
}
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
#define NEO_RESIZE(T, S)                                                                                               \
  void Resize##S(const std::int32_t* a, const std::int32_t* b, const std::int32_t* l, const std::int32_t* r,           \
                 const std::int32_t* x, int n, std::int32_t vertical, bool horizontal_first, T* o) {                   \
    ResizeInteger(a, b, l, r, x, n, vertical, horizontal_first, o);                                                    \
  }
NEO_RESIZE(std::uint8_t, U8)
NEO_RESIZE(std::uint16_t, U16)
NEO_RESIZE(std::int16_t, I16)
#undef NEO_RESIZE
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
#define NEO_EXPORT(T, S)                                                                                               \
  HWY_EXPORT(Resize##S);                                                                                               \
  void resize(const std::int32_t* a, const std::int32_t* b, const std::int32_t* l, const std::int32_t* r,              \
              const std::int32_t* x, int n, std::int32_t vertical, bool horizontal_first, T* o) {                      \
    HWY_DYNAMIC_DISPATCH(Resize##S)(a, b, l, r, x, n, vertical, horizontal_first, o);                                  \
  }
NEO_EXPORT(std::uint8_t, U8)
NEO_EXPORT(std::uint16_t, U16) NEO_EXPORT(std::int16_t, I16)
#undef NEO_EXPORT
    HWY_EXPORT(ResizeF32);
void resize(const double* a, const double* b, const std::int64_t* l, const std::int64_t* r, const double* x, int n,
            double dx, double dy, double ry, float* o) {
  HWY_DYNAMIC_DISPATCH(ResizeF32)(a, b, l, r, x, n, dx, dy, ry, o);
}
} // namespace neo_mv::simd::mask_rows
#endif
