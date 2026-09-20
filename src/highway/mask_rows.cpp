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
template <class D, class T>
void ResizeChunk(D d, const double* top, const double* bottom, const std::int64_t* left, const std::int64_t* right,
                 const double* rx, double dx, double dy, double ry, T* out) {
  const hn::Rebind<std::int64_t, D> di;
  const auto ix = hn::LoadU(di, left), jx = hn::LoadU(di, right);
  const auto s0 = hn::GatherIndex(d, top, ix), s1 = hn::GatherIndex(d, top, jx);
  const auto s2 = hn::GatherIndex(d, bottom, ix), s3 = hn::GatherIndex(d, bottom, jx);
  auto value = hn::Zero(d);
  if constexpr (std::is_same_v<T, float>) {
    const auto a = hn::Div(hn::LoadU(d, rx), hn::Set(d, dx)), b = hn::Set(d, ry / dy);
    const auto c = hn::Sub(hn::Set(d, 1.0), a), e = hn::Sub(hn::Set(d, 1.0), b);
    const auto h0 = hn::Add(hn::Mul(c, s0), hn::Mul(a, s1)), h1 = hn::Add(hn::Mul(c, s2), hn::Mul(a, s3));
    value = hn::Add(hn::Mul(e, h0), hn::Mul(b, h1));
    // Reject outside the finite round-to-nearest interval before demotion.
    if (!hn::AllTrue(d, hn::Lt(hn::Abs(value), hn::Set(d, 0x1.ffffffp127))))
      throw std::invalid_argument("non-finite resized float");
    const auto max = hn::Set(d, double(std::numeric_limits<float>::max()));
    value = hn::Min(max, hn::Max(hn::Neg(max), value));
    const hn::Rebind<float, D> df;
    hn::StoreU(hn::DemoteTo(df, value), df, out);
  } else {
    const auto r = hn::LoadU(d, rx), c = hn::Sub(hn::Set(d, dx), r);
    const auto b = hn::Set(d, ry), e = hn::Set(d, dy - ry);
    // All products/sums are exact integers below 2^48. The denominator is at
    // most 2^32: quotient rounding error is less than the distance to the next
    // integer, so truncating this binary64 division gives the exact quotient.
    auto sum = hn::Add(hn::Mul(hn::Mul(c, e), s0), hn::Mul(hn::Mul(r, e), s1));
    sum = hn::Add(sum, hn::Mul(hn::Mul(c, b), s2));
    sum = hn::Add(sum, hn::Mul(hn::Mul(r, b), s3));
    value = hn::Div(hn::Add(sum, hn::Set(d, dx * dy / 2)), hn::Set(d, dx * dy));
    const hn::Rebind<std::int32_t, D> ds;
    auto integers = hn::DemoteTo(ds, value);
    if constexpr (std::is_same_v<T, std::int16_t>)
      integers = hn::Sub(integers, hn::Set(ds, 32768));
    const hn::Rebind<T, D> narrow;
    hn::StoreU(hn::DemoteTo(narrow, integers), narrow, out);
  }
}
#endif
template <class T>
void Resize(const double* top, const double* bottom, const std::int64_t* left, const std::int64_t* right,
            const double* rx, int width, double dx, double dy, double ry, T* out) {
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
    if constexpr (std::is_same_v<T, float>) {
      const double a = rx[x] / dx, b = ry / dy;
      const double c = 1.0 - a, e = 1.0 - b;
      const double h0 = c * s0 + a * s1;
      const double h1 = c * s2 + a * s3;
      out[x] = mask_detail::binary32(e * h0 + b * h1);
    } else {
      const double r = rx[x], c = dx - r;
      const double b = ry, e = dy - ry;
      double sum = (c * e) * s0 + (r * e) * s1;
      sum = sum + (c * b) * s2;
      sum = sum + (r * b) * s3;
      // This row API admits integer denominators <= 2^32 only. Products and
      // sums are exact in binary64; division followed by truncation preserves
      // the exact rounded-integer formula, as in the vector branch above.
      const double denominator = dx * dy;
      auto integer = static_cast<std::int32_t>((sum + denominator / 2) / denominator);
      if constexpr (std::is_same_v<T, std::int16_t>)
        integer -= 32768;
      out[x] = static_cast<T>(integer);
    }
  }
#endif
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
  void Resize##S(const double* a, const double* b, const std::int64_t* l, const std::int64_t* r, const double* x,      \
                 int n, double dx, double dy, double ry, T* o) {                                                       \
    Resize(a, b, l, r, x, n, dx, dy, ry, o);                                                                           \
  }
NEO_RESIZE(std::uint8_t, U8)
NEO_RESIZE(std::uint16_t, U16)
NEO_RESIZE(std::int16_t, I16)
NEO_RESIZE(float, F32)
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
  void resize(const double* a, const double* b, const std::int64_t* l, const std::int64_t* r, const double* x, int n,  \
              double dx, double dy, double ry, T* o) {                                                                 \
    HWY_DYNAMIC_DISPATCH(Resize##S)(a, b, l, r, x, n, dx, dy, ry, o);                                                  \
  }
NEO_EXPORT(std::uint8_t, U8) NEO_EXPORT(std::uint16_t, U16) NEO_EXPORT(std::int16_t, I16) NEO_EXPORT(float, F32)
#undef NEO_EXPORT
} // namespace neo_mv::simd::mask_rows
#endif
