#include "highway/estimate_image.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/estimate_image.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::estimate {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class D, class V>
V ImageFinite(D d, V v) {
  if (!hn::AllTrue(d, hn::IsFinite(v)))
    throw std::invalid_argument("non-finite DepanEstimate image sample/intermediate");
  return v;
}
template <class D, class T>
void ExtractChunk(D d, const T* source, float* output, float maximum) {
  const hn::Rebind<T, D> narrow;
  auto v = hn::Zero(d);
  if constexpr (std::is_same_v<T, float>)
    v = ImageFinite(d, hn::LoadU(d, source));
  else {
    const hn::Rebind<std::int32_t, D> wide;
    v = hn::ConvertTo(d, hn::PromoteTo(wide, hn::LoadU(narrow, source)));
    if (!hn::AllTrue(d, hn::Le(v, hn::Set(d, maximum))))
      throw std::invalid_argument("DepanEstimate sample exceeds nominal precision");
  }
  hn::StoreU(v, d, output);
}
template <class T>
void ExtractImage(const T* source, float* output, std::size_t count, float maximum) {
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= lanes; i += lanes)
    ExtractChunk(d, source + i, output + i, maximum);
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    ExtractChunk(one, source + i, output + i, maximum);
}
template <class D, class T>
void DisplayChunk(D d, const float* source, T* output, float minimum, float norm, float maximum) {
  const auto delta = ImageFinite(d, hn::Sub(ImageFinite(d, hn::LoadU(d, source)), hn::Set(d, minimum)));
  const auto q = ImageFinite(d, hn::Mul(delta, hn::Set(d, norm)));
  if constexpr (std::is_same_v<T, float>)
    hn::StoreU(q, d, output);
  else {
    const auto integer = hn::Trunc(q);
    if (!hn::AllTrue(d, hn::And(hn::Ge(integer, hn::Zero(d)), hn::Le(integer, hn::Set(d, maximum)))))
      throw std::invalid_argument("DepanEstimate display integer is outside nominal precision");
    const hn::Rebind<std::int32_t, D> wide;
    const hn::Rebind<T, D> narrow;
    hn::StoreU(hn::DemoteTo(narrow, hn::ConvertTo(wide, integer)), narrow, output);
  }
}
template <class T>
void DisplayImage(const float* source, T* output, std::size_t count, float minimum, float norm, float maximum) {
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= lanes; i += lanes)
    DisplayChunk(d, source + i, output + i, minimum, norm, maximum);
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    DisplayChunk(one, source + i, output + i, minimum, norm, maximum);
}
void Extrema(const float* source, std::size_t count, float& minimum, float& maximum) {
  if (!count)
    throw std::invalid_argument("empty DepanEstimate correlation display");
  minimum = maximum = depan::finite(source[0]);
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= lanes; i += lanes) {
    const auto v = ImageFinite(d, hn::LoadU(d, source + i));
    const float lo = hn::GetLane(hn::MinOfLanes(d, v)), hi = hn::GetLane(hn::MaxOfLanes(d, v));
    // Select the first actual sample so equal signed zeros retain scalar order.
    if (lo < minimum)
      minimum = source[i + hn::FindFirstTrue(d, hn::Eq(v, hn::Set(d, lo)))];
    if (hi > maximum)
      maximum = source[i + hn::FindFirstTrue(d, hn::Eq(v, hn::Set(d, hi)))];
  }
  for (; i < count; ++i) {
    const auto v = depan::finite(source[i]);
    if (v < minimum)
      minimum = v;
    if (v > maximum)
      maximum = v;
  }
}
void Extract8(const std::uint8_t* p, float* q, std::size_t n, float m) {
  ExtractImage(p, q, n, m);
}
void Extract16(const std::uint16_t* p, float* q, std::size_t n, float m) {
  ExtractImage(p, q, n, m);
}
void Extract32(const float* p, float* q, std::size_t n, float m) {
  ExtractImage(p, q, n, m);
}
void Display8(const float* p, std::uint8_t* q, std::size_t n, float a, float b, float c) {
  DisplayImage(p, q, n, a, b, c);
}
void Display16(const float* p, std::uint16_t* q, std::size_t n, float a, float b, float c) {
  DisplayImage(p, q, n, a, b, c);
}
void Display32(const float* p, float* q, std::size_t n, float a, float b, float c) {
  DisplayImage(p, q, n, a, b, c);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::estimate
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::estimate {
HWY_EXPORT(Extract8);
HWY_EXPORT(Extract16);
HWY_EXPORT(Extract32);
HWY_EXPORT(Display8);
HWY_EXPORT(Display16);
HWY_EXPORT(Display32);
HWY_EXPORT(Extrema);
void extract_row(const std::uint8_t* p, float* q, std::size_t n, float m) {
  HWY_DYNAMIC_DISPATCH(Extract8)(p, q, n, m);
}
void extract_row(const std::uint16_t* p, float* q, std::size_t n, float m) {
  HWY_DYNAMIC_DISPATCH(Extract16)(p, q, n, m);
}
void extract_row(const float* p, float* q, std::size_t n, float m) {
  HWY_DYNAMIC_DISPATCH(Extract32)(p, q, n, m);
}
void display_row(const float* p, std::uint8_t* q, std::size_t n, float a, float b, float c) {
  HWY_DYNAMIC_DISPATCH(Display8)(p, q, n, a, b, c);
}
void display_row(const float* p, std::uint16_t* q, std::size_t n, float a, float b, float c) {
  HWY_DYNAMIC_DISPATCH(Display16)(p, q, n, a, b, c);
}
void display_row(const float* p, float* q, std::size_t n, float a, float b, float c) {
  HWY_DYNAMIC_DISPATCH(Display32)(p, q, n, a, b, c);
}
void extrema(const float* p, std::size_t n, float& a, float& b) {
  HWY_DYNAMIC_DISPATCH(Extrema)(p, n, a, b);
}
} // namespace neo_mv::simd::estimate
#endif
