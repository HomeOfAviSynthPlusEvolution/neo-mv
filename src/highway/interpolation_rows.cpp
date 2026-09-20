#include "highway/interpolation_rows.hpp"
#include <stdexcept>
#include <type_traits>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/interpolation_rows.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::interpolation_rows {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class D, class V>
auto Checked(D d, V value) {
  if constexpr (std::is_same_v<hn::TFromD<D>, float>)
    if (!hn::AllTrue(d, hn::IsFinite(value)))
      throw std::invalid_argument("non-finite SIMD interpolation intermediate");
  return value;
}
template <class D, class V>
auto Add(D d, V a, V b) {
  return Checked(d, hn::Add(a, b));
}
template <class D, class V>
auto Mul(D d, V a, V b) {
  return Checked(d, hn::Mul(a, b));
}
template <class D, class V>
auto H(D d, V z) {
  if constexpr (std::is_same_v<hn::TFromD<D>, float>)
    return Mul(d, z, hn::Set(d, 1.0f / 256));
  else
    return hn::ShiftRight<8>(hn::Add(z, hn::Set(d, 256)));
}
template <class V>
auto MinFirst(V a, V b) {
  return hn::IfThenElse(hn::Lt(b, a), b, a);
}
template <class V>
auto MaxFirst(V a, V b) {
  return hn::IfThenElse(hn::Lt(a, b), b, a);
}

template <class D, class T>
void ComposeChunk(D d, const T* a, const T* c, const T* x, const T* y, const T* mf, const T* mb, bool extra, int time,
                  T* output) {
  const auto A = hn::LoadU(d, a), C = hn::LoadU(d, c), X = hn::LoadU(d, x), Y = hn::LoadU(d, y);
  const auto F = hn::LoadU(d, mf), B = hn::LoadU(d, mb), full = hn::Set(d, T(256));
  const auto f = hn::Sub(full, F), b = hn::Sub(full, B);
  auto U = hn::Zero(d), V = hn::Zero(d);
  if (extra) {
    const auto lo = MinFirst(A, C), hi = MaxFirst(A, C);
    const auto CK = MaxFirst(lo, MinFirst(Y, hi)), CE = MaxFirst(lo, MinFirst(X, hi));
    U = H(d, Add(d, Mul(d, CK, F), Mul(d, A, f)));
    V = H(d, Add(d, Mul(d, CE, B), Mul(d, C, b)));
  } else {
    // Integer maximum inner numerator including H's bias is 4,278,125,056,
    // below UINT32_MAX. Every subsequent weighted sum is smaller.
    const auto innerU = H(d, Mul(d, F, Add(d, Mul(d, C, b), Mul(d, B, X))));
    U = H(d, Add(d, Mul(d, A, f), innerU));
    const auto innerV = H(d, Mul(d, B, Add(d, Mul(d, A, f), Mul(d, F, Y))));
    V = H(d, Add(d, Mul(d, C, b), innerV));
  }
  const auto sum = Add(d, Mul(d, U, hn::Set(d, T(256 - time))), Mul(d, V, hn::Set(d, T(time))));
  auto value = hn::Zero(d);
  if constexpr (std::is_same_v<T, float>)
    value = H(d, sum);
  else
    value = hn::Sub(hn::ShiftRight<8>(sum), hn::Set(d, 1));
  hn::StoreU(value, d, output);
}
template <class T>
void Compose(const T* a, const T* c, const T* x, const T* y, const T* mf, const T* mb, int width, bool extra, int time,
             T* out) {
  const hn::ScalableTag<T> d;
  const int n = int(hn::Lanes(d));
  int i = 0;
  for (; i <= width - n; i += n)
    ComposeChunk(d, a + i, c + i, x + i, y + i, mf + i, mb + i, extra, time, out + i);
  const hn::CappedTag<T, 1> one;
  for (; i < width; ++i)
    ComposeChunk(one, a + i, c + i, x + i, y + i, mf + i, mb + i, extra, time, out + i);
}
template <class D, class T>
void BlendChunk(D d, const T* a, const T* b, int time, T* out) {
  const auto value =
      Add(d, Mul(d, hn::LoadU(d, a), hn::Set(d, T(256 - time))), Mul(d, hn::LoadU(d, b), hn::Set(d, T(time))));
  if constexpr (std::is_same_v<T, float>)
    hn::StoreU(H(d, value), d, out);
  else
    hn::StoreU(hn::ShiftRight<8>(value), d, out);
}
template <class T>
void Blend(const T* a, const T* b, int width, int time, T* out) {
  const hn::ScalableTag<T> d;
  const int n = int(hn::Lanes(d));
  int i = 0;
  for (; i <= width - n; i += n)
    BlendChunk(d, a + i, b + i, time, out + i);
  const hn::CappedTag<T, 1> one;
  for (; i < width; ++i)
    BlendChunk(one, a + i, b + i, time, out + i);
}
template <class T>
std::uint32_t Sum(const T* samples, std::size_t count) {
  const hn::ScalableTag<std::uint32_t> d;
  const hn::Rebind<T, decltype(d)> narrow;
  const auto n = hn::Lanes(d);
  auto accum = hn::Zero(d);
  std::size_t i = 0;
  for (; i + n <= count; i += n)
    accum = hn::Add(accum, hn::PromoteTo(d, hn::LoadU(narrow, samples + i)));
  auto total = hn::ReduceSum(d, accum);
  for (; i < count; ++i)
    total += samples[i];
  return total; // At most 65537*65535 == UINT32_MAX, without modular overflow.
}
#define NEO_TEMPORAL_IMPL(T, S)                                                                                        \
  void Compose##S(const T* a, const T* c, const T* x, const T* y, const T* mf, const T* mb, int w, bool e, int t,      \
                  T* out) {                                                                                            \
    Compose(a, c, x, y, mf, mb, w, e, t, out);                                                                         \
  }                                                                                                                    \
  void Blend##S(const T* a, const T* b, int w, int t, T* out) {                                                        \
    Blend(a, b, w, t, out);                                                                                            \
  }
NEO_TEMPORAL_IMPL(std::uint32_t, U32)
NEO_TEMPORAL_IMPL(float, F32)
#undef NEO_TEMPORAL_IMPL
std::uint32_t SumU8(const std::uint8_t* p, std::size_t n) {
  return Sum(p, n);
}
std::uint32_t SumU16(const std::uint16_t* p, std::size_t n) {
  return Sum(p, n);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::interpolation_rows
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace neo_mv::simd::interpolation_rows {
#define NEO_TEMPORAL_EXPORT(T, S)                                                                                      \
  HWY_EXPORT(Compose##S);                                                                                              \
  HWY_EXPORT(Blend##S);                                                                                                \
  void compose(const T* a, const T* c, const T* x, const T* y, const T* mf, const T* mb, int w, bool e, int t,         \
               T* out) {                                                                                               \
    HWY_DYNAMIC_DISPATCH(Compose##S)(a, c, x, y, mf, mb, w, e, t, out);                                                \
  }                                                                                                                    \
  void blend(const T* a, const T* b, int w, int t, T* out) {                                                           \
    HWY_DYNAMIC_DISPATCH(Blend##S)(a, b, w, t, out);                                                                   \
  }
NEO_TEMPORAL_EXPORT(std::uint32_t, U32)
NEO_TEMPORAL_EXPORT(float, F32)
#undef NEO_TEMPORAL_EXPORT
HWY_EXPORT(SumU8);
HWY_EXPORT(SumU16);
std::uint32_t sum(const std::uint8_t* p, std::size_t n) {
  return HWY_DYNAMIC_DISPATCH(SumU8)(p, n);
}
std::uint32_t sum(const std::uint16_t* p, std::size_t n) {
  return HWY_DYNAMIC_DISPATCH(SumU16)(p, n);
}
} // namespace neo_mv::simd::interpolation_rows
#endif
