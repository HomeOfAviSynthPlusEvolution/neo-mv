#include "highway/render_rows.hpp"
#include <stdexcept>
#include <algorithm>
#include <type_traits>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/render_rows.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"
HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::detail {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class T>
using Acc = std::conditional_t<std::is_same_v<T, float>, float, std::int32_t>;
template <class D, class V>
void CheckFinite(D d, V v) {
  if constexpr (std::is_same_v<hn::TFromD<D>, float>)
    if (!hn::AllTrue(d, hn::IsFinite(v)))
      throw std::overflow_error("non-finite render intermediate");
}
template <class D, class T>
auto LoadSample(D d, const T* p) {
  const hn::Rebind<T, D> narrow;
  if constexpr (std::is_same_v<T, float>)
    return hn::LoadU(d, p);
  else
    return hn::PromoteTo(d, hn::LoadU(narrow, p));
}
template <class D, class V, class T>
void StoreSample(D d, V v, T* p) {
  if constexpr (std::is_same_v<T, float>)
    hn::StoreU(v, d, p);
  else {
    const hn::Rebind<T, D> narrow;
    hn::StoreU(hn::DemoteTo(narrow, v), narrow, p);
  }
}
template <class D, class T>
void WeightedChunk(D d, const T* centre, const T* const* refs, const int* weights, int cw, int nr, T* out, int x) {
  using A = Acc<T>;
  auto c = LoadSample(d, centre + x);
  auto sum = hn::Mul(c, hn::Set(d, A(cw)));
  CheckFinite(d, sum);
  if constexpr (!std::is_same_v<T, float>)
    sum = hn::Add(sum, hn::Set(d, 128));
  for (int r = 0; r < nr; ++r) {
    auto sample = refs[r] ? LoadSample(d, refs[r] + x) : c;
    auto product = hn::Mul(sample, hn::Set(d, A(weights[r])));
    CheckFinite(d, product);
    sum = hn::Add(sum, product);
    CheckFinite(d, sum);
  }
  if constexpr (std::is_same_v<T, float>)
    StoreSample(d, hn::Mul(sum, hn::Set(d, 1.0f / 256)), out + x);
  else
    StoreSample(d, hn::ShiftRight<8>(sum), out + x);
}
template <class T>
void Weighted(const T* centre, const T* const* refs, const int* weights, int cw, int nr, T* out, int count) {
  const hn::ScalableTag<Acc<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    WeightedChunk(d, centre, refs, weights, cw, nr, out, x);
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    WeightedChunk(one, centre, refs, weights, cw, nr, out, x);
}
template <class D, class T>
void AddChunk(D d, const T* src, const std::uint16_t* coeff, Acc<T>* sum) {
  const hn::Rebind<std::uint16_t, D> dw;
  const hn::Rebind<std::int32_t, D> di;
  auto wi = hn::PromoteTo(di, hn::LoadU(dw, coeff));
  auto old = hn::LoadU(d, sum);
  if constexpr (std::is_same_v<T, float>) {
    auto product = hn::Mul(LoadSample(d, src), hn::ConvertTo(d, wi));
    CheckFinite(d, product);
    auto value = hn::Add(old, hn::Mul(product, hn::Set(d, 1.0f / 64)));
    CheckFinite(d, value);
    hn::StoreU(value, d, sum);
  } else
    hn::StoreU(hn::Add(old, hn::ShiftRight<6>(hn::Mul(LoadSample(d, src), wi))), d, sum);
}
template <class T>
void Add(const T* src, const std::uint16_t* coeff, Acc<T>* sum, int count) {
  const hn::ScalableTag<Acc<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    AddChunk(d, src + x, coeff + x, sum + x);
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    AddChunk(one, src + x, coeff + x, sum + x);
}
template <class D, class T>
void FinishChunk(D d, const Acc<T>* sum, T* out, std::int64_t maximum) {
  auto v = hn::LoadU(d, sum);
  if constexpr (std::is_same_v<T, float>)
    StoreSample(d, hn::Mul(v, hn::Set(d, 1.0f / 32)), out);
  else
    StoreSample(d, hn::Min(hn::ShiftRight<5>(hn::Add(v, hn::Set(d, 16))), hn::Set(d, std::int32_t(maximum))), out);
}
template <class T>
void Finish(const Acc<T>* sum, T* out, int count, std::int64_t maximum) {
  const hn::ScalableTag<Acc<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    FinishChunk(d, sum + x, out + x, maximum);
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    FinishChunk(one, sum + x, out + x, maximum);
}
template <class D, class T>
void LimitChunk(D d, const T* q, const T* centre, T* out, std::int64_t maximum, std::int64_t limit, float flimit) {
  const auto c = LoadSample(d, centre), v = LoadSample(d, q);
  if constexpr (std::is_same_v<T, float>) {
    const auto lo = hn::Sub(c, hn::Set(d, flimit)), hi = hn::Add(c, hn::Set(d, flimit));
    CheckFinite(d, lo);
    CheckFinite(d, hi);
    // Ordered comparisons preserve the input zero's sign when it is in range.
    StoreSample(d, hn::IfThenElse(hn::Lt(v, lo), lo, hn::IfThenElse(hn::Gt(v, hi), hi, v)), out);
  } else {
    const auto lo = hn::Max(hn::Sub(c, hn::Set(d, std::int32_t(limit))), hn::Zero(d));
    const auto hi = hn::Min(hn::Add(c, hn::Set(d, std::int32_t(limit))), hn::Set(d, std::int32_t(maximum)));
    StoreSample(d, hn::Min(hi, hn::Max(lo, v)), out);
  }
}
template <class T>
void Limit(const T* q, const T* centre, T* out, int count, bool active, std::int64_t maximum, std::int64_t limit,
           float flimit) {
  if (!active) {
    const hn::ScalableTag<T> d;
    const int n = int(hn::Lanes(d));
    int x = 0;
    for (; x <= count - n; x += n)
      hn::StoreU(hn::LoadU(d, q + x), d, out + x);
    for (; x < count; ++x)
      out[x] = q[x];
    return;
  }
  if constexpr (!std::is_same_v<T, float>) {
    const hn::ScalableTag<T> d;
    const int n = int(hn::Lanes(d));
    int x = 0;
    const auto amount = hn::Set(d, T(limit)), max = hn::Set(d, T(maximum));
    for (; x <= count - n; x += n) {
      const auto c = hn::LoadU(d, centre + x), v = hn::LoadU(d, q + x);
      const auto lo = hn::SaturatedSub(c, amount), hi = hn::Min(max, hn::SaturatedAdd(c, amount));
      hn::StoreU(hn::Min(hi, hn::Max(lo, v)), d, out + x);
    }
    for (; x < count; ++x) {
      const auto lo = std::max<std::int64_t>(0, std::int64_t(centre[x]) - limit);
      const auto hi = std::min(maximum, std::int64_t(centre[x]) + limit);
      out[x] = T(std::min(hi, std::max(lo, std::int64_t(q[x]))));
    }
  } else {
    const hn::ScalableTag<float> d;
    const int n = int(hn::Lanes(d));
    int x = 0;
    for (; x <= count - n; x += n)
      LimitChunk(d, q + x, centre + x, out + x, maximum, limit, flimit);
    const hn::CappedTag<float, 1> one;
    for (; x < count; ++x)
      LimitChunk(one, q + x, centre + x, out + x, maximum, limit, flimit);
  }
}
#define NEO_IMPL(T, A, S)                                                                                              \
  void Weighted##S(const T* c, const T* const* r, const int* w, int cw, int nr, T* o, int n) {                         \
    Weighted(c, r, w, cw, nr, o, n);                                                                                   \
  }                                                                                                                    \
  void Add##S(const T* p, const std::uint16_t* w, A* a, int n) {                                                       \
    Add(p, w, a, n);                                                                                                   \
  }                                                                                                                    \
  void Finish##S(const A* a, T* o, int n, std::int64_t m) {                                                            \
    Finish(a, o, n, m);                                                                                                \
  }                                                                                                                    \
  void Limit##S(const T* q, const T* c, T* o, int n, bool active, std::int64_t m, std::int64_t l, float f) {           \
    Limit(q, c, o, n, active, m, l, f);                                                                                \
  }
NEO_IMPL(std::uint8_t, std::int32_t, U8)
NEO_IMPL(std::uint16_t, std::int32_t, U16)
NEO_IMPL(float, float, F32)
#undef NEO_IMPL
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::detail
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::detail {
#define NEO_EXPORT(T, A, S)                                                                                            \
  HWY_EXPORT(Weighted##S);                                                                                             \
  HWY_EXPORT(Add##S);                                                                                                  \
  HWY_EXPORT(Finish##S);                                                                                               \
  HWY_EXPORT(Limit##S);                                                                                                \
  void weighted(const T* c, const T* const* r, const int* w, int cw, int nr, T* o, int n) {                            \
    HWY_DYNAMIC_DISPATCH(Weighted##S)(c, r, w, cw, nr, o, n);                                                          \
  }                                                                                                                    \
  void overlap_add(const T* p, const std::uint16_t* w, A* a, int n) {                                                  \
    HWY_DYNAMIC_DISPATCH(Add##S)(p, w, a, n);                                                                          \
  }                                                                                                                    \
  void overlap_finish(const A* a, T* o, int n, std::int64_t m) {                                                       \
    HWY_DYNAMIC_DISPATCH(Finish##S)(a, o, n, m);                                                                       \
  }                                                                                                                    \
  void change_limit(const T* q, const T* c, T* o, int n, bool active, std::int64_t m, std::int64_t l, float f) {       \
    HWY_DYNAMIC_DISPATCH(Limit##S)(q, c, o, n, active, m, l, f);                                                       \
  }
NEO_EXPORT(std::uint8_t, std::int32_t, U8)
NEO_EXPORT(std::uint16_t, std::int32_t, U16)
NEO_EXPORT(float, float, F32)
#undef NEO_EXPORT
} // namespace neo_mv::simd::detail
#endif
