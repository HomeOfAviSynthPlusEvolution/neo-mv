#include "highway/interpolation_rows.hpp"
#include "highway/flow_sampling.hpp"
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
#include "highway/flow_sampling-inl.hpp"
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

template <class D, class Vec>
HWY_INLINE auto ComposeValue(D d, Vec A, Vec C, Vec X, Vec Y, Vec F, Vec B, bool extra, int time) {
  using T = hn::TFromD<D>;
  const auto full = hn::Set(d, T(256));
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
  return value;
}
template <class D, class T>
void ComposeChunk(D d, const T* a, const T* c, const T* x, const T* y, const T* mf, const T* mb, bool extra, int time,
                  T* output) {
  hn::StoreU(ComposeValue(d, hn::LoadU(d, a), hn::LoadU(d, c), hn::LoadU(d, x), hn::LoadU(d, y), hn::LoadU(d, mf),
                          hn::LoadU(d, mb), extra, time),
             d, output);
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
template <class D, class T>
HWY_INLINE auto LoadTyped(D d, const T* p, int bits) {
  const hn::Rebind<T, D> narrow;
  if constexpr (std::is_same_v<T, float>) {
    return Checked(d, hn::LoadU(d, p));
  } else {
    const auto v = hn::PromoteTo(d, hn::LoadU(narrow, p));
    if (bits < int(sizeof(T) * 8) && !hn::AllTrue(d, hn::Le(v, hn::Set(d, (1u << bits) - 1))))
      throw std::invalid_argument("interpolation sample exceeds bit depth");
    return v;
  }
}
template <class D>
HWY_INLINE auto LoadMask(D d, const std::uint8_t* p) {
  const hn::Rebind<std::uint8_t, D> bytes;
  const hn::Rebind<std::uint32_t, D> integers;
  const auto v = hn::PromoteTo(integers, hn::LoadU(bytes, p));
  if constexpr (std::is_same_v<hn::TFromD<D>, float>)
    return hn::ConvertTo(d, v);
  else
    return v;
}
template <class D, class T, class V>
HWY_INLINE void StoreTyped(D d, V v, T* out) {
  if constexpr (std::is_same_v<T, float>)
    hn::StoreU(v, d, out);
  else {
    const hn::Rebind<T, D> narrow;
    // Match scalar conversion, including modular narrowing at integer endpoints.
    hn::StoreU(hn::TruncateTo(narrow, v), narrow, out);
  }
}
template <class T>
void RenderSampled(const SampledPlane<T>& p, span2d::Plane<T> out) {
  using Lane = std::conditional_t<std::is_same_v<T, float>, float, std::uint32_t>;
  const hn::ScalableTag<Lane> d;
  const int lanes = int(hn::Lanes(d));
  const hn::CappedTag<Lane, 1> one;
  const bool extra = p.fields[2] != nullptr;
  const int width = out.width(), height = out.height();
  const neo_mv::FlowSamplingPlan plans[]{{p.left, width, height, p.time}, {p.right, width, height, 256 - p.time}};
  std::array<std::vector<T>, 4> rows;
  for (auto& row : rows)
    row.resize(width);
  FlowSampleStorage storage[2]{};
  for (int k = 0; k < 2; ++k) {
    storage[k].coordinates_validated = true;
    storage[k].output_pixel_stride = sizeof(T);
    storage[k].output_stride = std::ptrdiff_t(width) * sizeof(T);
    for (int a = 0; a < p.images[k].pel * p.images[k].pel; ++a) {
      storage[k].planes[a] = reinterpret_cast<const std::byte*>(p.images[k].planes[a].row(0).data());
      storage[k].strides[a] = p.images[k].planes[a].stride_bytes();
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int k = 0; k < (extra ? 4 : 2); ++k) {
      auto& st = storage[k % 2];
      st.output = reinterpret_cast<std::byte*>(rows[k].data());
      FlowSample<sizeof(T)>(plans[k % 2], *p.fields[k], &st, PhaseRounding::floor, y, 1);
    }
    const T* x = extra ? rows[2].data() : p.images[0].planes[0].row(y + p.left.pad_y).data() + p.left.pad_x;
    const T* z = extra ? rows[3].data() : p.images[1].planes[0].row(y + p.right.pad_y).data() + p.right.pad_x;
    const auto compose = [&](auto tag, int i) HWY_ATTR {
      const auto offset = std::size_t(y) * width + i;
      const auto v =
          ComposeValue(tag, LoadTyped(tag, rows[0].data() + i, p.bits), LoadTyped(tag, rows[1].data() + i, p.bits),
                       LoadTyped(tag, x + i, p.bits), LoadTyped(tag, z + i, p.bits), LoadMask(tag, p.masks[0] + offset),
                       LoadMask(tag, p.masks[1] + offset), extra, p.time);
      StoreTyped(tag, v, out.row(y).data() + i);
    };
    int i = 0;
    for (; i + lanes <= width; i += lanes)
      compose(d, i);
    for (; i < width; ++i)
      compose(one, i);
  }
}
#define NEO_SAMPLED_IMPL(T, S)                                                                                         \
  void Render##S(const SampledPlane<T>& p, span2d::Plane<T> out) {                                                     \
    RenderSampled(p, out);                                                                                             \
  }
NEO_SAMPLED_IMPL(std::uint8_t, U8)
NEO_SAMPLED_IMPL(std::uint16_t, U16)
NEO_SAMPLED_IMPL(float, F32)
#undef NEO_SAMPLED_IMPL

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
#define NEO_SAMPLED_EXPORT(T, S)                                                                                       \
  HWY_EXPORT(Render##S);                                                                                               \
  void render(const SampledPlane<T>& p, span2d::Plane<T> out) {                                                        \
    HWY_DYNAMIC_DISPATCH(Render##S)(p, out);                                                                           \
  }
NEO_SAMPLED_EXPORT(std::uint8_t, U8)
NEO_SAMPLED_EXPORT(std::uint16_t, U16)
NEO_SAMPLED_EXPORT(float, F32)
#undef NEO_SAMPLED_EXPORT
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
