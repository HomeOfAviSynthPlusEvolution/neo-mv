#include "core/motion/dct_fft.hpp"

#if NEO_MV_DCT_SIMD
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "core/motion/dct_transform.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::dct_detail {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
struct VectorOps {
  const hn::CappedTag<float, 8> d;
  int lanes() const { return int(hn::Lanes(d)); }
  using V = hn::VFromD<decltype(d)>;
  V load(const float* p) const { return hn::LoadU(d, p); }
  void store(V v, float* p) const { hn::StoreU(v, d, p); }
  V set(float v) const { return hn::Set(d, v); }
  V zero() const { return hn::Zero(d); }
  V add(V a, V b) const { return hn::Add(a, b); }
  V sub(V a, V b) const { return hn::Sub(a, b); }
  V mul(V a, V b) const { return hn::Mul(a, b); }
  V neg(V a) const { return hn::Neg(a); }
  HWY_INLINE void transpose(const float* in, int w, int h, int stride, float* out, int out_stride) const {
    int tile = 1;
// Fixed-width SVE targets still use sizeless vector types, which cannot be
// array elements. Keep this array-based tile on non-SVE fixed-width targets.
#if HWY_MAX_BYTES >= 32 && !HWY_HAVE_SCALABLE && !(HWY_TARGET & HWY_ALL_SVE)
    tile = 8;
    const hn::Repartition<std::uint64_t, decltype(d)> d64;
    for (int y = 0; y < h; y += 8) {
      for (int x = 0; x < w; x += 8) {
        V r[8], a[8], b[8];
        for (int i = 0; i < 8; ++i)
          r[i] = y + i < h ? load(in + (y + i) * stride + x) : zero();
        for (int i = 0; i < 4; ++i) {
          a[2 * i] = hn::InterleaveLower(d, r[2 * i], r[2 * i + 1]);
          a[2 * i + 1] = hn::InterleaveUpper(d, r[2 * i], r[2 * i + 1]);
        }
        for (int j = 0; j < 8; j += 4) {
          b[j] = hn::BitCast(d, hn::InterleaveLower(d64, hn::BitCast(d64, a[j]), hn::BitCast(d64, a[j + 2])));
          b[j + 1] = hn::BitCast(d, hn::InterleaveUpper(d64, hn::BitCast(d64, a[j]), hn::BitCast(d64, a[j + 2])));
          b[j + 2] = hn::BitCast(d, hn::InterleaveLower(d64, hn::BitCast(d64, a[j + 1]), hn::BitCast(d64, a[j + 3])));
          b[j + 3] = hn::BitCast(d, hn::InterleaveUpper(d64, hn::BitCast(d64, a[j + 1]), hn::BitCast(d64, a[j + 3])));
        }
        for (int i = 0; i < 4; ++i) {
          if (x + i < w)
            store(hn::ConcatLowerLower(d, b[i + 4], b[i]), out + (x + i) * out_stride + y);
          if (x + i + 4 < w)
            store(hn::ConcatUpperUpper(d, b[i + 4], b[i]), out + (x + i + 4) * out_stride + y);
        }
      }
    }
    return;
#elif HWY_TARGET != HWY_SCALAR
    const hn::CappedTag<float, 4> d4;
    const hn::Repartition<std::uint64_t, decltype(d4)> d64;
    if (hn::Lanes(d4) == 4) {
      tile = 4;
      for (int y = 0; y + 4 <= h; y += 4)
        for (int x = 0; x + 4 <= w; x += 4) {
          const auto r0 = hn::LoadU(d4, in + y * stride + x), r1 = hn::LoadU(d4, in + (y + 1) * stride + x);
          const auto r2 = hn::LoadU(d4, in + (y + 2) * stride + x), r3 = hn::LoadU(d4, in + (y + 3) * stride + x);
          const auto a0 = hn::InterleaveLower(d4, r0, r1), a1 = hn::InterleaveUpper(d4, r0, r1);
          const auto a2 = hn::InterleaveLower(d4, r2, r3), a3 = hn::InterleaveUpper(d4, r2, r3);
          hn::StoreU(hn::BitCast(d4, hn::InterleaveLower(d64, hn::BitCast(d64, a0), hn::BitCast(d64, a2))), d4,
                     out + x * out_stride + y);
          hn::StoreU(hn::BitCast(d4, hn::InterleaveUpper(d64, hn::BitCast(d64, a0), hn::BitCast(d64, a2))), d4,
                     out + (x + 1) * out_stride + y);
          hn::StoreU(hn::BitCast(d4, hn::InterleaveLower(d64, hn::BitCast(d64, a1), hn::BitCast(d64, a3))), d4,
                     out + (x + 2) * out_stride + y);
          hn::StoreU(hn::BitCast(d4, hn::InterleaveUpper(d64, hn::BitCast(d64, a1), hn::BitCast(d64, a3))), d4,
                     out + (x + 3) * out_stride + y);
        }
    }
#endif
    const int full_w = tile == 1 ? 0 : w / tile * tile;
    const int full_h = tile == 1 ? 0 : h / tile * tile;
    for (int y = 0; y < h; ++y)
      for (int x = (y < full_h ? full_w : 0); x < w; ++x)
        out[x * out_stride + y] = in[y * stride + x];
    for (int x = 0; x < w; ++x)
      std::fill(out + x * out_stride + h, out + (x + 1) * out_stride, 0.0f);
  }
};
#define NEO_MV_DCT_INLINE HWY_INLINE
#include "core/motion/dct_float-inl.hpp"
#undef NEO_MV_DCT_INLINE
void TransformNative(int w, int h, const float* input, float* rows, float* output) {
  Transform(VectorOps{}, w, h, input, rows, output);
}
void QuantizeNative(int w, int h, const float* input, int* output, int maximum) {
  const hn::CappedTag<float, 8> d;
  const hn::Rebind<std::int32_t, decltype(d)> di;
  const int lanes = int(hn::Lanes(d)), stride = padded_stride(w);
  const auto half = hn::Set(d, 0.5f);
  const auto one = hn::Set(di, 1);
  const auto midpoint = hn::Set(di, (maximum + 1) / 2), upper = hn::Set(di, maximum);
  for (int y = 0; y < h; ++y) {
    int x = y == 0 ? 1 : 0;
    for (; x < w; x += lanes) {
      const int count = std::min(lanes, w - x);
      const auto value =
          count == lanes ? hn::LoadU(d, input + y * stride + x) : hn::LoadN(d, input + y * stride + x, count);
      const auto low = hn::Floor(value);
      const auto base = hn::ConvertTo(di, low);
      const auto fraction = hn::Sub(value, low);
      const auto odd = hn::RebindMask(d, hn::Eq(hn::And(base, one), one));
      const auto inc = hn::Or(hn::Gt(fraction, half), hn::And(hn::Eq(fraction, half), odd));
      const auto q = hn::Add(base, hn::IfThenElseZero(hn::RebindMask(di, inc), one));
      const auto result = hn::Min(upper, hn::Max(hn::Zero(di), hn::Add(q, midpoint)));
      if (count == lanes)
        hn::StoreU(result, di, output + y * w + x);
      else
        hn::StoreN(result, di, output + y * w + x, count);
    }
  }
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::dct_detail
HWY_AFTER_NAMESPACE();
#endif

#if !NEO_MV_DCT_SIMD || HWY_ONCE
namespace neo_mv::dct_detail {
namespace scalar {
struct ScalarOps {
  static constexpr int lanes() { return 1; }
  float load(const float* p) const { return *p; }
  void store(float v, float* p) const { *p = v; }
  float set(float v) const { return v; }
  float zero() const { return 0; }
  float add(float a, float b) const { return a + b; }
  float sub(float a, float b) const { return a - b; }
  float mul(float a, float b) const { return a * b; }
  float neg(float a) const { return -a; }
  void transpose(const float* in, int w, int h, int stride, float* out, int out_stride) const {
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y)
        out[x * out_stride + y] = in[y * stride + x];
      std::fill(out + x * out_stride + h, out + (x + 1) * out_stride, 0.0f);
    }
  }
};
#define NEO_MV_DCT_INLINE inline
#include "core/motion/dct_float-inl.hpp"
#undef NEO_MV_DCT_INLINE
} // namespace scalar
#if NEO_MV_DCT_SIMD
HWY_EXPORT(TransformNative);
HWY_EXPORT(QuantizeNative);
#endif
void transform_block(int w, int h, const float* input, float* rows, float* output, bool simd) {
#if NEO_MV_DCT_SIMD
  if (simd)
    return HWY_DYNAMIC_DISPATCH(TransformNative)(w, h, input, rows, output);
#else
  (void)simd;
#endif
  scalar::Transform(scalar::ScalarOps{}, w, h, input, rows, output);
}
void quantize_ac(int w, int h, const float* input, int* output, int maximum, bool simd) {
#if NEO_MV_DCT_SIMD
  if (simd)
    return HWY_DYNAMIC_DISPATCH(QuantizeNative)(w, h, input, output, maximum);
#else
  (void)simd;
#endif
  const int stride = padded_stride(w);
  for (int y = 0; y < h; ++y)
    for (int x = y == 0 ? 1 : 0; x < w; ++x)
      output[y * w + x] = std::clamp(round_even(input[y * stride + x]) + (maximum + 1) / 2, 0, maximum);
}
} // namespace neo_mv::dct_detail
#endif
