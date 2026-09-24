#include "highway/rows.hpp"
#include "core/motion/block_metric.hpp"
#include "core/motion/analyse.hpp"
#include <algorithm>
#include <limits>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/rows.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"
HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::detail {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
std::int64_t Target() {
  return HWY_TARGET;
}
template <class T> using Wide = std::conditional_t<std::is_same_v<T, float>, float, std::int32_t>;
template <class T> using FormulaWide = std::conditional_t<std::is_same_v<T, std::uint8_t>, std::int16_t, Wide<T>>;
template <class D, class V> HWY_INLINE void finite(D d, V v) {
  if constexpr (std::is_same_v<hn::TFromD<D>, float>)
    if (!hn::AllTrue(d, hn::IsFinite(v)))
      throw std::invalid_argument("non-finite SIMD sample/intermediate");
}
template <class T> void Extract(const T *p, T *out, int count, int pel, int phase) {
  const hn::ScalableTag<T> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n) {
    auto a = hn::Zero(d), b = a, c = a, e = a;
    if (pel == 2)
      hn::LoadInterleaved2(d, p + pel * x, a, b);
    else
      hn::LoadInterleaved4(d, p + pel * x, a, b, c, e);
    hn::StoreU(phase == 0 ? a : phase == 1 ? b : phase == 2 ? c : e, d, out + x);
  }
  for (; x < count; ++x)
    out[x] = p[pel * x + phase];
}
template <class T> void Scan(const T *p, int count, std::int64_t maximum) {
  const hn::ScalableTag<T> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n) {
    auto v = hn::LoadU(d, p + x);
    if constexpr (std::is_same_v<T, float>)
      finite(d, v);
    else if (!hn::AllTrue(d, hn::Le(v, hn::Set(d, T(maximum)))))
      throw std::invalid_argument("sample exceeds bit depth");
  }
  for (; x < count; ++x) {
    if constexpr (std::is_same_v<T, float>) {
      if (!std::isfinite(p[x]))
        throw std::invalid_argument("non-finite sample");
    } else if (p[x] > maximum)
      throw std::invalid_argument("sample exceeds bit depth");
  }
}
template <class T> void Copy(const T *p, T *q, int count) {
  const hn::ScalableTag<T> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    hn::StoreU(hn::LoadU(d, p + x), d, q + x);
  for (; x < count; ++x)
    q[x] = p[x];
}
template <class T> void Fill(T v, T *q, int count) {
  const hn::ScalableTag<T> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    hn::StoreU(hn::Set(d, v), d, q + x);
  for (; x < count; ++x)
    q[x] = v;
}
template <int Step, class T, class D> auto LoadWide(D d, const T *p) {
  const hn::Rebind<T, D> narrow;
  auto v = hn::Zero(narrow);
  if constexpr (Step == 1)
    v = hn::LoadU(narrow, p);
  else {
    auto unused = v;
    hn::LoadInterleaved2(narrow, p, v, unused);
  }
  if constexpr (std::is_same_v<T, float>)
    return v;
  else
    return hn::PromoteTo(d, v);
}
template <class D, class V> HWY_INLINE auto Calculate(D d, V a, V b, V c, V e, V f, V g, Formula op, std::int64_t maximum) {
  using A = hn::TFromD<D>;
  auto v = a;
  int shift = 1;
  switch (op) {
  case Formula::average:
    v = hn::Add(a, b);
    break;
  case Formula::four_average:
    v = hn::Add(hn::Add(hn::Add(a, b), c), e);
    shift = 2;
    break;
  case Formula::reduce4:
    v = hn::Add(hn::Add(a, hn::Mul(hn::Set(d, A(3)), hn::Add(b, c))), e);
    shift = 3;
    break;
  case Formula::reduce6:
    v = hn::Add(
        a, hn::Add(hn::Add(g, hn::Mul(hn::Set(d, A(10)), hn::Add(c, e))), hn::Mul(hn::Set(d, A(5)), hn::Add(b, f))));
    shift = 5;
    break;
  case Formula::sharp4h:
  case Formula::sharp4v:
    if constexpr (std::is_same_v<A, float>) {
      auto ends = op == Formula::sharp4v ? hn::Sub(hn::Neg(a), e) : hn::Neg(hn::Add(a, e));
      v = hn::Add(ends, hn::Mul(hn::Set(d, A(9)), hn::Add(b, c)));
    } else
      v = hn::Sub(hn::Sub(hn::Mul(hn::Set(d, A(9)), hn::Add(b, c)), a), e);
    shift = 4;
    break;
  case Formula::sharp6:
    if constexpr (std::is_same_v<A, float>)
      v = hn::Add(
          a, hn::Add(g, hn::Mul(hn::Set(d, A(5)), hn::Sub(hn::Mul(hn::Set(d, A(4)), hn::Add(c, e)), hn::Add(b, f)))));
    else
      v = hn::Sub(hn::Add(hn::Add(a, g), hn::Mul(hn::Set(d, A(20)), hn::Add(c, e))),
                  hn::Mul(hn::Set(d, A(5)), hn::Add(b, f)));
    shift = 5;
    break;
  }
  if constexpr (std::is_same_v<A, float>) {
    finite(d, v);
    return hn::Mul(v, hn::Set(d, 1.0f / float(1 << shift)));
  } else
    return hn::Min(hn::Max(hn::ShiftRightSame(hn::Add(v, hn::Set(d, 1 << (shift - 1))), shift), hn::Zero(d)),
                   hn::Set(d, static_cast<A>(maximum)));
}
template <Formula Op, int Step, class T, class D>
HWY_INLINE void FormulaChunk(D d, const T *const *p, T *out, std::int64_t maximum) {
  const hn::Rebind<T, D> narrow;
  auto a = LoadWide<Step>(d, p[0]), b = LoadWide<Step>(d, p[1]), c = hn::Zero(d), e = c, f = c, g = c;
  if constexpr (Op != Formula::average) {
    c = LoadWide<Step>(d, p[2]);
    e = LoadWide<Step>(d, p[3]);
  }
  if constexpr (Op == Formula::reduce6 || Op == Formula::sharp6) {
    f = LoadWide<Step>(d, p[4]);
    g = LoadWide<Step>(d, p[5]);
  }
  auto v = Calculate(d, a, b, c, e, f, g, Op, maximum);
  if constexpr (std::is_same_v<T, float>)
    hn::StoreU(v, narrow, out);
  else
    hn::StoreU(hn::DemoteTo(narrow, v), narrow, out);
}
template <class D>
HWY_INLINE auto LoadBiasedU16(D d, const std::uint16_t* p) {
  const hn::RebindToUnsigned<D> du;
  return hn::BitCast(d, hn::Xor(hn::LoadU(du, p), hn::Set(du, 32768)));
}

template <Formula Op, int Step, class T>
void FormulaRowKnownStep(const T *const *p, T *out, int count, std::int64_t maximum) {
  const hn::ScalableTag<FormulaWide<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  constexpr int nt = Op == Formula::average ? 2 : ((Op == Formula::reduce6 || Op == Formula::sharp6) ? 6 : 4);
  const T *taps[6]{};
#if HWY_ARCH_X86 && HWY_TARGET != HWY_SCALAR && HWY_TARGET != HWY_EMU128
  if constexpr (std::is_same_v<T, std::uint16_t> && Op == Formula::sharp6 && Step == 1) {
    const hn::ScalableTag<std::int16_t> d16;
    const hn::RebindToUnsigned<decltype(d16)> du16;
    const hn::Repartition<std::int32_t, decltype(d16)> d32;
    const int packed_lanes = int(hn::Lanes(d16));
    const auto bias = hn::Set(du16, 32768);
    const auto c1 = hn::Set(d16, 1), cm5 = hn::Set(d16, -5), c20 = hn::Set(d16, 20);
    const auto rounding = hn::Set(d32, 16), limit = hn::Set(d32, int(maximum) - 32768);
    for (; x <= count - packed_lanes; x += packed_lanes) {
      const auto a = LoadBiasedU16(d16, p[0] + x), b = LoadBiasedU16(d16, p[1] + x), c = LoadBiasedU16(d16, p[2] + x);
      const auto e = LoadBiasedU16(d16, p[3] + x), f = LoadBiasedU16(d16, p[4] + x), g = LoadBiasedU16(d16, p[5] + x);
      const auto lo = hn::Add(hn::Add(hn::WidenMulPairwiseAdd(d32, hn::InterleaveLower(d16, a, g), c1),
                                     hn::WidenMulPairwiseAdd(d32, hn::InterleaveLower(d16, b, f), cm5)),
                              hn::WidenMulPairwiseAdd(d32, hn::InterleaveLower(d16, c, e), c20));
      const auto hi = hn::Add(hn::Add(hn::WidenMulPairwiseAdd(d32, hn::InterleaveUpper(d16, a, g), c1),
                                     hn::WidenMulPairwiseAdd(d32, hn::InterleaveUpper(d16, b, f), cm5)),
                              hn::WidenMulPairwiseAdd(d32, hn::InterleaveUpper(d16, c, e), c20));
      // The coefficients sum to 32: signed input bias survives the rounded
      // shift unchanged. Signed saturation clamps the lower output endpoint.
      const auto value = hn::ReorderDemote2To(d16, hn::Min(hn::ShiftRight<5>(hn::Add(lo, rounding)), limit),
                                             hn::Min(hn::ShiftRight<5>(hn::Add(hi, rounding)), limit));
      hn::StoreU(hn::Xor(hn::BitCast(du16, value), bias), du16, out + x);
    }
  }
#endif
  for (; x <= count - n; x += n) {
    for (int j = 0; j < nt; ++j)
      taps[j] = p[j] + x * Step;
    FormulaChunk<Op, Step>(d, taps, out + x, maximum);
  }
  const hn::CappedTag<FormulaWide<T>, 1> one;
  for (; x < count; ++x) {
    for (int j = 0; j < nt; ++j)
      taps[j] = p[j] + x * Step;
    FormulaChunk<Op, 1>(one, taps, out + x, maximum);
  }
}
template <Formula Op, class T>
void FormulaRowKnown(const T *const *p, int step, T *out, int count, std::int64_t maximum) {
  if (step == 1)
    FormulaRowKnownStep<Op, 1>(p, out, count, maximum);
  else
    FormulaRowKnownStep<Op, 2>(p, out, count, maximum);
}
template <class T>
void FormulaRow(const T *const *p, int step, T *out, int count, Formula op, std::int64_t maximum) {
  // Select once per row so tap count, coefficients and shifts are constants
  // throughout the vector loop, including its scalar tail.
  switch (op) {
#define NEO_FORMULA_CASE(NAME) \
  case Formula::NAME: FormulaRowKnown<Formula::NAME>(p, step, out, count, maximum); return;
    NEO_FORMULA_CASE(average)
    NEO_FORMULA_CASE(four_average)
    NEO_FORMULA_CASE(reduce4)
    NEO_FORMULA_CASE(reduce6)
    NEO_FORMULA_CASE(sharp4h)
    NEO_FORMULA_CASE(sharp4v)
    NEO_FORMULA_CASE(sharp6)
#undef NEO_FORMULA_CASE
  }
}

template <class V> void Hadamard(V &a, V &b, V &c, V &d) {
  const auto ab = hn::Add(a, b), cd = hn::Add(c, d), am = hn::Sub(a, b), cm = hn::Sub(c, d);
  a = hn::Add(ab, cd);
  b = hn::Add(am, cm);
  c = hn::Sub(ab, cd);
  d = hn::Sub(am, cm);
}
// Motion blocks fit this bounded fast path. The entire u16 SAD is at most
// 128 * 128 * 65535 < INT32_MAX, including all lanes and scalar tails.
// Larger public metric rectangles keep the checked accumulation below.
template <class T, class D>
std::int64_t SmallSad(D d, const T *a, std::ptrdiff_t as, const T *b, std::ptrdiff_t bs, int w, int h) {
  const int n = int(hn::Lanes(d));
  auto sum = hn::Zero(d);
  std::int64_t tail = 0;
  for (int y = 0; y < h; ++y) {
    int x = 0;
    for (; x <= w - n; x += n) {
      if constexpr (std::is_same_v<T, std::uint16_t>) {
        const hn::Rebind<T, D> narrow;
        const auto difference = hn::AbsDiff(hn::LoadU(narrow, a + x), hn::LoadU(narrow, b + x));
        sum = hn::Add(sum, hn::PromoteTo(d, difference));
      } else {
        sum = hn::Add(sum, hn::Abs(hn::Sub(LoadWide<1>(d, a + x), LoadWide<1>(d, b + x))));
      }
    }
    for (; x < w; ++x)
      tail += std::abs(int(a[x]) - int(b[x]));
    if (y + 1 < h) {
      a += as;
      b += bs;
    }
  }
  return tail + hn::ReduceSum(d, sum);
}

#if HWY_TARGET != HWY_SCALAR
template <class D>
HWY_INLINE auto ShortSadPairs(D narrow, const std::uint16_t *a, const std::uint16_t *b) {
  const hn::RebindToSigned<D> signed_narrow;
  const hn::Repartition<std::int32_t, D> d;
  const auto va = hn::LoadU(narrow, a), vb = hn::LoadU(narrow, b);
#if HWY_ARCH_X86 && HWY_TARGET <= HWY_AVX3
  // At most one saturated difference is nonzero. AVX512 can combine the OR
  // and the following bias XOR in one ternary operation, avoiding the
  // min/max/subtract dependency used by the generic unsigned absolute diff.
  const auto diff = hn::Or(hn::SaturatedSub(va, vb), hn::SaturatedSub(vb, va));
#else
  const auto diff = hn::AbsDiff(va, vb);
#endif
  // XOR maps the entire unsigned difference range to d - 32768.
  // Pairwise signed multiply-add is exact; restore the prefix bias below.
  return hn::WidenMulPairwiseAdd(d, hn::BitCast(signed_narrow, hn::Xor(diff, hn::Set(narrow, 0x8000))),
                                hn::Set(signed_narrow, 1));
}

template <int Width, int Height, bool Bounded = false>
HWY_INLINE std::int64_t FixedShortSad(const std::uint16_t *a, std::ptrdiff_t as,
                                     const std::uint16_t *b, std::ptrdiff_t bs,
                                     std::int64_t limit = INT64_MAX) {
  const hn::CappedTag<std::uint16_t, Width> narrow;
  const hn::Repartition<std::int32_t, decltype(narrow)> d;
  const int lanes = int(hn::Lanes(narrow));
  auto sum0 = hn::Zero(d), sum1 = sum0, sum2 = sum0, sum3 = sum0;
  std::int64_t total = 0;
  for (int y = 0; y < Height; y += 4) {
    const auto* ar = a + y * as;
    const auto* br = b + y * bs;
    for (int x = 0; x < Width; x += lanes) {
      sum0 = hn::Add(sum0, ShortSadPairs(narrow, ar + x, br + x));
      sum1 = hn::Add(sum1, ShortSadPairs(narrow, ar + as + x, br + bs + x));
      sum2 = hn::Add(sum2, ShortSadPairs(narrow, ar + 2 * as + x, br + 2 * bs + x));
      sum3 = hn::Add(sum3, ShortSadPairs(narrow, ar + 3 * as + x, br + 3 * bs + x));
    }
    if constexpr (Bounded) {
      // For 16-row blocks, skip the first reduction: check at 8/12/16.
      if (Height != 16 || y != 0) {
        total = hn::ReduceSum(d, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3))) +
                std::int64_t(y + 4) * Width * 32768;
        if (total >= limit)
          return total;
      }
    }
  }
  if constexpr (Bounded)
    return total;
  else
    return hn::ReduceSum(d, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3))) +
           std::int64_t(Height) * Width * 32768;
}

template <int Width, int Height, bool Bounded = false, bool LateCheck = false>
HWY_INLINE std::int64_t FixedByteSad(const std::uint8_t *a, std::ptrdiff_t as, const std::uint8_t *b, std::ptrdiff_t bs,
                                    std::int64_t limit = INT64_MAX) {
  const hn::CappedTag<std::uint8_t, Width> d;
  const hn::Repartition<std::uint64_t, decltype(d)> wide;
  auto sum0 = hn::Zero(wide), sum1 = sum0, sum2 = sum0, sum3 = sum0;
  std::int64_t total = 0;
  for (int y = 0; y < Height; y += 4) {
    const auto* ar = a + y * as;
    const auto* br = b + y * bs;
    sum0 = hn::Add(sum0, hn::SumsOf8AbsDiff(hn::LoadU(d, ar), hn::LoadU(d, br)));
    sum1 = hn::Add(sum1, hn::SumsOf8AbsDiff(hn::LoadU(d, ar + as), hn::LoadU(d, br + bs)));
    sum2 = hn::Add(sum2, hn::SumsOf8AbsDiff(hn::LoadU(d, ar + 2 * as), hn::LoadU(d, br + 2 * bs)));
    sum3 = hn::Add(sum3, hn::SumsOf8AbsDiff(hn::LoadU(d, ar + 3 * as), hn::LoadU(d, br + 3 * bs)));
    if constexpr (Bounded) {
      if (Height != 16 || (LateCheck ? y >= 8 : y != 0)) {
        total = static_cast<std::int64_t>(hn::ReduceSum(wide, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3))));
        if (total >= limit)
          return total;
      }
    }
  }
  if constexpr (Bounded)
    return total;
  else
    return static_cast<std::int64_t>(hn::ReduceSum(wide, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3))));
}
HWY_INLINE std::int64_t FixedByteSad4(const std::uint8_t *a, std::ptrdiff_t as, const std::uint8_t *b, std::ptrdiff_t bs) {
  const hn::CappedTag<std::uint8_t, 16> d;
  const hn::Repartition<std::uint64_t, decltype(d)> wide;
  // Read exactly four bytes per row; packing allows one SAD for the block.
  HWY_ALIGN std::uint8_t packed_a[16], packed_b[16];
  for (int y = 0; y < 4; ++y) {
    hwy::CopyBytes<4>(a + y * as, packed_a + 4 * y);
    hwy::CopyBytes<4>(b + y * bs, packed_b + 4 * y);
  }
  return static_cast<std::int64_t>(hn::ReduceSum(wide,
      hn::SumsOf8AbsDiff(hn::LoadU(d, packed_a), hn::LoadU(d, packed_b))));
}
template <class D>
std::int64_t SmallByteSad(D d, const std::uint8_t *a, std::ptrdiff_t as, const std::uint8_t *b,
                          std::ptrdiff_t bs, int w, int h) {
  const hn::Repartition<std::uint64_t, D> wide;
  const int n = int(hn::Lanes(d));
  auto sum0 = hn::Zero(wide), sum1 = sum0, sum2 = sum0, sum3 = sum0;
  std::int64_t tail = 0;
  int y = 0;
  for (; y + 3 < h; y += 4) {
    int x = 0;
    for (; x <= w - n; x += n) {
      sum0 = hn::Add(sum0, hn::SumsOf8AbsDiff(hn::LoadU(d, a + x), hn::LoadU(d, b + x)));
      sum1 = hn::Add(sum1, hn::SumsOf8AbsDiff(hn::LoadU(d, a + as + x), hn::LoadU(d, b + bs + x)));
      sum2 = hn::Add(sum2, hn::SumsOf8AbsDiff(hn::LoadU(d, a + 2 * as + x), hn::LoadU(d, b + 2 * bs + x)));
      sum3 = hn::Add(sum3, hn::SumsOf8AbsDiff(hn::LoadU(d, a + 3 * as + x), hn::LoadU(d, b + 3 * bs + x)));
    }
    for (; x < w; ++x) {
      tail += std::abs(int(a[x]) - int(b[x]));
      tail += std::abs(int(a[as + x]) - int(b[bs + x]));
      tail += std::abs(int(a[2 * as + x]) - int(b[2 * bs + x]));
      tail += std::abs(int(a[3 * as + x]) - int(b[3 * bs + x]));
    }
    if (y + 4 < h) {
      a += 4 * as;
      b += 4 * bs;
    }
  }
  for (; y < h; ++y) {
    int x = 0;
    for (; x <= w - n; x += n)
      sum0 = hn::Add(sum0, hn::SumsOf8AbsDiff(hn::LoadU(d, a + x), hn::LoadU(d, b + x)));
    for (; x < w; ++x)
      tail += std::abs(int(a[x]) - int(b[x]));
    if (y + 1 < h) {
      a += as;
      b += bs;
    }
  }
  return tail + static_cast<std::int64_t>(hn::ReduceSum(wide, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3))));
}
#endif

#if HWY_TARGET != HWY_SCALAR
template <class D, class V> HWY_INLINE V ByteSatdHorizontal(D d, V row) {
  const hn::Repartition<std::int32_t, D> pairs;
  const auto adjacent = hn::Reverse2(d, row);
  row = hn::OddEven(hn::Sub(row, adjacent), hn::Add(row, adjacent));
  const auto opposite = hn::Reverse4(d, hn::Reverse2(d, row));
  return hn::BitCast(d, hn::OddEven(hn::BitCast(pairs, hn::Sub(row, opposite)),
                                   hn::BitCast(pairs, hn::Add(row, opposite))));
}

template <class D>
HWY_INLINE auto ByteSatdRow(D d, const std::uint8_t* a, const std::uint8_t* b) {
  const hn::Rebind<std::uint8_t, D> bytes;
  return ByteSatdHorizontal(d, hn::Sub(hn::PromoteTo(d, hn::LoadU(bytes, a)),
                                      hn::PromoteTo(d, hn::LoadU(bytes, b))));
}

// Keep adjacent pixels together instead of deinterleaving four byte streams.
// A byte residual is at most 255, a 4x4 coefficient at most 4080, and
// four absolute coefficients sum to at most 16320: signed 16-bit is enough.
// Widen each pair before accumulating across cells. For a 128x128 block,
// even the conservative bound 128*128*4080 fits signed 32-bit.
template <class D>
std::int64_t ByteSatd(D d, const std::uint8_t *a, std::ptrdiff_t as,
                      const std::uint8_t *b, std::ptrdiff_t bs, int w, int h) {
  const hn::Repartition<std::int32_t, D> wide;
  const int n = int(hn::Lanes(d));
  const int end = w / n * n;
  auto sum = hn::Zero(wide);
  for (int y = 0; y < h; y += 4) {
    for (int x = 0; x < end; x += n) {
      auto r0 = ByteSatdRow(d, a + y * as + x, b + y * bs + x);
      auto r1 = ByteSatdRow(d, a + (y + 1) * as + x, b + (y + 1) * bs + x);
      auto r2 = ByteSatdRow(d, a + (y + 2) * as + x, b + (y + 2) * bs + x);
      auto r3 = ByteSatdRow(d, a + (y + 3) * as + x, b + (y + 3) * bs + x);
      Hadamard(r0, r1, r2, r3);
      const auto absolute = hn::Add(hn::Add(hn::Abs(r0), hn::Abs(r1)), hn::Add(hn::Abs(r2), hn::Abs(r3)));
      sum = hn::Add(sum, hn::WidenMulPairwiseAdd(wide, absolute, hn::Set(d, 1)));
    }
  }
  // Every integer 4x4 cell has an even absolute sum, so division may be
  // deferred until after the vector reduction without changing rounding.
  std::int64_t total = hn::ReduceSum(wide, sum) / 2;
  if constexpr (hn::MaxLanes(d) > 4) {
    if (end != w) {
      const hn::Half<D> half;
      total += ByteSatd(half, a + end, as, b + end, bs, w - end, h);
    }
  }
  return total;
}
#endif

template <class D, class V> HWY_INLINE V SatdAbs(D d, V value) {
  auto magnitude = hn::Abs(value);
#if defined(__clang__) && __clang_major__ == 22 && HWY_ARCH_ARM_A64 && \
    (HWY_TARGET == HWY_NEON || HWY_TARGET == HWY_NEON_WITHOUT_AES || HWY_TARGET == HWY_NEON_BF16)
  if constexpr (std::is_same_v<hn::TFromD<D>, std::int32_t> && hn::MaxLanes(d) <= 2) {
    // LLVM 22's AArch64 MachineCombiner maps SABAv2i32 to itself instead of
    // SABDv2i32 in getAccumulationStartOpcode, emitting a missing operand.
    // Keep ABS separate from ADD for 64-bit NEON SATD vectors (also used by
    // one-lane capped tags); wider vectors and other targets are unaffected.
    // This register barrier emits no instruction and does not clobber memory.
    asm("" : "+w"(magnitude.raw));
  }
#endif
  return magnitude;
}

template <class T, class D>
std::int64_t MetricWithTag(D d, const T *a, std::ptrdiff_t as, const T *b, std::ptrdiff_t bs, int w, int h, bool satd) {
  const hn::Rebind<T, D> narrow;
  const int n = int(hn::Lanes(d));
  using A = Wide<T>;
  std::int64_t total = 0;
  float ftotal = 0;
  HWY_ALIGN A sums[4][hn::MaxLanes(d)];
  auto append = [&](A s) {
    if constexpr (std::is_same_v<T, float>)
      ftotal += s; // Nonnegative terms: overflow cannot recover; encode checks the final sum.
    else
      total = metric_detail::accumulate(total, std::int64_t(s));
  };
  if (!satd) {
    for (int y = 0; y < h; ++y) {
      const T *ar = a + y * as;
      const T *br = b + y * bs;
      int x = 0;
      for (; x <= w - n; x += n) {
        auto av = LoadWide<1>(d, ar + x), bv = LoadWide<1>(d, br + x);
        finite(d, av);
        finite(d, bv);
        auto diff = hn::Abs(hn::Sub(av, bv));
        finite(d, diff);
        if constexpr (std::is_same_v<T, float>) {
          hn::StoreU(diff, d, sums[0]);
          for (int i = 0; i < n; ++i)
            append(sums[0][i]);
        } else
          append(hn::ReduceSum(d, diff));
      }
      for (; x < w; ++x)
        append(metric_detail::magnitude(
            metric_detail::subtract(metric_detail::finite(A(ar[x])), metric_detail::finite(A(br[x])))));
    }
  } else {
    for (int y = 0; y < h; y += 4) {
      int x = 0;
      for (; x <= w - 4 * n; x += 4 * n) {
        auto r00 = hn::Zero(d), r01 = r00, r02 = r00, r03 = r00, r10 = r00, r11 = r00, r12 = r00, r13 = r00;
        auto r20 = r00, r21 = r00, r22 = r00, r23 = r00, r30 = r00, r31 = r00, r32 = r00, r33 = r00;
#define NEO_ROW(J, A0, A1, A2, A3)                                                                                     \
  {                                                                                                                    \
    auto a0 = hn::Zero(narrow), a1 = a0, a2 = a0, a3 = a0, b0 = a0, b1 = a0, b2 = a0, b3 = a0;                         \
    hn::LoadInterleaved4(narrow, a + (y + J) * as + x, a0, a1, a2, a3);                                                \
    hn::LoadInterleaved4(narrow, b + (y + J) * bs + x, b0, b1, b2, b3);                                                \
    if constexpr (std::is_same_v<T, float>) {                                                                          \
      finite(d, a0);                                                                                                   \
      finite(d, a1);                                                                                                   \
      finite(d, a2);                                                                                                   \
      finite(d, a3);                                                                                                   \
      finite(d, b0);                                                                                                   \
      finite(d, b1);                                                                                                   \
      finite(d, b2);                                                                                                   \
      finite(d, b3);                                                                                                   \
      A0 = hn::Sub(a0, b0);                                                                                            \
      A1 = hn::Sub(a1, b1);                                                                                            \
      A2 = hn::Sub(a2, b2);                                                                                            \
      A3 = hn::Sub(a3, b3);                                                                                            \
    } else {                                                                                                           \
      A0 = hn::Sub(hn::PromoteTo(d, a0), hn::PromoteTo(d, b0));                                                        \
      A1 = hn::Sub(hn::PromoteTo(d, a1), hn::PromoteTo(d, b1));                                                        \
      A2 = hn::Sub(hn::PromoteTo(d, a2), hn::PromoteTo(d, b2));                                                        \
      A3 = hn::Sub(hn::PromoteTo(d, a3), hn::PromoteTo(d, b3));                                                        \
    }                                                                                                                  \
    Hadamard(A0, A1, A2, A3);                                                                                          \
    finite(d, A0);                                                                                                     \
    finite(d, A1);                                                                                                     \
    finite(d, A2);                                                                                                     \
    finite(d, A3);                                                                                                     \
  }
        NEO_ROW(0, r00, r01, r02, r03)
        NEO_ROW(1, r10, r11, r12, r13)
        NEO_ROW(2, r20, r21, r22, r23)
        NEO_ROW(3, r30, r31, r32, r33)
#undef NEO_ROW
        auto cell_sum = hn::Zero(d);
#define NEO_COL(I, A0, A1, A2, A3)                                                                                     \
  Hadamard(A0, A1, A2, A3);                                                                                            \
  {                                                                                                                    \
    auto s = hn::Add(hn::Add(hn::Add(SatdAbs(d, A0), SatdAbs(d, A1)), SatdAbs(d, A2)), SatdAbs(d, A3));                \
    finite(d, s);                                                                                                      \
    if constexpr (std::is_same_v<T, float> || hn::MaxLanes(d) > 128)                                                   \
      hn::StoreU(s, d, sums[I]);                                                                                       \
    else                                                                                                              \
      cell_sum = hn::Add(cell_sum, s);                                                                                \
  }
            NEO_COL(0, r00, r10, r20, r30) NEO_COL(1, r01, r11, r21, r31) NEO_COL(2, r02, r12, r22, r32)
                NEO_COL(3, r03, r13, r23, r33)
#undef NEO_COL
        // Each integer cell contributes at most 128 * 65535 after /2.
        // Up to 128 cells therefore reduce safely in signed 32-bit lanes.
        if constexpr (!std::is_same_v<T, float> && hn::MaxLanes(d) <= 128) {
          append(hn::ReduceSum(d, hn::ShiftRight<1>(cell_sum)));
        } else for (int i = 0; i < n; ++i) {
          if constexpr (std::is_same_v<T, float>)
            for (int c = 0; c < 4; ++c)
              append(sums[c][i]);
          else
            append((sums[0][i] + sums[1][i] + sums[2][i] + sums[3][i]) / 2);
        }
      }
      for (; x < w; x += 4) {
        using S = std::conditional_t<std::is_same_v<T, float>, float, std::int64_t>;
        std::array<std::array<S, 4>, 4> r{};
        for (int j = 0; j < 4; ++j) {
          for (int i = 0; i < 4; ++i)
            r[j][i] = metric_detail::subtract(metric_detail::finite(S(a[(y + j) * as + x + i])),
                                              metric_detail::finite(S(b[(y + j) * bs + x + i])));
          r[j] = metric_detail::hadamard(r[j]);
        }
        S cell = 0;
        for (int c = 0; c < 4; ++c) {
          auto col = metric_detail::hadamard<S>({r[0][c], r[1][c], r[2][c], r[3][c]});
          auto s = metric_detail::add(
              metric_detail::add(metric_detail::add(metric_detail::magnitude(col[0]), metric_detail::magnitude(col[1])),
                                 metric_detail::magnitude(col[2])),
              metric_detail::magnitude(col[3]));
          if constexpr (std::is_same_v<T, float>)
            append(s);
          else
            cell += s;
        }
        if constexpr (!std::is_same_v<T, float>)
          append(static_cast<A>(cell / 2));
      }
    }
    if constexpr (std::is_same_v<T, float>)
      ftotal *= 0.5f;
  }
  if constexpr (std::is_same_v<T, float>)
    return encode_float_error(ftotal);
  else
    return total;
}
template <class T>
std::int64_t Metric(const T *a, std::ptrdiff_t as, const T *b, std::ptrdiff_t bs, int w, int h, bool satd) {
#if HWY_TARGET != HWY_SCALAR
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    if (satd && w <= 128 && h <= 128) {
      if (w < 8)
        return ByteSatd(hn::CappedTag<std::int16_t, 4>{}, a, as, b, bs, w, h);
      if (w < 16)
        return ByteSatd(hn::CappedTag<std::int16_t, 8>{}, a, as, b, bs, w, h);
      if (w < 32)
        return ByteSatd(hn::CappedTag<std::int16_t, 16>{}, a, as, b, bs, w, h);
      return ByteSatd(hn::CappedTag<std::int16_t, 32>{}, a, as, b, bs, w, h);
    }
  }
#endif
  if constexpr (!std::is_same_v<T, float>) {
    if (!satd && w <= 128 && h <= 128) {
#if HWY_TARGET != HWY_SCALAR
      if constexpr (std::is_same_v<T, std::uint8_t>) {
        if (w == 4 && h == 4)
          return FixedByteSad4(a, as, b, bs);
        if (w == 16 && h == 16)
          return FixedByteSad<16, 16>(a, as, b, bs);
        if (w == 8 && h == 8)
          return FixedByteSad<8, 8>(a, as, b, bs);
        if (w >= 64)
          return SmallByteSad(hn::CappedTag<T, 64>{}, a, as, b, bs, w, h);
        if (w >= 32)
          return SmallByteSad(hn::CappedTag<T, 32>{}, a, as, b, bs, w, h);
        if (w >= 16)
          return SmallByteSad(hn::CappedTag<T, 16>{}, a, as, b, bs, w, h);
        if (w >= 8)
          return SmallByteSad(hn::CappedTag<T, 8>{}, a, as, b, bs, w, h);
      }
      if constexpr (std::is_same_v<T, std::uint16_t>) {
        if (w == 16 && h == 16 && 16 % hn::Lanes(hn::CappedTag<std::uint16_t, 16>{}) == 0)
          return FixedShortSad<16, 16>(a, as, b, bs);
        if (w == 8 && h == 8 && 8 % hn::Lanes(hn::CappedTag<std::uint16_t, 8>{}) == 0)
          return FixedShortSad<8, 8>(a, as, b, bs);
      }
#endif
      if (w < 8)
        return SmallSad(hn::CappedTag<std::int32_t, 4>{}, a, as, b, bs, w, h);
      if (w < 16)
        return SmallSad(hn::CappedTag<std::int32_t, 8>{}, a, as, b, bs, w, h);
      return SmallSad(hn::ScalableTag<std::int32_t>{}, a, as, b, bs, w, h);
    }
  }
  if constexpr (std::is_same_v<T, float>) {
    // Vectorize sample validation and subtraction for small SAD blocks, while
    // MetricWithTag still appends float differences in original pixel order.
    if (!satd && w < 8)
      return MetricWithTag(hn::CappedTag<float, 4>{}, a, as, b, bs, w, h, false);
    if (!satd && w < 16)
      return MetricWithTag(hn::CappedTag<float, 8>{}, a, as, b, bs, w, h, false);
  }
  // Narrow SATD lanes for small blocks so even one cell uses the vector kernel.
  if (satd) {
    if (w < 8)
      return MetricWithTag(hn::CappedTag<Wide<T>, 1>{}, a, as, b, bs, w, h, satd);
    if (w < 16)
      return MetricWithTag(hn::CappedTag<Wide<T>, 2>{}, a, as, b, bs, w, h, satd);
    if (w < 32)
      return MetricWithTag(hn::CappedTag<Wide<T>, 4>{}, a, as, b, bs, w, h, satd);
    if (w < 64)
      return MetricWithTag(hn::CappedTag<Wide<T>, 8>{}, a, as, b, bs, w, h, satd);
  }
  return MetricWithTag(hn::ScalableTag<Wide<T>>{}, a, as, b, bs, w, h, satd);
}
// Selected only for three 4:2:0 SAD planes with 16x16 Y and 8x8 U/V.
template <class T>
void MetricBatch420(const MetricRequest<T> *requests, int, std::int64_t *errors) {
#if HWY_TARGET != HWY_SCALAR
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    const auto &y = requests[0], &u = requests[1], &v = requests[2];
    errors[0] = FixedByteSad<16, 16>(y.source, y.source_stride, y.reference, y.reference_stride);
    errors[1] = FixedByteSad<8, 8>(u.source, u.source_stride, u.reference, u.reference_stride);
    errors[2] = FixedByteSad<8, 8>(v.source, v.source_stride, v.reference, v.reference_stride);
    return;
  }
  if constexpr (std::is_same_v<T, std::uint16_t>) {
    if (16 % hn::Lanes(hn::CappedTag<std::uint16_t, 16>{}) == 0 &&
        8 % hn::Lanes(hn::CappedTag<std::uint16_t, 8>{}) == 0) {
      const auto &y = requests[0], &u = requests[1], &v = requests[2];
      errors[0] = FixedShortSad<16, 16>(y.source, y.source_stride, y.reference, y.reference_stride);
      errors[1] = FixedShortSad<8, 8>(u.source, u.source_stride, u.reference, u.reference_stride);
      errors[2] = FixedShortSad<8, 8>(v.source, v.source_stride, v.reference, v.reference_stride);
      return;
    }
  }
#endif
  for (int i = 0; i < 3; ++i) {
    const auto &r = requests[i];
    errors[i] = Metric(r.source, r.source_stride, r.reference, r.reference_stride, r.width, r.height, r.satd);
  }
}
// Selected only for three 4:2:0 SAD planes with 8x8 Y and 4x4 U/V.
template <class T>
void MetricBatch420Small(const MetricRequest<T> *requests, int, std::int64_t *errors) {
#if HWY_TARGET != HWY_SCALAR
  if constexpr (std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::uint16_t>) {
    const auto &y = requests[0], &u = requests[1], &v = requests[2];
    if constexpr (std::is_same_v<T, std::uint8_t>)
      errors[0] = FixedByteSad<8, 8>(y.source, y.source_stride, y.reference, y.reference_stride);
    else if (8 % hn::Lanes(hn::CappedTag<std::uint16_t, 8>{}) == 0)
      errors[0] = FixedShortSad<8, 8>(y.source, y.source_stride, y.reference, y.reference_stride);
    else
      errors[0] = SmallSad(hn::CappedTag<std::int32_t, 8>{}, y.source, y.source_stride,
                           y.reference, y.reference_stride, 8, 8);
    if constexpr (std::is_same_v<T, std::uint8_t>) {
      errors[1] = FixedByteSad4(u.source, u.source_stride, u.reference, u.reference_stride);
      errors[2] = FixedByteSad4(v.source, v.source_stride, v.reference, v.reference_stride);
    } else {
      errors[1] = SmallSad(hn::CappedTag<std::int32_t, 4>{}, u.source, u.source_stride,
                           u.reference, u.reference_stride, 4, 4);
      errors[2] = SmallSad(hn::CappedTag<std::int32_t, 4>{}, v.source, v.source_stride,
                           v.reference, v.reference_stride, 4, 4);
    }
    return;
  }
#endif
  for (int i = 0; i < 3; ++i) {
    const auto &r = requests[i];
    errors[i] = Metric(r.source, r.source_stride, r.reference, r.reference_stride, r.width, r.height, r.satd);
  }
}
template <class T, int Width, bool Bounded, bool LateCheck = false>
HWY_INLINE std::int64_t Sad420Plane(const MetricRequest<T>& r, std::int64_t limit) {
#if HWY_TARGET != HWY_SCALAR
  if constexpr (std::is_same_v<T, std::uint16_t>) {
    if (Width % hn::Lanes(hn::CappedTag<std::uint16_t, Width>{}) == 0)
      return FixedShortSad<Width, Width, Bounded>(r.source, r.source_stride,
                                                r.reference, r.reference_stride, limit);
  }
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    if constexpr (Width == 4)
      return FixedByteSad4(r.source, r.source_stride, r.reference, r.reference_stride);
    else
      return FixedByteSad<Width, Width, Bounded, LateCheck>(r.source, r.source_stride,
                                                r.reference, r.reference_stride, limit);
  }
#endif
  return Metric(r.source, r.source_stride, r.reference, r.reference_stride, Width, Width, false);
}

template <class T, int LumaWidth>
bool MetricBatch420Bounded(const MetricRequest<T> *requests, std::int64_t limit, std::int64_t *errors) {
  errors[0] = Sad420Plane<T, LumaWidth, true>(requests[0], limit);
  if (errors[0] >= limit)
    return false;
  limit -= errors[0];
  for (int k = 1; k < 3; ++k) {
    errors[k] = Sad420Plane<T, LumaWidth / 2, false>(requests[k], limit);
    if (errors[k] >= limit)
      return false;
    limit -= errors[k];
  }
  return true;
}

template <int Width, bool Bounded>
HWY_NOINLINE std::int64_t AnalyseShortSad(const MetricRequest<std::uint16_t>& r, std::int64_t limit) {
  return Sad420Plane<std::uint16_t, Width, Bounded>(r, limit);
}
template <class T, int Width, bool Bounded, bool Fused>
HWY_INLINE std::int64_t MotionSad420Plane(const MetricRequest<T>& r, std::int64_t limit) {
  // Keep the larger u16 arithmetic body out of the fused search's live state.
  // This is a direct call within the already selected target, not dispatch.
  if constexpr (Fused && std::is_same_v<T, std::uint16_t>)
    return AnalyseShortSad<Width, Bounded>(r, limit);
  else
    return Sad420Plane<T, Width, Bounded>(r, limit);
}

HWY_INLINE std::int64_t MotionQuotient(std::int64_t value, int pel) {
  switch (pel) {
  case 1: return value;
  case 2: return value >= 0 ? value / 2 : -((-value + 1) / 2);
  default: return value >= 0 ? value / 4 : -((-value + 3) / 4);
  }
}

template <class T>
HWY_INLINE MetricRequest<T> MotionPlane(const MotionMetricRequest<T>& block,
                                        const MetricReferenceFrames<T>& frames, int k,
                                        std::int64_t qx, std::int64_t qy, std::size_t phase) {
  auto r = block.planes[k];
  r.reference_stride = frames.strides[k][phase];
  // Combine offsets before forming a pointer: the zero-vector block origin
  // need not itself be inside the admitted reference view.
  const auto offset = (std::ptrdiff_t(block.y[k]) + qy) * r.reference_stride +
                      (std::ptrdiff_t(block.x[k]) + qx);
  r.reference = frames.references[k][phase] + offset;
  return r;
}

template <class T, int LumaWidth, bool Bounded = true, int Pel = 0>
HWY_INLINE bool MotionMetric420Bounded(const MotionMetricRequest<T>& block, const MetricReferenceFrames<T>& frames,
                            int vx, int vy, std::int64_t limit, std::int64_t* errors) {
  const int pel = Pel == 0 ? block.pel : Pel;
  const auto qx = MotionQuotient(vx, pel), qy = MotionQuotient(vy, pel);
  const auto phase = std::size_t((vy - pel * qy) * pel + vx - pel * qx);
  const auto y = MotionPlane(block, frames, 0, qx, qy, phase);
  errors[0] = MotionSad420Plane<T, LumaWidth, Bounded, (Pel != 0)>(y, limit);
  if constexpr (Bounded) {
    if (errors[0] >= limit)
      return false;
    limit -= errors[0];
  }

  // Chroma truncates the vector before applying the phase floor division.
  // Defer both its phase and address work until the luma candidate survives.
  const auto tx = std::int64_t(vx) / 2, ty = std::int64_t(vy) / 2;
  const auto cx = MotionQuotient(tx, pel), cy = MotionQuotient(ty, pel);
  const auto chroma_phase = std::size_t((ty - pel * cy) * pel + tx - pel * cx);
  for (int k = 1; k < 3; ++k) {
    const auto r = MotionPlane(block, frames, k, cx, cy, chroma_phase);
    errors[k] = MotionSad420Plane<T, LumaWidth / 2, false, (Pel != 0)>(r, limit);
    if constexpr (Bounded) {
      if (errors[k] >= limit)
        return false;
      limit -= errors[k];
    }
  }
  return true;
}

namespace fused_motion {
// Instantiate the same scalar selection rules inside this SIMD target. This
// makes the error evaluator visible to the search loop without a function
// pointer or an optional BlockError crossing the boundary per candidate.
#define NEO_MV_MOTION_ATTR HWY_ATTR
#include "core/motion/search-loop-inl.hpp"
#include "core/motion/analyse-block-inl.hpp"
#undef NEO_MV_MOTION_ATTR

template <class T, int Pel, int GrayWidth = 0>
struct AnalyseSad {
  const MotionMetricRequest<T>& block;
  const MetricReferenceFrames<T>& frames;

  template <bool Bounded>
  HWY_INLINE bool metric(MotionVector vector, std::int64_t limit, std::int64_t* errors) const {
    if constexpr (GrayWidth != 0) {
      const auto qx = MotionQuotient(vector.x, Pel), qy = MotionQuotient(vector.y, Pel);
      const auto phase = std::size_t((vector.y - Pel * qy) * Pel + vector.x - Pel * qx);
      const auto r = MotionPlane(block, frames, 0, qx, qy, phase);
      // For 8x8, one final reduction is cheaper than checking a four-row prefix.
      // Gray 16x16 first checks at 12 rows; 420 keeps its earlier checks.
      errors[0] = Sad420Plane<T, GrayWidth, (Bounded && GrayWidth != 8), true>(r, limit);
      errors[1] = errors[2] = 0;
      return !Bounded || errors[0] < limit;
    } else {
      return MotionMetric420Bounded<T, 16, Bounded, Pel>(block, frames, vector.x, vector.y, limit, errors);
    }
  }
  HWY_INLINE BlockError operator()(MotionVector vector) const {
    std::int64_t errors[3];
    metric<false>(vector, INT64_MAX, errors);
    const auto chroma = errors[1] + errors[2];
    return {errors[0], chroma, errors[0] + chroma};
  }
  HWY_INLINE bool improve(MotionVector vector, const SearchParams& p, SearchResult& best) const {
    if (best.cost <= 0)
      return false;
    std::int64_t errors[3];
    if (!metric<true>(vector, best.cost, errors))
      return false;
    const auto luma = errors[0], chroma = errors[1] + errors[2], raw = luma + chroma;
    const auto dx = std::int64_t(vector.x) - p.predictor.x, dy = std::int64_t(vector.y) - p.predictor.y;
    // PreparedBlockError::analyse has proved this bound for the whole domain.
    const auto cost = raw + p.lambda * (dx * dx + dy * dy) / 256 +
                      luma * p.penalty / 256 + chroma * p.penalty / 256;
    if (cost >= best.cost)
      return false;
    best = {vector, cost, raw};
    return true;
  }
  SearchResult refine(SearchResult initial, const SearchParams& p) {
    return refine_impl(initial, p, *this);
  }
  // Keep this less frequent ring body out of the main search's live state.
  HWY_NOINLINE bool improve_expansion(MotionVector vector, const SearchParams& p, SearchResult& best) const {
    return improve(vector, p, best);
  }
};
} // namespace fused_motion

template <class T, int Pel>
SearchResult AnalyseBlock420(const MotionMetricRequest<T>& block, const MetricReferenceFrames<T>& frames,
                             MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero,
                             CandidateDomain omega, int layer, std::int64_t lambda,
                             std::int64_t bad_threshold, AnalyseControls controls) {
  fused_motion::AnalyseSad<T, Pel> execution{block, frames};
  return fused_motion::block_impl(predictor, spatial, zero, omega, layer, Pel, lambda, bad_threshold,
                                  controls, execution);
}
template <class T>
SearchResult AnalyseBlock420(const MotionMetricRequest<T>& block, const MetricReferenceFrames<T>& frames,
                             MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero,
                             CandidateDomain omega, int layer, std::int64_t lambda,
                             std::int64_t bad_threshold, AnalyseControls controls) {
  if (block.pel == 1)
    return AnalyseBlock420<T, 1>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
  if (block.pel == 2)
    return AnalyseBlock420<T, 2>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
  return AnalyseBlock420<T, 4>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
}
#define NEO_ANALYSE_IMPL(T, S)                                                                                       \
  SearchResult AnalyseBlock420##S(const MotionMetricRequest<T>& block, const MetricReferenceFrames<T>& frames,        \
      MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega,           \
      int layer, std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls) {                         \
    return AnalyseBlock420(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);   \
  }
NEO_ANALYSE_IMPL(std::uint8_t, U8)
NEO_ANALYSE_IMPL(std::uint16_t, U16)
#undef NEO_ANALYSE_IMPL

template <int Width, int Pel>
SearchResult AnalyseBlockGray(const MotionMetricRequest<std::uint8_t>& block,
    const MetricReferenceFrames<std::uint8_t>& frames, MotionTriple predictor,
    const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega, int layer,
    std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls) {
  fused_motion::AnalyseSad<std::uint8_t, Pel, Width> execution{block, frames};
  return fused_motion::block_impl(predictor, spatial, zero, omega, layer, Pel, lambda,
                                  bad_threshold, controls, execution);
}
template <int Width>
SearchResult AnalyseBlockGrayPel(const MotionMetricRequest<std::uint8_t>& block,
    const MetricReferenceFrames<std::uint8_t>& frames, MotionTriple predictor,
    const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega, int layer,
    std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls) {
  if (block.pel == 1)
    return AnalyseBlockGray<Width, 1>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
  if (block.pel == 2)
    return AnalyseBlockGray<Width, 2>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
  return AnalyseBlockGray<Width, 4>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
}

SearchResult AnalyseBlockGrayU8(const MotionMetricRequest<std::uint8_t>& block,
    const MetricReferenceFrames<std::uint8_t>& frames, MotionTriple predictor,
    const SpatialPredictors& spatial, MotionVector zero, CandidateDomain omega, int layer,
    std::int64_t lambda, std::int64_t bad_threshold, AnalyseControls controls) {
  if (block.planes[0].width == 4)
    return AnalyseBlockGrayPel<4>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
  if (block.planes[0].width == 8)
    return AnalyseBlockGrayPel<8>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
  return AnalyseBlockGrayPel<16>(block, frames, predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
}

#define NEO_IMPL(T, S)                                                                                                 \
  void Extract##S(const T *p, T *q, int n, int pel, int phase) {                                                       \
    Extract(p, q, n, pel, phase);                                                                                      \
  }                                                                                                                    \
  void Scan##S(const T *p, int n, std::int64_t m) {                                                                    \
    Scan(p, n, m);                                                                                                     \
  }                                                                                                                    \
  void Copy##S(const T *p, T *q, int n) {                                                                              \
    Copy(p, q, n);                                                                                                     \
  }                                                                                                                    \
  void Fill##S(T v, T *p, int n) {                                                                                     \
    Fill(v, p, n);                                                                                                     \
  }                                                                                                                    \
  void Formula##S(const T *const *p, int step, T *q, int n, Formula op, std::int64_t m) {                              \
    FormulaRow(p, step, q, n, op, m);                                                                                  \
  }                                                                                                                    \
  std::int64_t Metric##S(const T *a, std::ptrdiff_t as, const T *b, std::ptrdiff_t bs, int w, int h, bool s) {         \
    return Metric(a, as, b, bs, w, h, s);                                                                              \
  }                                                                                                                    \
  void MetricBatch##S(const MetricRequest<T> *requests, int count, std::int64_t *errors) {                             \
    for (int i = 0; i < count; ++i) {                                                                                   \
      const auto &r = requests[i];                                                                                     \
      errors[i] = Metric(r.source, r.source_stride, r.reference, r.reference_stride, r.width, r.height, r.satd);     \
    }                                                                                                                  \
  }                                                                                                                    \
  void MetricBatch420##S(const MetricRequest<T> *requests, int count, std::int64_t *errors) {                          \
    MetricBatch420(requests, count, errors);                                                                           \
  }                                                                                                                    \
  void MetricBatch420Small##S(const MetricRequest<T> *requests, int count, std::int64_t *errors) {                     \
    MetricBatch420Small(requests, count, errors);                                                                      \
  }                                                                                                                    \
  bool MetricBatch420Bounded##S(const MetricRequest<T> *requests, std::int64_t limit, std::int64_t *errors) {           \
    return MetricBatch420Bounded<T, 16>(requests, limit, errors);                                                       \
  }                                                                                                                    \
  bool MetricBatch420SmallBounded##S(const MetricRequest<T> *requests, std::int64_t limit, std::int64_t *errors) {      \
    return MetricBatch420Bounded<T, 8>(requests, limit, errors);                                                        \
  }                                                                                                                    \
  bool MotionMetric420Bounded##S(const MotionMetricRequest<T>& block, const MetricReferenceFrames<T>& frames,         \
                                 int vx, int vy, std::int64_t limit, std::int64_t* errors) {                           \
    return MotionMetric420Bounded<T, 16>(block, frames, vx, vy, limit, errors);                                         \
  }                                                                                                                    \
  bool MotionMetric420SmallBounded##S(const MotionMetricRequest<T>& block, const MetricReferenceFrames<T>& frames,    \
                                      int vx, int vy, std::int64_t limit, std::int64_t* errors) {                      \
    return MotionMetric420Bounded<T, 8>(block, frames, vx, vy, limit, errors);                                          \
  }
NEO_IMPL(std::uint8_t, U8) NEO_IMPL(std::uint16_t, U16) NEO_IMPL(float, F32)
#undef NEO_IMPL
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::detail
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::detail {
HWY_EXPORT(Target);
const char *target_name() {
  return hwy::TargetName(HWY_DYNAMIC_DISPATCH(Target)());
}
HWY_EXPORT(AnalyseBlock420U8);
HWY_EXPORT(AnalyseBlock420U16);
HWY_EXPORT(AnalyseBlockGrayU8);
AnalyseBlockFunction<std::uint8_t> analyse_block_gray_function() {
  return HWY_DYNAMIC_DISPATCH(AnalyseBlockGrayU8);
}
AnalyseBlockFunction<std::uint8_t> analyse_block_420_function(std::uint8_t*) {
  return HWY_DYNAMIC_DISPATCH(AnalyseBlock420U8);
}
AnalyseBlockFunction<std::uint16_t> analyse_block_420_function(std::uint16_t*) {
  return HWY_DYNAMIC_DISPATCH(AnalyseBlock420U16);
}
#define NEO_EXPORT(T, S)                                                                                               \
  HWY_EXPORT(Extract##S);                                                                                              \
  HWY_EXPORT(Scan##S);                                                                                                 \
  HWY_EXPORT(Copy##S);                                                                                                 \
  HWY_EXPORT(Fill##S);                                                                                                 \
  HWY_EXPORT(Formula##S);                                                                                              \
  HWY_EXPORT(Metric##S);                                                                                               \
  HWY_EXPORT(MetricBatch##S);                                                                                          \
  HWY_EXPORT(MetricBatch420##S);                                                                                       \
  HWY_EXPORT(MetricBatch420Small##S);                                                                                  \
  HWY_EXPORT(MetricBatch420Bounded##S);                                                                                \
  HWY_EXPORT(MotionMetric420Bounded##S);                                                                             \
  HWY_EXPORT(MotionMetric420SmallBounded##S);                                                                        \
  HWY_EXPORT(MetricBatch420SmallBounded##S);                                                                           \
  void extract(const T *p, T *q, int n, int pel, int phase) {                                                          \
    HWY_DYNAMIC_DISPATCH(Extract##S)(p, q, n, pel, phase);                                                             \
  }                                                                                                                    \
  void scan(const T *p, int n, std::int64_t m) {                                                                       \
    HWY_DYNAMIC_DISPATCH(Scan##S)(p, n, m);                                                                            \
  }                                                                                                                    \
  void copy(const T *p, T *q, int n) {                                                                                 \
    HWY_DYNAMIC_DISPATCH(Copy##S)(p, q, n);                                                                            \
  }                                                                                                                    \
  void fill(T v, T *p, int n) {                                                                                        \
    HWY_DYNAMIC_DISPATCH(Fill##S)(v, p, n);                                                                            \
  }                                                                                                                    \
  void formula(const T *const *p, int s, T *q, int n, Formula f, std::int64_t m) {                                     \
    HWY_DYNAMIC_DISPATCH(Formula##S)(p, s, q, n, f, m);                                                                \
  }                                                                                                                    \
  std::int64_t metric(const T *a, std::ptrdiff_t as, const T *b, std::ptrdiff_t bs, int w, int h, bool s) {            \
    return HWY_DYNAMIC_DISPATCH(Metric##S)(a, as, b, bs, w, h, s);                                                     \
  }                                                                                                                    \
  void metric_batch(const MetricRequest<T> *requests, int count, std::int64_t *errors) {                               \
    HWY_DYNAMIC_DISPATCH(MetricBatch##S)(requests, count, errors);                                                     \
  }                                                                                                                    \
  MetricBatchFunction<T> metric_batch_function(T *) {                                                                  \
    return HWY_DYNAMIC_DISPATCH(MetricBatch##S);                                                                       \
  }                                                                                                                    \
  MetricBatchFunction<T> metric_batch_420_function(T *) {                                                              \
    return HWY_DYNAMIC_DISPATCH(MetricBatch420##S);                                                                    \
  }                                                                                                                    \
  MetricBatchFunction<T> metric_batch_420_small_function(T *) {                                                        \
    return HWY_DYNAMIC_DISPATCH(MetricBatch420Small##S);                                                               \
  }                                                                                                                    \
  BoundedMetricBatchFunction<T> metric_batch_420_bounded_function(T *) {                                               \
    return HWY_DYNAMIC_DISPATCH(MetricBatch420Bounded##S);                                                             \
  }                                                                                                                    \
  BoundedMetricBatchFunction<T> metric_batch_420_small_bounded_function(T *) {                                         \
    return HWY_DYNAMIC_DISPATCH(MetricBatch420SmallBounded##S);                                                        \
  }                                                                                                                    \
  BoundedMotionMetricFunction<T> motion_metric_420_bounded_function(T *, int width) {                                 \
    return width == 16 ? HWY_DYNAMIC_DISPATCH(MotionMetric420Bounded##S)                                               \
                       : HWY_DYNAMIC_DISPATCH(MotionMetric420SmallBounded##S);                                       \
  }
NEO_EXPORT(std::uint8_t, U8) NEO_EXPORT(std::uint16_t, U16) NEO_EXPORT(float, F32)
#undef NEO_EXPORT
} // namespace neo_mv::simd::detail
#endif
