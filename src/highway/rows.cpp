#include "highway/rows.hpp"
#include "core/motion/block_metric.hpp"
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
template <Formula Op, int Step, class T>
void FormulaRowKnownStep(const T *const *p, T *out, int count, std::int64_t maximum) {
  const hn::ScalableTag<FormulaWide<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  constexpr int nt = Op == Formula::average ? 2 : ((Op == Formula::reduce6 || Op == Formula::sharp6) ? 6 : 4);
  const T *taps[6]{};
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
template <int Width, int Height>
std::int64_t FixedShortSad(const std::uint16_t *a, std::ptrdiff_t as, const std::uint16_t *b, std::ptrdiff_t bs) {
  const hn::CappedTag<std::int32_t, Width> d;
  const hn::Rebind<std::uint16_t, decltype(d)> narrow;
  const int lanes = int(hn::Lanes(d));
  auto sum0 = hn::Zero(d), sum1 = sum0, sum2 = sum0, sum3 = sum0;
  for (int y = 0; y < Height; y += 4) {
    const auto* ar = a + y * as;
    const auto* br = b + y * bs;
    for (int x = 0; x < Width; x += lanes) {
      sum0 = hn::Add(sum0, hn::PromoteTo(d, hn::AbsDiff(hn::LoadU(narrow, ar + x), hn::LoadU(narrow, br + x))));
      sum1 = hn::Add(sum1, hn::PromoteTo(d, hn::AbsDiff(hn::LoadU(narrow, ar + as + x),
                                                        hn::LoadU(narrow, br + bs + x))));
      sum2 = hn::Add(sum2, hn::PromoteTo(d, hn::AbsDiff(hn::LoadU(narrow, ar + 2 * as + x),
                                                        hn::LoadU(narrow, br + 2 * bs + x))));
      sum3 = hn::Add(sum3, hn::PromoteTo(d, hn::AbsDiff(hn::LoadU(narrow, ar + 3 * as + x),
                                                        hn::LoadU(narrow, br + 3 * bs + x))));
    }
  }
  return hn::ReduceSum(d, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3)));
}
template <int Width, int Height>
std::int64_t FixedByteSad(const std::uint8_t *a, std::ptrdiff_t as, const std::uint8_t *b, std::ptrdiff_t bs) {
  const hn::CappedTag<std::uint8_t, Width> d;
  const hn::Repartition<std::uint64_t, decltype(d)> wide;
  auto sum0 = hn::Zero(wide), sum1 = sum0, sum2 = sum0, sum3 = sum0;
  for (int y = 0; y < Height; y += 4) {
    const auto* ar = a + y * as;
    const auto* br = b + y * bs;
    sum0 = hn::Add(sum0, hn::SumsOf8AbsDiff(hn::LoadU(d, ar), hn::LoadU(d, br)));
    sum1 = hn::Add(sum1, hn::SumsOf8AbsDiff(hn::LoadU(d, ar + as), hn::LoadU(d, br + bs)));
    sum2 = hn::Add(sum2, hn::SumsOf8AbsDiff(hn::LoadU(d, ar + 2 * as), hn::LoadU(d, br + 2 * bs)));
    sum3 = hn::Add(sum3, hn::SumsOf8AbsDiff(hn::LoadU(d, ar + 3 * as), hn::LoadU(d, br + 3 * bs)));
  }
  return static_cast<std::int64_t>(hn::ReduceSum(wide, hn::Add(hn::Add(sum0, sum1), hn::Add(sum2, sum3))));
}
std::int64_t FixedByteSad4(const std::uint8_t *a, std::ptrdiff_t as, const std::uint8_t *b, std::ptrdiff_t bs) {
  const hn::CappedTag<std::uint8_t, 8> d;
  const hn::Repartition<std::uint64_t, decltype(d)> wide;
  auto sum = hn::Zero(wide);
  for (int y = 0; y < 4; ++y) {
    sum = hn::Add(sum, hn::SumsOf8AbsDiff(hn::LoadN(d, a, 4), hn::LoadN(d, b, 4)));
    if (y + 1 < 4) {
      a += as;
      b += bs;
    }
  }
  return static_cast<std::int64_t>(hn::ReduceSum(wide, sum));
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
    auto s = hn::Add(hn::Add(hn::Add(hn::Abs(A0), hn::Abs(A1)), hn::Abs(A2)), hn::Abs(A3));                            \
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
  if constexpr (!std::is_same_v<T, float>) {
    if (!satd && w <= 128 && h <= 128) {
#if HWY_TARGET != HWY_SCALAR
      if constexpr (std::is_same_v<T, std::uint8_t>) {
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
        if (w == 16 && h == 16 && 16 % hn::Lanes(hn::CappedTag<std::int32_t, 16>{}) == 0)
          return FixedShortSad<16, 16>(a, as, b, bs);
        if (w == 8 && h == 8 && 8 % hn::Lanes(hn::CappedTag<std::int32_t, 8>{}) == 0)
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
    if (16 % hn::Lanes(hn::CappedTag<std::int32_t, 16>{}) == 0 &&
        8 % hn::Lanes(hn::CappedTag<std::int32_t, 8>{}) == 0) {
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
    else if (8 % hn::Lanes(hn::CappedTag<std::int32_t, 8>{}) == 0)
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
template <class T, int LumaWidth>
bool MetricBatch420Bounded(const MetricRequest<T> *requests, std::int64_t limit, std::int64_t *errors) {
  for (int k = 0; k < 3; ++k) {
    const auto &r = requests[k];
#if HWY_TARGET != HWY_SCALAR
    if constexpr (std::is_same_v<T, std::uint8_t>) {
      if (k == 0) {
        std::int64_t sum = 0;
        for (int row = 0; row < LumaWidth; row += 4) {
          sum += FixedByteSad<LumaWidth, 4>(r.source + row * r.source_stride, r.source_stride,
                                           r.reference + row * r.reference_stride, r.reference_stride);
          if (sum >= limit)
            return false;
        }
        errors[0] = sum;
        limit -= sum;
        continue;
      }
    }
#endif
#if HWY_TARGET != HWY_SCALAR
    if constexpr (std::is_same_v<T, std::uint8_t>) {
      if constexpr (LumaWidth == 16)
        errors[k] = k == 0 ? FixedByteSad<16, 16>(r.source, r.source_stride, r.reference, r.reference_stride)
                           : FixedByteSad<8, 8>(r.source, r.source_stride, r.reference, r.reference_stride);
      else
        errors[k] = k == 0 ? FixedByteSad<8, 8>(r.source, r.source_stride, r.reference, r.reference_stride)
                           : FixedByteSad4(r.source, r.source_stride, r.reference, r.reference_stride);
    } else
#endif
      errors[k] = Metric(r.source, r.source_stride, r.reference, r.reference_stride,
                         r.width, r.height, false);
    if (errors[k] >= limit)
      return false;
    limit -= errors[k];
  }
  return true;
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
  }
NEO_EXPORT(std::uint8_t, U8) NEO_EXPORT(std::uint16_t, U16) NEO_EXPORT(float, F32)
#undef NEO_EXPORT
} // namespace neo_mv::simd::detail
#endif
