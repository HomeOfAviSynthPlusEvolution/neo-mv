#include "core/motion/dct_refine.hpp"
#include "core/motion/dct_rounding.hpp"
#include "core/motion/dct_fixed_constants.hpp"
#include <boost/multiprecision/cpp_int.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace neo_mv::dct_detail {
// Q60 cosine enclosures and Q88 products. Up to16384 unsigned16 samples
// keep every product, accumulated numerator and cell comparison below2^122.
bool fixed_round(const std::uint16_t* input, int w, int h, int u, int v, int q) {

#if defined(__SIZEOF_INT128__) && !defined(NEO_MV_DCT_PORTABLE_WIDE)
  using Wide = __int128;
#else
  using Wide = boost::multiprecision::int128_t;
#endif
  const Wide step = Wide(1) << 32, mask = step - 1;
  const auto floor32 = [&](Wide x) -> Wide {
    return x >= 0 ? x >> 32 : -(((-x) + mask) >> 32);
  };
  const auto ceil32 = [&](Wide x) -> Wide {
    return -floor32(-x);
  };
  auto cx = fixed_cosine_table(w), cy = fixed_cosine_table(h);
  Wide low = 0, high = 0;
  for (int y = 0; y < h; ++y) {
    auto b = cy[((2 * y + 1) * v) % (4 * h)];
    for (int x = 0; x < w; ++x) {
      auto a = cx[((2 * x + 1) * u) % (4 * w)];
      Wide products[4] = {Wide(a.lo) * b.lo, Wide(a.lo) * b.hi, Wide(a.hi) * b.lo, Wide(a.hi) * b.hi};
      Wide lo = products[0], hi = products[0];
      for (int j = 1; j < 4; ++j) {
        if (products[j] < lo)
          lo = products[j];
        if (products[j] > hi)
          hi = products[j];
      }
      auto sample = std::int64_t(input[y * w + x]);
      low += 4 * sample * floor32(lo);
      high += 4 * sample * ceil32(hi);
    }
  }
  const Wide dl = Wide(sqrt2_floor) * w * h, dh = Wide(sqrt2_floor + 1) * w * h;
  const auto lower = 2 * q - 1, upper = 2 * q + 1;
  const Wide lower_limit = Wide(lower) * (lower >= 0 ? dh : dl) * (Wide(1) << 28);
  const Wide upper_limit = Wide(upper) * (upper >= 0 ? dl : dh) * (Wide(1) << 28);
  return 2 * low > lower_limit && 2 * high < upper_limit;
}
namespace {
using Int = boost::multiprecision::number<boost::multiprecision::cpp_int_backend<>, boost::multiprecision::et_off>;
struct Interval {
  Int lo, hi;
};
inline Int down(Int a, const Int& b) {
  Int q = a / b;
  if (a < 0 && a % b != 0)
    --q;
  return q;
}
inline Int up(Int a, const Int& b) {
  return -down(-a, b);
}
inline Int round_even(const Int& value, const Int& unit) {
  Int q = down(value, unit);
  const Int twice_fraction = 2 * (value - q * unit);
  if (twice_fraction > unit || (twice_fraction == unit && q % 2 != 0))
    ++q;
  return q;
}
inline Interval plus(const Interval& a, const Interval& b) {
  return {a.lo + b.lo, a.hi + b.hi};
}
inline Interval minus(const Interval& a) {
  return {-a.hi, -a.lo};
}
inline Interval divide(const Interval& a, const Int& d) {
  return {down(a.lo, d), up(a.hi, d)};
}
struct Arithmetic {
  Int unit;
  int bits;
  Interval pi;
  explicit Arithmetic(int precision) : unit(Int(1) << precision), bits(precision) {
    auto a = atan(5), b = atan(239);
    pi = {16 * a.lo - 4 * b.hi, 16 * a.hi - 4 * b.lo};
  }
  Interval times(const Interval& a, const Interval& b) const {
    Int products[4] = {a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi};
    return {down(*std::min_element(products, products + 4), unit), up(*std::max_element(products, products + 4), unit)};
  }
  Interval atan(int q) const {
    Interval sum{0, 0};
    Int power = q;
    const int count = bits / 4 + 8;
    for (int j = 0; j < count; ++j) {
      Int denominator = power * (2 * j + 1);
      Interval term{unit / denominator, up(unit, denominator)};
      sum = plus(sum, j % 2 ? minus(term) : term);
      power *= q * q;
    }
    Int remainder = up(unit, Int(power * (2 * count + 1)));
    if (count % 2)
      sum.lo -= remainder;
    else
      sum.hi += remainder;
    return sum;
  }
  Interval cos(int n, int k, int x) const {
    int e = ((2 * x + 1) * k) % (4 * n);
    e = std::min(e, 4 * n - e);
    Interval angle = divide({pi.lo * e, pi.hi * e}, Int(2 * n));
    auto square = times(angle, angle);
    Interval term{unit, unit}, sum = term;
    for (int j = 1;; ++j) {
      term = divide(times(term, square), ((Int(2) * j - 1) * (Int(2) * j)));
      sum = plus(sum, j % 2 ? minus(term) : term);
      if (j >= 4 && term.hi <= 1) {
        auto tail = divide(times(term, square), ((Int(2) * j + 1) * (Int(2) * j + 2)));
        return {sum.lo - tail.hi, sum.hi + tail.hi};
      }
    }
  }
  Interval coefficient(const std::uint16_t* input, int w, int h, int u, int v) const {
    std::vector<Interval> cx, cy;
    for (int x = 0; x < w; ++x)
      cx.push_back(cos(w, u, x));
    for (int y = 0; y < h; ++y)
      cy.push_back(cos(h, v, y));
    Interval sum{0, 0};
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        auto term = times(cx[x], cy[y]);
        auto sample = std::int64_t(input[y * w + x]);
        sum.lo += 4 * sample * term.lo;
        sum.hi += 4 * sample * term.hi;
      }
    // Integer sqrt is exact floor; positive denominator interval encloses sqrt(2)*area.
    Int root = boost::multiprecision::sqrt(Int(2 * unit * unit));
    Int denominators[2] = {root * w * h, (root + 1) * w * h};
    Int low = down(Int(sum.lo * unit), denominators[0]), high = up(Int(sum.hi * unit), denominators[0]);
    for (auto& d : denominators) {
      low = std::min(low, down(Int(sum.lo * unit), d));
      high = std::max(high, up(Int(sum.hi * unit), d));
    }
    return {low, high};
  }
};

} // namespace

int interval_round(const std::uint16_t* input, int w, int h, int u, int v) {
  for (int precision = 96;;) {
    Arithmetic arithmetic(precision);
    auto value = arithmetic.coefficient(input, w, h, u, v);
    const Int& one = arithmetic.unit;
    Int low = round_even(value.lo, one), high = round_even(value.hi, one);
    if (low == high)
      return low.convert_to<int>();
    // Equality is resolved algebraically; otherwise narrowing enclosures
    // eventually isolate the nonzero distance from the nearest half integer.
    if (high - low == 1) {
      auto k = low.convert_to<std::int64_t>();
      if (exact_half(input, w, h, u, v, k))
        return int(k + (k % 2 != 0));
    }
    if (precision > std::numeric_limits<int>::max() / 2)
      throw std::overflow_error("DCT refinement precision overflow");
    precision *= 2;
  }
}
int refine_ac(const std::uint16_t* input, int w, int h, int u, int v, double estimate) {
  const auto k = std::int64_t(std::floor(estimate));
  if (exact_half(input, w, h, u, v, k))
    return int(k + (k % 2 != 0));
  const double fraction = estimate - double(k);
  const int guess = int(k) + (fraction > .5 || (fraction == .5 && k % 2 != 0));
  if (fixed_round(input, w, h, u, v, guess))
    return guess;
  return interval_round(input, w, h, u, v);
}
} // namespace neo_mv::dct_detail
