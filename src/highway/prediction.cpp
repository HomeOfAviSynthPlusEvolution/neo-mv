#include "highway/prediction.hpp"
#include <cstddef>
#include <cstring>
#include <type_traits>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/prediction.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;

#if HWY_TARGET != HWY_SCALAR && HWY_HAVE_FLOAT64
template <class D, class V>
HWY_INLINE void LoadTriples(D d, const MotionTriple* src, V& x, V& y, V& error) {
  static_assert(std::is_trivially_copyable_v<MotionTriple> && sizeof(MotionTriple) == 16);
  static_assert(offsetof(MotionTriple, vector) == 0 && offsetof(MotionTriple, error) == 8);
  static_assert(sizeof(MotionVector) == 8 && offsetof(MotionVector, x) == 0 && offsetof(MotionVector, y) == 4);
  // Copy object representations legally. Fixed-width targets fold this into
  // contiguous vector loads, avoiding scalar gathers and component arrays.
  HWY_ALIGN std::int64_t words[2 * hn::MaxLanes(d)];
  std::memcpy(words, src, hn::Lanes(d) * sizeof(MotionTriple));
  auto xy = hn::Zero(d);
  hn::LoadInterleaved2(d, words, xy, error);
#if HWY_IS_LITTLE_ENDIAN
  x = hn::ShiftRight<32>(hn::ShiftLeft<32>(xy));
  y = hn::ShiftRight<32>(xy);
#else
  y = hn::ShiftRight<32>(hn::ShiftLeft<32>(xy));
  x = hn::ShiftRight<32>(xy);
#endif
}
#endif

void InterpolatePredictions(const MotionGrid& parent, PredictionGeometry geometry, MotionGrid& child) {
#if HWY_TARGET == HWY_SCALAR || !HWY_HAVE_FLOAT64
  neo_mv::interpolate_predictions(parent, geometry, child);
#else
  prediction_detail::validate(child);
  if (&parent == &child)
    throw std::invalid_argument("prediction output aliases parent");
  const PredictionInterpolationPlan plan(parent, geometry);
  const hn::ScalableTag<std::int64_t> d;
  const hn::Rebind<double, decltype(d)> df;
  const int lanes = static_cast<int>(hn::Lanes(d));
  HWY_ALIGN std::int64_t output[4 * hn::MaxLanes(d)];
  const int pairs = std::min(parent.width - 1, (child.width - 1) / 2);
  for (int y = 0; y < child.height; ++y) {
    const auto t = std::min(std::int64_t(y), std::int64_t(2) * parent.height - 1);
    if (t == 0 || t == std::int64_t(2) * parent.height - 1) {
      // The scalar contract duplicates horizontal edge samples in its own
      // order, which matters for asymmetric overlap. Keep that exact rule.
      for (int x = 0; x < child.width; ++x)
        child.values[std::size_t(y) * child.width + x] = plan(x, y);
      continue;
    }
    const int py = static_cast<int>(t / 2);
    const bool down = t % 2 != 0;
    const int ny = py + (down ? 1 : -1);
    const auto* a = parent.values.data() + std::size_t(py) * parent.width;
    const auto* b = parent.values.data() + std::size_t(ny) * parent.width;
    auto* dst = child.values.data() + std::size_t(y) * child.width;
    dst[0] = plan(0, y);
    int p = 0;
    for (; p <= pairs - lanes; p += lanes) {
      auto ax = hn::Zero(d), ay = ax, ae = ax, bx = ax, by = ax, be = ax;
      auto cx = ax, cy = ax, ce = ax, dx = ax, dy = ax, de = ax;
      LoadTriples(d, a + p, ax, ay, ae);
      LoadTriples(d, a + p + 1, bx, by, be);
      LoadTriples(d, b + p, cx, cy, ce);
      LoadTriples(d, b + p + 1, dx, dy, de);
      const auto valid_error = [&](auto v) {
        return hn::And(hn::Ge(v, hn::Zero(d)), hn::Le(v, hn::Set(d, plan.error_limit())));
      };
      bool safe =
          hn::AllTrue(d, hn::And(hn::And(valid_error(ae), valid_error(be)), hn::And(valid_error(ce), valid_error(de))));
      if (safe) {
        const auto interpolate = [&](auto va, auto vb, auto vc, auto vd, int parity, bool error) {
          const auto& w = plan.weights(2 * int(down) + (parity == 0));
          // The even child uses the right parent as A, with its neighbour
          // to the left. Both children can reuse the same four loads.
          const int left = parity == 0 ? 0 : 1, right = 1 - left;
          auto sum = hn::Add(hn::Mul(va, hn::Set(d, w[left])), hn::Mul(vb, hn::Set(d, w[right])));
          sum = hn::Add(sum, hn::Mul(vc, hn::Set(d, w[left + 2])));
          sum = hn::Add(sum, hn::Mul(vd, hn::Set(d, w[right + 2])));
          if (plan.overlap())
            sum = hn::ConvertTo(d, hn::Mul(hn::ConvertTo(df, sum), hn::Set(df, plan.reciprocal())));
          else if (error)
            sum = hn::Add(sum, hn::Set(d, 8));
          sum = hn::ShiftRightSame(sum, error ? 4 : plan.shift());
          if (!error)
            safe =
                safe && hn::AllTrue(d, hn::And(hn::Ge(sum, hn::Set(d, INT32_MIN)), hn::Le(sum, hn::Set(d, INT32_MAX))));
          return sum;
        };
        const auto x0 = interpolate(ax, bx, cx, dx, 0, false), y0 = interpolate(ay, by, cy, dy, 0, false);
        const auto e0 = interpolate(ae, be, ce, de, 0, true);
        const auto x1 = interpolate(ax, bx, cx, dx, 1, false), y1 = interpolate(ay, by, cy, dy, 1, false);
        const auto e1 = interpolate(ae, be, ce, de, 1, true);
        const auto pack = [&](auto x, auto y) {
#if HWY_IS_LITTLE_ENDIAN
          return hn::Or(hn::And(x, hn::Set(d, 0xffffffffLL)), hn::ShiftLeft<32>(y));
#else
          return hn::Or(hn::And(y, hn::Set(d, 0xffffffffLL)), hn::ShiftLeft<32>(x));
#endif
        };
        if (safe) {
          hn::StoreInterleaved4(pack(x0, y0), e0, pack(x1, y1), e1, d, output);
          std::memcpy(dst + 2 * p + 1, output, 2 * lanes * sizeof(MotionTriple));
        }
      }
      if (!safe)
        for (int i = 0; i < lanes; ++i)
          for (int parity = 0; parity < 2; ++parity) {
            const int x = 2 * (p + i) + 1 + parity;
            dst[x] = plan(x, y);
          }
    }
    // Includes incomplete SIMD groups, the right edge and clamped extension.
    for (int x = 2 * p + 1; x < child.width; ++x)
      dst[x] = plan(x, y);
  }
#endif
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace neo_mv::simd {
HWY_EXPORT(InterpolatePredictions);
void interpolate_predictions(const MotionGrid& parent, PredictionGeometry geometry, MotionGrid& child) {
  HWY_DYNAMIC_DISPATCH(InterpolatePredictions)(parent, geometry, child);
}
} // namespace neo_mv::simd
#endif
