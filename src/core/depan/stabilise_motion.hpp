#pragma once
#include "core/depan/stabilise_parameters.hpp"

namespace neo_mv::depan::stabilise {
struct Interval {
  int begin, end;
};
inline void frame_valid(int n, int frames) {
  if (frames <= 0 || n < 0 || n >= frames)
    throw std::invalid_argument("invalid DepanStabilise frame index");
}
inline Interval initial_interval(int n, int frames, const Parameters& p, const Coefficients& c) {
  frame_valid(n, frames);
  int lower, upper;
  if (p.method == 0) {
    const float cap = mul(c.fps, 5);
    // Only this quotient permits positive overflow before applying its cap.
    // Both operands are binary32; the binary64 quotient cannot overflow.
    const float product = mul(10, c.fps);
    if (finite(p.cutoff) <= 0)
      throw std::invalid_argument("invalid DepanStabilise cutoff");
    const double quotient = double(product) / double(p.cutoff);
    const float t = quotient > double(std::numeric_limits<float>::max()) ? cap : [&] {
      const float rounded = f32(quotient);
      return rounded < cap ? rounded : cap;
    }();
    lower = (std::max)(0, truncate32(sub(f32(n), t)));
    if (lower > n)
      throw std::invalid_argument("DepanStabilise lookback begins after current frame");
    upper = n;
  } else {
    lower = static_cast<int>((std::max)(std::int64_t(0), std::int64_t(n) - c.radius));
    upper = static_cast<int>((std::min)(std::int64_t(frames - 1), std::int64_t(n) + c.radius));
  }
  return {lower, upper};
}
// The callback must decode all five properties and reject malformed tuples.
// It may retain decoded values for subsequent cumulative and neighbor passes.
template <class Decode>
Interval select_interval(int n, int frames, const Parameters& p, const Coefficients& c, Decode&& decode) {
  const auto initial = initial_interval(n, frames, p, c);
  const int lower = initial.begin, upper = initial.end;
  int begin = lower, end = upper;
  for (int k = n; k >= lower; --k) {
    if (k == 0 || !decode(k).good) {
      begin = k;
      break;
    }
  }
  if (p.method == 1) {
    for (std::int64_t k = std::int64_t(n) + 1; k <= upper; ++k)
      if (!decode(static_cast<int>(k)).good) {
        end = static_cast<int>(k) - 1;
        break;
      }
    const int half = (std::min)(n - begin, end - n);
    begin = n - half;
    end = n + half;
  }
  return {begin, end};
}
// Phase 7 deliberately ignores the input h when inverting the similarity
// model. Keep Phase 5's stricter analysis_inverse admission unchanged.
inline Transform inverse(Transform t) {
  validate(t);
  t.h = t.u;
  return analysis_inverse(t);
}
inline Transform zoom(float z, const Coefficients& c) {
  return coordinates({0, 0, 0, z, true}, c.aspect, c.cx, c.cy, 1, true);
}
template <class GetMotion>
std::vector<Transform> cumulative(Interval interval, const Coefficients& c, GetMotion&& get) {
  if (interval.begin < 0 || interval.end < interval.begin)
    throw std::invalid_argument("invalid DepanStabilise cumulative interval");
  std::vector<Transform> result(std::size_t(interval.end) - interval.begin + 1);
  for (std::int64_t j = std::int64_t(interval.begin) + 1; j <= interval.end; ++j) {
    const Motion m = get(static_cast<int>(j));
    if (!m.good)
      throw std::invalid_argument("invalid motion inside DepanStabilise cumulative interval");
    const auto index = std::size_t(j - interval.begin);
    result[index] = compose(result[index - 1], coordinates(m, c.aspect, c.cx, c.cy, 1, true));
  }
  return result;
}
} // namespace neo_mv::depan::stabilise
