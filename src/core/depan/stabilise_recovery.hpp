#pragma once
#include "core/depan/stabilise_motion.hpp"

namespace neo_mv::depan::stabilise::recovery {
// Only the method-0 smoothing-to-correction boundary uses this arithmetic.
// Public inputs, cumulative inverses, finite limit expressions and rendering
// keep the checked Depan operations. Never change the thread's FP environment.
inline float rounded(double x) {
  if (std::isnan(x))
    return std::numeric_limits<float>::quiet_NaN();
  if (std::abs(x) >= 0x1.ffffffp127)
    return std::copysign(std::numeric_limits<float>::infinity(), x);
  return f32(x);
}
inline float add(float a, float b) {
  return rounded(double(a) + double(b));
}
inline float sub(float a, float b) {
  return rounded(double(a) - double(b));
}
inline float mul(float a, float b) {
  return rounded(double(a) * double(b));
}
inline float div(float a, float b) {
  if (std::isfinite(a) && std::isfinite(b) && b == 0)
    throw std::invalid_argument("zero DepanStabilise divisor");
  return rounded(double(a) / double(b));
}

// These private transforms preserve the checked operators' expression order,
// but admit propagated infinities/NaNs until ordered correction limiting.
inline Transform compose(Transform A, Transform B) {
  return {add(add(B.tx, mul(B.u, A.tx)), mul(B.v, A.ty)),
          add(add(B.ty, mul(B.w, A.tx)), mul(B.h, A.ty)),
          add(mul(B.u, A.u), mul(B.v, A.w)),
          add(mul(B.u, A.v), mul(B.v, A.h)),
          add(mul(B.w, A.u), mul(B.h, A.w)),
          add(mul(B.w, A.v), mul(B.h, A.h))};
}
inline Transform zoom(float scale, const Coefficients& c) {
  // Specialization of coordinates({0,0,0,scale}, ..., 1, true). Retain
  // zero products: 0 * infinity must propagate NaN, not disappear.
  const float logarithm = rounded(std::log(double(scale <= 0 ? 1.0f : scale)));
  float z = rounded(std::exp(double(mul(1, logarithm))));
  if (std::abs(sub(z, 1)) < .000001f)
    z = 1;
  return {add(add(c.cx, mul(add(mul(-c.cx, 1), mul(div(c.cy, c.aspect), 0)), z)), 0),
          add(c.cy, mul(add(mul(add(mul(div(-c.cy, c.aspect), 1), mul(-c.cx, 0)), z), 0), c.aspect)),
          mul(1, z),
          mul(div(-0.0f, c.aspect), z),
          mul(mul(0, z), c.aspect),
          mul(1, z)};
}
inline Motion motion(Transform t, const Coefficients& c) {
  const float theta = -rounded(std::atan(double(div(mul(c.aspect, t.v), t.u))));
  const float r = div(mul(theta, 180), pi);
  const float s = rounded(std::sin(double(theta))), cosine = rounded(std::cos(double(theta)));
  const float z = div(t.u, cosine);
  return {
      sub(sub(t.tx, c.cx), mul(add(mul(-c.cx, cosine), mul(div(c.cy, c.aspect), s)), z)),
      sub(sub(div(t.ty, c.aspect), div(c.cy, c.aspect)), mul(add(mul(div(-c.cy, c.aspect), cosine), mul(-c.cx, s)), z)),
      r, z, true};
}
} // namespace neo_mv::depan::stabilise::recovery
