#pragma once
#include "core/depan/numeric.hpp"

namespace neo_mv::depan {
struct Motion {
  float dx = 0, dy = 0, rotation = 0, zoom = 1;
  bool good = false;
};
struct Transform {
  float tx = 0, ty = 0, u = 1, v = 0, w = 0, h = 1;
};
inline void validate(Transform t) {
  for (float x : {t.tx, t.ty, t.u, t.v, t.w, t.h})
    finite(x);
}
inline void geometry(float a, float cx, float cy) {
  if (finite(a) <= 0)
    throw std::invalid_argument("nonpositive Depan aspect");
  finite(cx);
  finite(cy);
}
constexpr float pi = 0x1.921fb6p1f;
inline Transform coordinates(Motion m, float a, float cx, float cy, float f, bool forward) {
  geometry(a, cx, cy);
  const float dx = mul(f, m.dx), dy = mul(f, m.dy);
  float theta = div(mul(mul(f, m.rotation), pi), 180);
  if (std::abs(theta) < 0.000001f)
    theta = 0;
  finite(m.zoom);
  float z = exp32(mul(f, log32(m.zoom <= 0 ? 1.0f : m.zoom)));
  if (std::abs(sub(z, 1)) < 0.000001f)
    z = 1;
  const float s = sin32(theta), c = cos32(theta);
  Transform t;
  t.u = t.h = mul(c, z);
  t.v = mul(div(-s, a), z);
  t.w = mul(mul(s, z), a);
  if (forward) {
    t.tx = add(add(cx, mul(add(mul(-cx, c), mul(div(cy, a), s)), z)), dx);
    t.ty = add(cy, mul(add(mul(add(mul(div(-cy, a), c), mul(-cx, s)), z), dy), a));
  } else {
    t.tx = add(cx, mul(sub(mul(add(-cx, dx), c), mul(add(div(-cy, a), dy), s)), z));
    t.ty = add(cy, mul(mul(add(mul(add(div(-cy, a), dy), c), mul(add(-cx, dx), s)), z), a));
  }
  return t;
}
// Apply A, then B; retain all independently rounded affine coefficients.
inline Transform compose(Transform A, Transform B) {
  validate(A);
  validate(B);
  return {add(add(B.tx, mul(B.u, A.tx)), mul(B.v, A.ty)),
          add(add(B.ty, mul(B.w, A.tx)), mul(B.h, A.ty)),
          add(mul(B.u, A.u), mul(B.v, A.w)),
          add(mul(B.u, A.v), mul(B.v, A.h)),
          add(mul(B.w, A.u), mul(B.h, A.w)),
          add(mul(B.w, A.v), mul(B.h, A.h))};
}
inline Transform analysis_inverse(Transform t) {
  validate(t);
  if (t.h != t.u)
    throw std::invalid_argument("Depan inverse requires similarity model");
  const float b = t.v != 0 ? sqrt32(div(-t.w, t.v)) : 1.0f;
  const float d = add(mul(t.u, t.u), mul(mul(mul(t.v, t.v), b), b));
  Transform r;
  r.u = r.h = div(t.u, d);
  r.v = div(mul(-r.u, t.v), t.u);
  r.w = mul(mul(-r.v, b), b);
  r.tx = sub(mul(-r.u, t.tx), mul(r.v, t.ty));
  r.ty = sub(mul(-r.w, t.tx), mul(r.u, t.ty));
  return r;
}
inline Motion motion(Transform t, float a, float cx, float cy, bool forward) {
  validate(t);
  geometry(a, cx, cy);
  const float theta = -atan32(div(mul(a, t.v), t.u));
  const float r = div(mul(theta, 180), pi), s = sin32(theta), c = cos32(theta), z = div(t.u, c);
  Motion m{0, 0, r, z, true};
  if (forward) {
    m.dx = sub(sub(t.tx, cx), mul(add(mul(-cx, c), mul(div(cy, a), s)), z));
    m.dy = sub(sub(div(t.ty, a), div(cy, a)), mul(add(mul(div(-cy, a), c), mul(-cx, s)), z));
  } else {
    m.dx = sub(add(sub(add(mul(div(t.tx, z), c), mul(div(div(t.ty, z), a), s)), mul(div(cx, z), c)), cx),
               mul(div(div(cy, z), a), s));
    m.dy = sub(sub(add(add(mul(div(-t.tx, z), s), mul(div(div(t.ty, z), a), c)), mul(div(cx, z), s)), div(-cy, a)),
               mul(div(div(cy, z), a), c));
  }
  return m;
}
inline Transform plane_transform(Transform t, int ratio_x, int ratio_y) {
  validate(t);
  if ((ratio_x != 1 && ratio_x != 2) || (ratio_y != 1 && ratio_y != 2) || ratio_y > ratio_x)
    throw std::invalid_argument("unsupported Depan plane ratio");
  if (ratio_x == 2) {
    t.tx = div(t.tx, 2);
    if (ratio_y == 2)
      t.ty = div(t.ty, 2);
    else {
      t.v = div(t.v, 2);
      t.w = mul(t.w, 2);
    }
  }
  return t;
}
} // namespace neo_mv::depan
