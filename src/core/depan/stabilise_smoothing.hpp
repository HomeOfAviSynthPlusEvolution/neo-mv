#pragma once
#include "core/depan/stabilise_recovery.hpp"

namespace neo_mv::depan::stabilise {
inline float smooth_component(float b, float previous, float u, float u1, float u2, float alpha, float beta,
                              float kappa, float f) {
  using recovery::add;
  using recovery::sub;
  using recovery::mul;
  using recovery::div;
  const float a = sub(mul(2, b), previous);
  const float e = add(sub(sub(b, previous), u1), u2), d = sub(b, u1);
  const float nonlinear = div(mul(.5f, kappa), f);
  const float restoring = mul(mul(mul(mul(beta, f), f), d), add(1, mul(kappa, std::abs(d))));
  const float predicted = sub(sub(a, mul(mul(mul(alpha, f), e), add(1, mul(nonlinear, std::abs(e))))), restoring);
  const float ec = add(sub(sub(predicted, previous), u), u2);
  return sub(sub(a, mul(mul(mul(mul(alpha, f), .5f), ec), add(1, mul(mul(nonlinear, .5f), std::abs(ec))))), restoring);
}
inline std::vector<Transform> inertial(const std::vector<Transform>& cumulative, const Coefficients& c) {
  using recovery::add;
  using recovery::mul;
  if (cumulative.size() < 2)
    throw std::invalid_argument("DepanStabilise inertial interval is too short");
  std::vector<Transform> result(cumulative.size());
  for (std::size_t j = 2; j < cumulative.size(); ++j) {
    const auto& u = cumulative[j];
    auto component = [&](float Transform::* member, float alpha, float beta, float kappa) {
      return smooth_component(result[j - 1].*member, result[j - 2].*member, u.*member, cumulative[j - 1].*member,
                              cumulative[j - 2].*member, alpha, beta, kappa, c.f);
    };
    auto& r = result[j];
    r.tx = component(&Transform::tx, c.cd, c.cq, c.kx);
    r.ty = component(&Transform::ty, c.cd, c.cq, c.ky);
    r.v = component(&Transform::v, mul(c.cd, 2), mul(c.cq, 4), c.kr);
    r.u = r.h = mul(.5f, add(u.u, result[j - 1].u));
    r.w = mul(mul(-r.v, c.aspect), c.aspect);
  }
  return result;
}
template <bool Recover = false>
inline float zoom_bound(Transform t, int width, int height, const Coefficients& c) {
  if constexpr (!Recover)
    validate(t);
  constexpr auto add = Recover ? recovery::add : depan::add;
  constexpr auto sub = Recover ? recovery::sub : depan::sub;
  constexpr auto mul = Recover ? recovery::mul : depan::mul;
  constexpr auto div = Recover ? recovery::div : depan::div;
  float bound = finite(c.z0);
  auto include = [&](float v) {
    if (v < bound)
      bound = v;
  };
  include(add(1, div(add(t.tx, mul(t.v, c.cy)), c.cx)));
  include(sub(1, div(sub(add(add(t.tx, mul(t.u, f32(width))), mul(t.v, c.cy)), f32(width)), c.cx)));
  include(add(1, div(add(t.ty, mul(t.w, c.cx)), c.cy)));
  include(sub(1, div(sub(add(add(t.ty, mul(t.w, c.cx)), mul(t.h, f32(height))), f32(height)), c.cy)));
  return bound;
}
inline float smooth_zoom(float b, float previous, float a, float a1, float a2, float zf, const Coefficients& c) {
  using recovery::add;
  using recovery::sub;
  using recovery::mul;
  const float h = sub(mul(2, b), previous), e = add(sub(sub(b, previous), a1), a2), d = sub(b, a1);
  auto trial = [&](float factor) {
    const float restoring = mul(mul(mul(mul(mul(factor, factor), c.cq), c.f), c.f), d);
    const float predicted = sub(sub(h, mul(mul(mul(factor, c.cd), c.f), e)), restoring);
    const float ec = add(sub(sub(predicted, previous), a), a2);
    return sub(sub(h, mul(mul(mul(mul(factor, c.cd), c.f), .5f), ec)), restoring);
  };
  float result = trial(zf);
  if (result > b)
    result = trial(mul(zf, .7f));
  return result > 1 ? 1 : result;
}
inline Transform inertial_zoom(const std::vector<Transform>& cumulative, const std::vector<Transform>& smoothed,
                               int width, int height, const Parameters& p, const Coefficients& c) {
  if (cumulative.size() < 2 || cumulative.size() != smoothed.size())
    throw std::invalid_argument("invalid DepanStabilise adaptive interval");
  float a2 = c.z0, a1 = c.z0, s2 = c.z0, s1 = c.z0;
  Transform result;
  for (std::size_t j = 2; j < cumulative.size(); ++j) {
    const float a = zoom_bound<true>(recovery::compose(inverse(cumulative[j]), smoothed[j]), width, height, c);
    const float s = smooth_zoom(s1, s2, a, a1, a2, div(1, mul(p.cutoff, p.tzoom)), c);
    result = recovery::compose(smoothed[j], recovery::zoom(s, c));
    a2 = a1;
    a1 = a;
    s2 = s1;
    s1 = s;
  }
  return result;
}
// The sequence is centered on the current frame. All sums include endpoints
// and keep the prescribed binary32 left-to-right accumulation.
inline Transform window(const std::vector<Transform>& cumulative, int width, int height, const Parameters& p,
                        const Coefficients& c) {
  if (cumulative.empty() || cumulative.size() % 2 == 0 || cumulative.size() / 2 > std::size_t(c.radius))
    throw std::invalid_argument("invalid DepanStabilise symmetric interval");
  const auto middle = cumulative.size() / 2;
  auto weight = [&](std::size_t i, const std::vector<float>& weights) {
    return weights.at(i < middle ? middle - i : i - middle);
  };
  auto average = [&](float Transform::* member, std::size_t begin, std::size_t end) {
    float sum = 0, denominator = 0;
    for (auto i = begin; i <= end; ++i) {
      const float w = weight(i, c.weights);
      sum = add(sum, mul(cumulative[i].*member, w));
      denominator = add(denominator, w);
    }
    return div(sum, denominator);
  };
  Transform s;
  s.tx = average(&Transform::tx, 0, cumulative.size() - 1);
  s.ty = average(&Transform::ty, 0, cumulative.size() - 1);
  s.v = average(&Transform::v, 0, cumulative.size() - 1);
  s.u = s.h = average(&Transform::u, middle ? middle - 1 : 0, (std::min)(middle + 1, cumulative.size() - 1));
  s.w = mul(mul(-s.v, c.aspect), c.aspect);
  float scale = c.z0;
  if (p.addzoom) {
    const auto radius = (std::min)(middle, std::size_t(c.zoom_radius));
    const auto begin = middle - radius, end = middle + radius;
    float sum = 0, denominator = 0;
    for (auto i = begin; i <= end; ++i) {
      const float a = i == begin ? c.z0 : zoom_bound(compose(inverse(cumulative[i]), cumulative[i]), width, height, c);
      const float w = weight(i, c.zoom_weights);
      sum = add(sum, mul(a, w));
      denominator = add(denominator, w);
    }
    scale = div(sum, denominator);
    if (scale > 1)
      scale = 1;
  }
  return compose(s, zoom(scale, c));
}
struct Correction {
  Transform map;
  int begin;
};
inline Correction limit_correction(Transform raw, int n, int frames, int begin, const Parameters& p,
                                   const Coefficients& c) {
  frame_valid(n, frames);
  Motion m = recovery::motion(raw, c);
  if (std::int64_t(frames) < std::int64_t(p.fitlast) + n + 1) {
    const float e = div(f32(frames - n - 1), f32(p.fitlast));
    m.dx = recovery::mul(m.dx, e);
    m.dy = recovery::mul(m.dy, e);
    m.rotation = recovery::mul(m.rotation, e);
    m.zoom = recovery::add(c.z0, recovery::mul(recovery::sub(m.zoom, c.z0), e));
  }
  auto reset = [&] {
    m = {0, 0, 0, c.z0, true};
    begin = n;
  };
  auto translation = [&](float& v, float limit) {
    if (!std::isfinite(v)) {
      reset();
      return;
    }
    if (std::abs(v) > std::abs(limit)) {
      if (limit < 0) {
        reset();
        return;
      }
      auto reduce = [&] {
        v = mul(v >= 0 ? 1.0f : -1.0f, sqrt32(mul(std::abs(v), limit)));
      };
      reduce();
      if (std::abs(v) > std::abs(limit))
        reduce();
      if (std::abs(v) > std::abs(mul(limit, 1.5f)))
        v = mul(v >= 0 ? 1.0f : -1.0f, limit);
    }
  };
  translation(m.dx, p.dxmax);
  translation(m.dy, p.dymax);
  if (!std::isfinite(m.zoom))
    reset();
  else if (std::abs(sub(m.zoom, 1)) > sub(std::abs(c.zoom_limit), 1)) {
    if (c.zoom_limit < 0)
      reset();
    else {
      const float delta = sqrt32(mul(std::abs(sub(m.zoom, 1)), std::abs(sub(c.zoom_limit, 1))));
      m.zoom = m.zoom >= 1 ? add(1, delta) : sub(1, delta);
    }
  }
  if (!std::isfinite(m.rotation))
    reset();
  else if (std::abs(m.rotation) > std::abs(p.rotmax)) {
    if (p.rotmax < 0)
      reset();
    else
      m.rotation = mul(m.rotation >= 0 ? 1.0f : -1.0f, sqrt32(mul(std::abs(m.rotation), p.rotmax)));
  }
  return {coordinates(m, c.aspect, c.cx, c.cy, 1, true), begin};
}
inline Correction correction(const std::vector<Transform>& cumulative, Interval interval, int n, int frames, int width,
                             int height, const Parameters& p, const Coefficients& c) {
  frame_valid(n, frames);
  if (interval.begin > n || interval.end < n || interval.begin < 0 || interval.end >= frames ||
      cumulative.size() != std::size_t(interval.end) - interval.begin + 1)
    throw std::invalid_argument("invalid DepanStabilise correction interval");
  if (p.method == 0 && interval.begin == n)
    return {zoom(c.z0, c), n};
  // The current inverse remains checked before recoverable arithmetic.
  const auto current_inverse = inverse(cumulative[std::size_t(n - interval.begin)]);
  if (p.method == 0) {
    auto r = inertial(cumulative, c);
    const auto s =
        p.addzoom ? inertial_zoom(cumulative, r, width, height, p, c) : recovery::compose(r.back(), zoom(c.z0, c));
    return limit_correction(recovery::compose(current_inverse, s), n, frames, interval.begin, p, c);
  }
  const auto s = window(cumulative, width, height, p, c);
  const auto q = compose(current_inverse, s);
  return {coordinates(motion(q, c.aspect, c.cx, c.cy, true), c.aspect, c.cx, c.cy, 1, true), interval.begin};
}
} // namespace neo_mv::depan::stabilise
