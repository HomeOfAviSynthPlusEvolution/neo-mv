#pragma once
#include "core/mask/numeric.hpp"

namespace neo_mv::depan {
inline float f32(double x) {
  return mask_detail::binary32(x);
}
inline float finite(float x) {
  if (!std::isfinite(x))
    throw std::invalid_argument("non-finite Depan operand");
  return x;
}
// Local to a built-in arithmetic loop that does not change the rounding mode.
// Keep the environment in a register instead of consulting thread state for
// every operation. Exceptional values still use the exact conversion path.
class ArithmeticContext {
  bool nearest_ = mask_detail::nearest_rounding();

public:
  float add(float a, float b) const { return mask_detail::binary32(double(finite(a)) + double(finite(b)), nearest_); }
  float mul(float a, float b) const { return mask_detail::binary32(double(finite(a)) * double(finite(b)), nearest_); }
};
inline float add(float a, float b) {
  return f32(double(finite(a)) + double(finite(b)));
}
inline float sub(float a, float b) {
  return f32(double(finite(a)) - double(finite(b)));
}
inline float mul(float a, float b) {
  return f32(double(finite(a)) * double(finite(b)));
}
inline float div(float a, float b) {
  if (finite(b) == 0)
    throw std::invalid_argument("zero Depan divisor");
  return f32(double(finite(a)) / double(b));
}
inline float sqrt32(float x) {
  if (finite(x) < 0)
    throw std::invalid_argument("negative Depan square root");
  return f32(std::sqrt(double(x)));
}
inline float sin32(float x) {
  return x == 0 ? x : f32(std::sin(double(finite(x))));
}
inline float cos32(float x) {
  return x == 0 ? 1.0f : f32(std::cos(double(finite(x))));
}
inline float atan32(float x) {
  return x == 0 ? x : f32(std::atan(double(finite(x))));
}
inline float log32(float x) {
  if (finite(x) <= 0)
    throw std::invalid_argument("nonpositive Depan logarithm");
  return x == 1 ? 0.0f : f32(std::log(double(x)));
}
inline float exp32(float x) {
  return x == 0 ? 1.0f : f32(std::exp(double(finite(x))));
}
inline double finite64(double x) {
  if (!std::isfinite(x))
    throw std::invalid_argument("non-finite Depan binary64 arithmetic");
  return x;
}
inline double add64(double a, double b) {
  volatile double r = finite64(a) + finite64(b);
  return finite64(r);
}
inline double sub64(double a, double b) {
  volatile double r = finite64(a) - finite64(b);
  return finite64(r);
}
inline double mul64(double a, double b) {
  volatile double r = finite64(a) * finite64(b);
  return finite64(r);
}
inline double div64(double a, double b) {
  if (finite64(b) == 0)
    throw std::invalid_argument("zero Depan binary64 divisor");
  volatile double r = finite64(a) / b;
  return finite64(r);
}
} // namespace neo_mv::depan
