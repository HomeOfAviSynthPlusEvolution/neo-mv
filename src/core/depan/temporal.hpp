#pragma once
#include "core/depan/transform.hpp"
#include <optional>

namespace neo_mv::depan {
inline float decode_component(double x, float low, float high) {
  finite64(x);
  // Clamping first outside the binary32 endpoints also handles finite double
  // values whose conversion to float would overflow.
  return x < double(low) ? low : x > double(high) ? high : f32(x);
}
inline Motion decode_motion(double dx, double dy, double r, double z, std::int64_t good) {
  return {decode_component(dx, -1000000, 1000000), decode_component(dy, -1000000, 1000000),
          decode_component(r, -360000, 360000), decode_component(z, 0.01f, 100), good != 0};
}
inline bool field_parity(int n, std::optional<bool> tff, std::optional<std::int64_t> field) {
  if (tff)
    return *tff ^ ((n & 1) != 0);
  if (!field)
    throw std::invalid_argument("required Depan field parity missing");
  return *field != 0;
}
struct TemporalPosition {
  int source = 0, next = 0, last = 0;
  float fraction = 0;
  bool bypass = true, forward = true;
};
class TemporalPlan {
  int frames_, shift_;
  float offset_, aspect_, cx_, cy_;
  bool parity_;

public:
  TemporalPlan(int frames, int width, int height, double offset, double aspect = 1, bool fields = false,
               bool matchfields = true)
      : frames_(frames), offset_(f32(offset)), aspect_(f32(aspect)), cx_(div(f32(width), 2)), cy_(div(f32(height), 2)),
        parity_(fields && matchfields) {
    if (frames <= 0 || width <= 0 || height <= 0 || offset_ < -10 || offset_ > 10 || aspect_ <= 0)
      throw std::invalid_argument("invalid Depan temporal parameters");
    aspect_ = div(aspect_, fields ? 2.0f : 1.0f);
    geometry(aspect_, cx_, cy_);
    shift_ = static_cast<int>(offset_ > 0 ? std::ceil(offset_) : std::floor(offset_));
  }
  float aspect() const { return aspect_; }
  float cx() const { return cx_; }
  float cy() const { return cy_; }
  float offset() const { return offset_; }
  bool needs_parity() const { return parity_; }
  TemporalPosition position(int n) const {
    if (n < 0 || n >= frames_)
      throw std::invalid_argument("Depan frame index outside clip");
    const auto source = std::int64_t(n) - shift_;
    if (!shift_ || source < 0 || source >= frames_)
      return {n};
    const int s = static_cast<int>(source);
    const bool forward = shift_ > 0;
    return {s,      std::min(s, n) + 1, std::max(s, n), sub(add(offset_, forward ? 1.0f : -1.0f), f32(shift_)), false,
            forward};
  }
  Transform append(Transform accumulated, Motion m, const TemporalPosition& p) const {
    return compose(accumulated, coordinates(m, aspect_, cx_, cy_, p.fraction, p.forward));
  }
  Transform align(Transform t, bool top) const {
    if (parity_)
      t.ty = add(t.ty, top ? -0.5f : 0.5f);
    return t;
  }
  template <class Read>
  Transform accumulate(const TemporalPosition& p, Read&& read) const {
    Transform result;
    if (p.bypass)
      return result;
    for (int k = p.next; k <= p.last; ++k) {
      const Motion m = read(k);
      if (!m.good)
        return {};
      result = append(result, m, p);
    }
    return result;
  }
};
} // namespace neo_mv::depan
