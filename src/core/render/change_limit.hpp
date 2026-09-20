#pragma once

#include "core/super/subpixel.hpp"

namespace neo_mv {

template <class T>
class ChangeLimit {
  std::int64_t maximum_, integer_limit_ = 0;
  float limit_;
  bool active_;

public:
  explicit ChangeLimit(double limit, int bits) : maximum_(subpixel_detail::sample_max<T>(bits)) {
    // Define overflow of a host binary64 argument before narrowing to float.
    // All non-finite binary32 limits disable this parameter-specific operator.
    constexpr double overflow = 0x1.ffffffp+127;
    if (limit >= overflow)
      limit_ = std::numeric_limits<float>::infinity();
    else if (limit <= -overflow)
      limit_ = -std::numeric_limits<float>::infinity();
    else if (limit > std::numeric_limits<float>::max())
      limit_ = std::numeric_limits<float>::max();
    else if (limit < -std::numeric_limits<float>::max())
      limit_ = -std::numeric_limits<float>::max();
    else
      limit_ = static_cast<float>(limit);
    active_ = std::isfinite(limit_);
    if (active_ && limit_ <= 0)
      throw std::invalid_argument("finite change limit must be positive after float conversion");
    if constexpr (!std::is_same_v<T, float>) {
      if (active_ && limit_ >= static_cast<float>(maximum_))
        active_ = false;
      if (active_)
        integer_limit_ = static_cast<std::int64_t>(limit_ + 0.5f);
    }
  }
  bool active() const { return active_; }
  T operator()(T q, T centre) const {
    subpixel_detail::valid_sample(q, maximum_);
    subpixel_detail::valid_sample(centre, maximum_);
    if (!active_)
      return q;
    if constexpr (std::is_same_v<T, float>) {
      const float lo = centre - limit_, hi = centre + limit_;
      if (!std::isfinite(lo) || !std::isfinite(hi))
        throw std::overflow_error("non-finite change limit endpoint");
      // Comparisons preserve q's sign when either signed zero is in range.
      return q < lo ? lo : q > hi ? hi : q;
    } else {
      const auto lo = std::max<std::int64_t>(0, std::int64_t(centre) - integer_limit_);
      const auto hi = std::min(maximum_, std::int64_t(centre) + integer_limit_);
      return static_cast<T>(std::clamp(std::int64_t(q), lo, hi));
    }
  }
};

template <class T>
void limit_render_plane(const ChangeLimit<T>& limit, span2d::Plane<const T> composed, span2d::Plane<const T> centre,
                        span2d::Plane<T> output) {
  validate_plane(composed);
  validate_plane(centre);
  validate_plane(output);
  if (composed.width() != output.width() || composed.height() != output.height() || centre.width() != output.width() ||
      centre.height() != output.height() || active_rows_overlap(composed, output) ||
      active_rows_overlap(centre, output))
    throw std::invalid_argument("change limit plane geometry mismatch or output alias");
  for (int y = 0; y < output.height(); ++y)
    for (int x = 0; x < output.width(); ++x)
      output.row(y)[x] = limit(composed.row(y)[x], centre.row(y)[x]);
}

} // namespace neo_mv
