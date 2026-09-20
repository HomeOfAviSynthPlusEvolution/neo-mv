#pragma once

#include "core/motion/block_sampling.hpp"

#include <algorithm>
#include <optional>

namespace neo_mv {

struct SearchResult {
  MotionVector vector;
  std::int64_t cost, raw;
};
struct SearchParams {
  MotionVector predictor;
  std::int64_t lambda;
  std::int32_t penalty;
  CandidateDomain domain;
  std::int32_t type, range;
  std::optional<MotionVector> cross_center;
};

namespace search_detail {
inline std::int64_t product(std::int64_t a, std::int64_t b) {
  if (a != 0 && b > INT64_MAX / a)
    throw std::overflow_error("motion cost exceeds int64 range");
  return a * b;
}
inline std::int64_t distance(MotionVector v, MotionVector predictor, std::int64_t lambda) {
  const auto dx = std::int64_t(v.x) - predictor.x, dy = std::int64_t(v.y) - predictor.y;
  const auto x = static_cast<std::uint64_t>(dx < 0 ? -dx : dx);
  const auto y = static_cast<std::uint64_t>(dy < 0 ? -dy : dy);
  // Each square fits uint64, but their sum may not. Split before addition
  // and multiplication so an exactly representable final cost also works on
  // MSVC x86 without a native 128-bit integer type.
  const auto xx = x * x, yy = y * y;
  const auto remainder = (xx % 256 + yy % 256);
  const auto quotient = static_cast<std::int64_t>(xx / 256 + yy / 256 + remainder / 256);
  const auto r = static_cast<std::int64_t>(remainder % 256);
  const auto tail = metric_detail::accumulate(product(lambda / 256, r), (lambda % 256) * r / 256);
  return metric_detail::accumulate(product(lambda, quotient), tail);
}
inline std::int64_t penalty(std::int64_t error, int q) {
  return (error / 256) * q + (error % 256) * q / 256;
}
} // namespace search_detail

inline std::int64_t candidate_cost(MotionVector vector, MotionVector predictor, std::int64_t lambda, int penalty,
                                   BlockError error) {
  if (lambda < 0 || penalty < 0 || penalty > 256 || error.luma < 0 || error.chroma < 0)
    throw std::invalid_argument("invalid motion cost inputs");
  const auto raw = metric_detail::accumulate(error.luma, error.chroma);
  if (error.raw != raw)
    throw std::invalid_argument("inconsistent raw block error");
  auto cost = metric_detail::accumulate(search_detail::distance(vector, predictor, lambda), raw);
  cost = metric_detail::accumulate(cost, search_detail::penalty(error.luma, penalty));
  return metric_detail::accumulate(cost, search_detail::penalty(error.chroma, penalty));
}

// Caller precondition: validate_sampling_domain has admitted all of Omega and
// the initial vector using the real logical geometry before search starts.
// evaluate(v) is a deterministic block-error evaluator; its failures propagate,
// never becoming skipped candidates or an infinite comparison-cost sentinel.
template <class Evaluator>
SearchResult refine_motion(SearchResult initial, const SearchParams& p, Evaluator&& evaluate) {
  const auto omega = p.domain;
  if (p.lambda < 0 || p.penalty < 0 || p.penalty > 256 || p.type < 0 || p.type > 5 || p.range <= 0 ||
      initial.cost < 0 || initial.raw < 0 || omega.left < INT32_MIN || omega.top < INT32_MIN ||
      omega.right > std::int64_t(INT32_MAX) + 1 || omega.bottom > std::int64_t(INT32_MAX) + 1 ||
      omega.left >= omega.right || omega.top >= omega.bottom)
    throw std::invalid_argument("invalid motion refinement inputs");
  SearchResult best = initial;
  const auto consider = [&](std::int64_t x, std::int64_t y) {
    if (x < omega.left || x >= omega.right || y < omega.top || y >= omega.bottom)
      return false;
    const MotionVector v{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
    const BlockError error = evaluate(v);
    const auto cost = candidate_cost(v, p.predictor, p.lambda, p.penalty, error);
    if (cost >= best.cost)
      return false;
    best = {v, cost, error.raw};
    return true;
  };
  const auto offset = [&](MotionVector center, std::int64_t x, std::int64_t y) {
    return consider(std::int64_t(center.x) + x, std::int64_t(center.y) + y);
  };
  const auto ring = [&](MotionVector center, std::int64_t radius) {
    for (auto i = -radius + 1; i < radius; ++i) {
      offset(center, i, -radius);
      offset(center, i, radius);
    }
    for (auto j = -radius + 1; j < radius; ++j) {
      offset(center, -radius, j);
      offset(center, radius, j);
    }
    offset(center, -radius, -radius);
    offset(center, -radius, radius);
    offset(center, radius, -radius);
    offset(center, radius, radius);
  };
  const auto hexagonal = [&] {
    if (p.range == 1) {
      ring(best.vector, 1);
      return;
    }
    constexpr MotionVector h[] = {{-2, 0}, {-1, 2}, {1, 2}, {2, 0}, {1, -2}, {-1, -2}};
    auto center = best.vector;
    int direction = -1;
    for (int i = 0; i < 6; ++i)
      if (offset(center, h[i].x, h[i].y))
        direction = i;
    if (direction >= 0)
      for (int step = 0; step < p.range / 2 - 1; ++step) {
        center = best.vector;
        int winner = -1;
        for (int k = -1; k <= 1; ++k) {
          const int i = (direction + k + 6) % 6;
          if (offset(center, h[i].x, h[i].y))
            winner = i;
        }
        if (winner < 0)
          break;
        direction = winner;
      }
    ring(best.vector, 1);
  };

  const MotionVector origin = initial.vector;
  switch (p.type) {
    case 0: {
      constexpr MotionVector n[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      for (std::int64_t d = p.range; d > 0; d /= 2)
        for (;;) {
          const auto center = best.vector;
          bool improved = false;
          for (auto v : n)
            improved = offset(center, d * v.x, d * v.y) || improved;
          if (!improved)
            break;
        }
      break;
    }
    case 1:
      for (std::int64_t r = 1; r <= p.range; ++r)
        ring(origin, r);
      break;
    case 2:
      hexagonal();
      break;
    case 3: {
      const auto center = p.cross_center.value_or(origin);
      for (std::int64_t d = 1; d < p.range; d += 2) {
        offset(center, -d, 0);
        offset(center, d, 0);
      }
      for (std::int64_t d = 1; d < p.range; d += 2) {
        offset(center, 0, -d);
        offset(center, 0, d);
      }
      constexpr MotionVector group[] = {{-4, 2}, {-4, 1}, {-4, 0}, {-4, -1}, {-4, -2}, {4, -2},  {4, -1}, {4, 0},
                                        {4, 1},  {4, 2},  {2, 3},  {0, 4},   {-2, 3},  {-2, -3}, {0, -4}, {2, -3}};
      for (std::int64_t k = 1; k <= std::max(1, p.range / 4); ++k)
        for (auto v : group)
          offset(center, k * v.x, k * v.y);
      hexagonal();
      break;
    }
    case 4:
    case 5:
      for (std::int64_t d = 1; d <= p.range; ++d) {
        if (p.type == 4) {
          offset(origin, -d, 0);
          offset(origin, d, 0);
        } else {
          offset(origin, 0, -d);
          offset(origin, 0, d);
        }
      }
      break;
  }
  return best;
}

} // namespace neo_mv
