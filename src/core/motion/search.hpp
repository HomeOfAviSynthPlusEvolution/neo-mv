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
  // In the ordinary bounded search, the complete product fits signed 64-bit:
  // 2 * 32767^2 * INT32_MAX < INT64_MAX. Keep the split calculation for
  // public callers with larger vectors or lambda values.
  if (x <= 32767 && y <= 32767 && lambda >= 0 && lambda <= INT32_MAX)
    return lambda * (dx * dx + dy * dy) / 256;
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
template <class Evaluator>
auto bounded(Evaluator& evaluate, MotionVector vector, std::int64_t limit, MotionVector predictor,
             std::int64_t lambda, int penalty, int)
    -> decltype(evaluate.bounded(vector, limit, predictor, lambda, penalty)) {
  return evaluate.bounded(vector, limit, predictor, lambda, penalty);
}
template <class Evaluator>
std::optional<BlockError> bounded(Evaluator& evaluate, MotionVector vector, std::int64_t, MotionVector,
                                  std::int64_t, int, long) {
  return evaluate(vector);
}
} // namespace search_detail

#if defined(__GNUC__) || defined(__clang__)
__attribute__((always_inline))
#endif
inline std::int64_t candidate_cost(MotionVector vector, MotionVector predictor, std::int64_t lambda, int penalty,
                                   BlockError error) {
  if (lambda < 0 || penalty < 0 || penalty > 256 || error.luma < 0 || error.chroma < 0)
    throw std::invalid_argument("invalid motion cost inputs");
  const auto raw = metric_detail::accumulate(error.luma, error.chroma);
  if (error.raw != raw)
    throw std::invalid_argument("inconsistent raw block error");
  const auto dx = std::int64_t(vector.x) - predictor.x;
  const auto dy = std::int64_t(vector.y) - predictor.y;
  if (raw <= (1LL << 28) && lambda <= INT32_MAX && dx >= -32767 && dx <= 32767 && dy >= -32767 && dy <= 32767) {
    // The distance product is below 2^62 and each penalty product is at most
    // 2^36, so no checked accumulation is needed for this common case.
    return raw + lambda * (dx * dx + dy * dy) / 256 + error.luma * penalty / 256 +
           error.chroma * penalty / 256;
  }
  auto cost = metric_detail::accumulate(search_detail::distance(vector, predictor, lambda), raw);
  cost = metric_detail::accumulate(cost, search_detail::penalty(error.luma, penalty));
  return metric_detail::accumulate(cost, search_detail::penalty(error.chroma, penalty));
}

// Caller precondition: validate_sampling_domain has admitted all of Omega and
// the initial vector using the real logical geometry before search starts.
// evaluate(v) is a deterministic block-error evaluator; its failures propagate,
// never becoming skipped candidates or an infinite comparison-cost sentinel.
namespace search_detail {
#define NEO_MV_MOTION_ATTR
#include "core/motion/search-loop-inl.hpp"
#undef NEO_MV_MOTION_ATTR

template <class Evaluator>
struct SearchEvaluator {
  Evaluator& evaluate;
  BlockError operator()(MotionVector vector) { return evaluate(vector); }
  bool improve(MotionVector vector, const SearchParams& p, SearchResult& best) {
    const auto error = bounded(evaluate, vector, best.cost, p.predictor, p.lambda, p.penalty, 0);
    if (!error)
      return false;
    const auto cost = candidate_cost(vector, p.predictor, p.lambda, p.penalty, *error);
    if (cost >= best.cost)
      return false;
    best = {vector, cost, error->raw};
    return true;
  }
  SearchResult refine(SearchResult initial, const SearchParams& p) {
    return refine_impl(initial, p, *this);
  }
  bool improve_expansion(MotionVector vector, const SearchParams& p, SearchResult& best) {
    return improve(vector, p, best);
  }
};
} // namespace search_detail

template <class Evaluator>
SearchResult refine_motion(SearchResult initial, const SearchParams& p, Evaluator&& evaluate) {
  search_detail::SearchEvaluator<std::remove_reference_t<Evaluator>> execution{evaluate};
  return execution.refine(initial, p);
}

} // namespace neo_mv
