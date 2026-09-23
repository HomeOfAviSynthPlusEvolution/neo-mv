#pragma once
#include "core/mask/scores.hpp"
#include "highway/mask_rows.hpp"
#include <array>
namespace neo_mv::simd {
// These kernels consume complete, scene-eligible grids. The input plan owns
// metadata matching and scene eligibility; validation here still rejects forged
// grid shapes, negative SADs and vectors outside the public analysis domain.
template <class T>
class VectorLengthMaskPlan {
public:
  VectorLengthMaskPlan(const AnalysisMetadata& m, float f, float gamma, int time256)
      : grid_(m, f, gamma, time256), maximum_(score_detail::maximum<T>(m.bits)), f2_(score_detail::finite(f * f)),
        g2_(score_detail::finite(gamma / 2.0f)) {}

  // Validated is reserved for an unchanged grid admitted by the input plan.
  template <bool Validated = false>
  std::vector<T> generate(const MotionGrid& grid) const {
    if constexpr (!Validated)
      grid_.validate(grid);
    const auto& m = grid_.metadata();
    std::vector<T> output(grid.values.size());
    std::array<double, 256> xs, ys, scores;
    for (std::size_t first = 0; first < grid.values.size(); first += xs.size()) {
      const auto count = (std::min)(xs.size(), grid.values.size() - first);
      for (std::size_t i = 0; i < count; ++i) {
        xs[i] = grid.values[first + i].vector.x;
        ys[i] = grid.values[first + i].vector.y;
      }
      mask_rows::magnitude(xs.data(), ys.data(), count, m.pel, f2_, g2_, maximum_, scores.data());
      for (std::size_t i = 0; i < count; ++i)
        output[first + i] = mask_detail::quantize<T>(scores[i], m.bits);
    }
    return output;
  }

private:
  score_detail::Grid grid_;
  float maximum_, f2_, g2_;
};

template <class T>
class SADMaskPlan {
public:
  SADMaskPlan(const AnalysisMetadata& m, float f, float gamma, int time256)
      : grid_(m, f, gamma, time256), maximum_(score_detail::maximum<T>(m.bits)), gamma_(gamma),
        hx_(std::int64_t(256 - time256) * 16 / (grid_.step_x() * m.pel)),
        hy_(std::int64_t(256 - time256) * 16 / (grid_.step_y() * m.pel)),
        scale_(score_detail::finite(score_detail::finite(4.0f * f) /
                                    static_cast<float>(std::int64_t(m.block_width) * m.block_height))) {}

  // Validated is reserved for an unchanged grid admitted by the input plan.
  template <bool Validated = false>
  std::vector<T> generate(const MotionGrid& grid) const {
    if constexpr (!Validated)
      grid_.validate(grid);
    const auto& m = grid_.metadata();
    std::vector<T> output;
    output.reserve(grid.values.size());
    std::vector<float> samples(grid.values.size()), scores(grid.values.size());
    const int shift = (std::min)(16, m.bits) - 8;
    for (int by = 0; by < m.blocks_y; ++by) {
      for (int bx = 0; bx < m.blocks_x; ++bx) {
        const auto index = static_cast<std::size_t>(by) * m.blocks_x + bx;
        const auto v = grid.values[index].vector;
        // Signed integer division truncates toward zero, including negative V.
        const auto x = std::int64_t(bx) - std::int64_t(v.x) * hx_ / 4096;
        const auto y = std::int64_t(by) - std::int64_t(v.y) * hy_ / 4096;
        const auto selected = x < 0 || x >= m.blocks_x || y < 0 || y >= m.blocks_y
                                  ? index
                                  : static_cast<std::size_t>(y) * m.blocks_x + static_cast<std::size_t>(x);
        const auto sad = grid.values[selected].error >> shift;
        samples[index] = static_cast<float>(sad);
      }
    }
    mask_rows::sad(samples.data(), samples.size(), scale_, gamma_, maximum_, scores.data());
    for (float score : scores)
      output.push_back(mask_detail::quantize<T>(score, m.bits));
    return output;
  }

private:
  score_detail::Grid grid_;
  float maximum_, gamma_;
  std::int64_t hx_, hy_;
  float scale_;
};

template <class T>
class OcclusionMaskPlan {
  struct ScoreCache {
    std::int64_t overlap = -1;
    T value{};
  };
public:
  OcclusionMaskPlan(const AnalysisMetadata& m, float f, float gamma, int time256)
      : grid_(m, f, gamma, time256), maximum_(score_detail::maximum<T>(m.bits)), gamma_(gamma),
        hx_(std::int64_t(time256) * 16 / (grid_.step_x() * m.pel)),
        hy_(std::int64_t(time256) * 16 / (grid_.step_y() * m.pel)),
        ax_(score_detail::finite(score_detail::finite(80.0f * f) / static_cast<float>(grid_.step_x() * m.pel))),
        ay_(score_detail::finite(score_detail::finite(80.0f * f) / static_cast<float>(grid_.step_y() * m.pel))) {}

  // Validated is reserved for an unchanged grid admitted by the input plan.
  template <bool Validated = false>
  std::vector<T> generate(const MotionGrid& grid) const {
    if constexpr (!Validated)
      grid_.validate(grid);
    const auto& m = grid_.metadata();
    std::vector<T> output(grid.values.size(), T{0});
    ScoreCache horizontal, vertical;
    for (int by = 0; by < m.blocks_y; ++by) {
      for (int bx = 0; bx < m.blocks_x; ++bx) {
        const auto index = static_cast<std::size_t>(by) * m.blocks_x + bx;
        if (bx + 1 < m.blocks_x) {
          const auto overlap = std::int64_t(grid.values[index].vector.x) - grid.values[index + 1].vector.x;
          event(output, overlap, hx_, ax_, bx, m.blocks_x, static_cast<std::size_t>(by) * m.blocks_x, 1, horizontal);
        }
        if (by + 1 < m.blocks_y) {
          const auto overlap = std::int64_t(grid.values[index].vector.y) - grid.values[index + m.blocks_x].vector.y;
          event(output, overlap, hy_, ay_, by, m.blocks_y, static_cast<std::size_t>(bx), m.blocks_x, vertical);
        }
      }
    }
    return output;
  }

private:
#if defined(__GNUC__) || defined(__clang__)
  __attribute__((always_inline))
#endif
  inline void event(std::vector<T>& output, std::int64_t overlap, std::int64_t h, float scale, int position, int count,
             std::size_t start, std::size_t stride, ScoreCache& cache) const {
    if (overlap <= 0)
      return;
    const auto& m = grid_.metadata();
    const auto k = overlap * h / 4096;
    const auto left = m.delta > 0 ? (std::max)(std::int64_t{0}, std::int64_t(position) + 1 - k) : position;
    const auto right =
        m.delta > 0 ? std::int64_t(position) + 1 : (std::min)(std::int64_t(position) + 1 - k, std::int64_t(count) - 1);
    if (left > right)
      return;
    if (cache.overlap != overlap) {
      const float o = static_cast<float>(overlap);
      const float score =
          gamma_ == 1.0f ? score_detail::finite(score_detail::finite(maximum_ * o) * scale)
                         : score_detail::finite(maximum_ * mask_detail::power(score_detail::finite(o * scale), gamma_));
      cache.value = mask_detail::quantize<T>(score, m.bits);
      cache.overlap = overlap;
    }
    const auto value = cache.value;
    if (stride == 1) {
      mask_rows::max_span(output.data() + start + std::size_t(left), std::size_t(right - left + 1), value);
      return;
    }
    for (auto i = left; i <= right; ++i) {
      auto& sample = output[start + static_cast<std::size_t>(i) * stride];
      sample = (std::max)(sample, value);
    }
  }

  score_detail::Grid grid_;
  float maximum_, gamma_;
  std::int64_t hx_, hy_;
  float ax_, ay_;
};

} // namespace neo_mv::simd
