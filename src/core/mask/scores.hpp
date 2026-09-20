#pragma once

#include "core/mask/numeric.hpp"
#include "core/motion/analysis_field.hpp"

#include <cmath>
#include <vector>

namespace neo_mv {
namespace score_detail {
template <class F>
F finite(F value) {
  if (!std::isfinite(value))
    throw std::overflow_error("non-finite mask score intermediate");
  return value;
}

class Grid {
public:
  Grid(const AnalysisMetadata& metadata, float f, float gamma, int time256) : m_(metadata) {
    if (!valid_analysis_metadata(m_) || !std::isfinite(f) || f < 0 || !std::isfinite(gamma) || gamma < 0 ||
        time256 < 0 || time256 > 256)
      throw std::invalid_argument("invalid mask score parameters");
    const auto width = std::int64_t(m_.blocks_x - 1) * step_x() + m_.block_width;
    const auto height = std::int64_t(m_.blocks_y - 1) * step_y() + m_.block_height;
    if (width < m_.real_width || width > m_.width || height < m_.real_height || height > m_.height)
      throw std::invalid_argument("invalid mask grid coverage");
  }
  const AnalysisMetadata& metadata() const { return m_; }
  std::int64_t step_x() const { return m_.block_width - m_.overlap_x; }
  std::int64_t step_y() const { return m_.block_height - m_.overlap_y; }
  void validate(const MotionGrid& grid) const {
    if (grid.width != m_.blocks_x || grid.height != m_.blocks_y || grid.values.size() != field_detail::count(m_))
      throw std::invalid_argument("inconsistent mask motion grid");
    for (std::size_t i = 0; i < grid.values.size(); ++i)
      field_detail::vector(m_, grid.values[i], static_cast<int>(i % m_.blocks_x), static_cast<int>(i / m_.blocks_x));
  }

private:
  AnalysisMetadata m_;
};

template <class T>
float maximum(int bits) {
  // Also rejects a storage type inconsistent with the analysis precision.
  mask_detail::validate_storage<T>(bits);
  return bits == 32 ? 1.0f : static_cast<float>((std::uint32_t{1} << bits) - 1);
}
} // namespace score_detail

// These kernels consume complete, scene-eligible grids. The input plan owns
// metadata matching and scene eligibility; validation here still rejects forged
// grid shapes, negative SADs and vectors outside the public analysis domain.
template <class T>
class VectorLengthMaskPlan {
public:
  VectorLengthMaskPlan(const AnalysisMetadata& m, float f, float gamma, int time256)
      : grid_(m, f, gamma, time256), maximum_(score_detail::maximum<T>(m.bits)), f2_(score_detail::finite(f * f)),
        g2_(score_detail::finite(gamma / 2.0f)) {}

  std::vector<T> generate(const MotionGrid& grid) const {
    grid_.validate(grid);
    const auto& m = grid_.metadata();
    std::vector<T> output;
    output.reserve(grid.values.size());
    for (const auto& value : grid.values) {
      const double x = static_cast<double>(value.vector.x) / m.pel;
      const double y = static_cast<double>(value.vector.y) / m.pel;
      const double xx = score_detail::finite(x * x), yy = score_detail::finite(y * y);
      const double q = score_detail::finite(xx + yy);
      const double a = score_detail::finite(q * static_cast<double>(f2_));
      const double score =
          score_detail::finite(static_cast<double>(maximum_) * mask_detail::power(a, static_cast<double>(g2_)));
      output.push_back(mask_detail::quantize<T>(score, m.bits));
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

  std::vector<T> generate(const MotionGrid& grid) const {
    grid_.validate(grid);
    const auto& m = grid_.metadata();
    std::vector<T> output;
    output.reserve(grid.values.size());
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
        const float z = score_detail::finite(static_cast<float>(sad) * scale_);
        const float score = score_detail::finite(maximum_ * mask_detail::power(z, gamma_));
        output.push_back(mask_detail::quantize<T>(score, m.bits));
      }
    }
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
public:
  OcclusionMaskPlan(const AnalysisMetadata& m, float f, float gamma, int time256)
      : grid_(m, f, gamma, time256), maximum_(score_detail::maximum<T>(m.bits)), gamma_(gamma),
        hx_(std::int64_t(time256) * 16 / (grid_.step_x() * m.pel)),
        hy_(std::int64_t(time256) * 16 / (grid_.step_y() * m.pel)),
        ax_(score_detail::finite(score_detail::finite(80.0f * f) / static_cast<float>(grid_.step_x() * m.pel))),
        ay_(score_detail::finite(score_detail::finite(80.0f * f) / static_cast<float>(grid_.step_y() * m.pel))) {}

  std::vector<T> generate(const MotionGrid& grid) const {
    grid_.validate(grid);
    const auto& m = grid_.metadata();
    std::vector<T> output(grid.values.size(), T{0});
    for (int by = 0; by < m.blocks_y; ++by) {
      for (int bx = 0; bx < m.blocks_x; ++bx) {
        const auto index = static_cast<std::size_t>(by) * m.blocks_x + bx;
        if (bx + 1 < m.blocks_x) {
          const auto overlap = std::int64_t(grid.values[index].vector.x) - grid.values[index + 1].vector.x;
          event(output, overlap, hx_, ax_, bx, m.blocks_x, static_cast<std::size_t>(by) * m.blocks_x, 1);
        }
        if (by + 1 < m.blocks_y) {
          const auto overlap = std::int64_t(grid.values[index].vector.y) - grid.values[index + m.blocks_x].vector.y;
          event(output, overlap, hy_, ay_, by, m.blocks_y, static_cast<std::size_t>(bx), m.blocks_x);
        }
      }
    }
    return output;
  }

private:
  void event(std::vector<T>& output, std::int64_t overlap, std::int64_t h, float scale, int position, int count,
             std::size_t start, std::size_t stride) const {
    if (overlap <= 0)
      return;
    const auto& m = grid_.metadata();
    const auto k = overlap * h / 4096;
    const auto left = m.delta > 0 ? (std::max)(std::int64_t{0}, std::int64_t(position) + 1 - k) : position;
    const auto right =
        m.delta > 0 ? std::int64_t(position) + 1 : (std::min)(std::int64_t(position) + 1 - k, std::int64_t(count) - 1);
    if (left > right)
      return;
    const float o = static_cast<float>(overlap);
    const float score =
        gamma_ == 1.0f ? score_detail::finite(score_detail::finite(maximum_ * o) * scale)
                       : score_detail::finite(maximum_ * mask_detail::power(score_detail::finite(o * scale), gamma_));
    const auto value = mask_detail::quantize<T>(score, m.bits);
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
} // namespace neo_mv
