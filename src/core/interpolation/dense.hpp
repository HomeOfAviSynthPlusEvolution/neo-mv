#pragma once

#include "core/interpolation/input.hpp"
#include "core/mask/scores.hpp"

#include <memory>
#include <mutex>

namespace neo_mv {

struct DenseInterpolationFields {
  int width, height;
  DenseFlowField B, F;
  std::optional<DenseFlowField> BB, FF;
  OverwriteVector<std::uint8_t> mB, mF;
};
using DenseInterpolationField = DenseInterpolationFields;

struct InterpolationMotionFields {
  DenseFlowField B, F;
  std::optional<DenseFlowField> BB, FF;
};

struct SharedInterpolationFields {
  std::shared_ptr<const InterpolationMotionFields> motion;
  OverwriteVector<std::uint8_t> mB, mF;
};

template <class Resampler = GridResamplingPlan>
class DenseInterpolationPlan {
  std::array<AnalysisMetadata, 2> metadata_;
  DenseFlowPlan<Resampler> backward_, forward_;
  Resampler masks_;
  float f_;
  struct CachedMotion {
    std::array<std::vector<MotionVector>, 4> key;
    std::shared_ptr<const InterpolationMotionFields> fields;
  };
  struct Cache {
    std::mutex mutex;
    std::shared_ptr<const CachedMotion> entry;
  };
  // Copies of a plan have identical immutable geometry and may share one entry.
  std::shared_ptr<Cache> cache_ = std::make_shared<Cache>();

  static bool matches(const CachedMotion& entry, const std::array<const MotionGrid*, 4>& grids) {
    for (std::size_t k = 0; k < grids.size(); ++k) {
      const auto* grid = grids[k];
      const auto& key = entry.key[k];
      if (!grid) {
        if (!key.empty())
          return false;
        continue;
      }
      if (key.size() != grid->values.size())
        return false;
      for (std::size_t i = 0; i < key.size(); ++i)
        if (key[i].x != grid->values[i].vector.x || key[i].y != grid->values[i].vector.y)
          return false;
    }
    return true;
  }

  template <bool Validated>
  void validate_inputs(const MotionGrid& backward, const MotionGrid& forward, int time256,
                       const MotionGrid* extra_backward, const MotionGrid* extra_forward) const {
    if (time256 < 0 || time256 > 256 || bool(extra_backward) != bool(extra_forward))
      throw std::invalid_argument("invalid interpolation time or unpaired extra fields");
    if constexpr (!Validated) {
      validate(backward, 0);
      validate(forward, 1);
      if (extra_backward) {
        validate(*extra_backward, 0);
        validate(*extra_forward, 1);
      }
    }
  }

  static float normalization(double ml) {
    const float value = mask_detail::binary32(ml);
    if (value <= 0)
      throw std::invalid_argument("interpolation ml must be positive");
    const float reciprocal = 1.0f / value;
    if (!std::isfinite(reciprocal))
      throw std::invalid_argument("interpolation normalization is not finite");
    return reciprocal;
  }
  static AnalysisMetadata mask_metadata(AnalysisMetadata m) {
    // Only the internal event output precision changes. Public fields are
    // validated with their original metadata before this event kernel runs.
    m.bits = 8;
    return m;
  }
  void validate(const MotionGrid& grid, std::size_t direction) const {
    const auto& m = metadata_[direction];
    if (grid.width != m.blocks_x || grid.height != m.blocks_y || grid.values.size() != field_detail::count(m))
      throw std::invalid_argument("inconsistent interpolation motion grid");
    for (std::size_t i = 0; i < grid.values.size(); ++i)
      field_detail::vector(m, grid.values[i], static_cast<int>(i % m.blocks_x), static_cast<int>(i / m.blocks_x));
  }
  OverwriteVector<std::uint8_t> mask(const MotionGrid& grid, std::size_t direction, int time256) const {
    const auto small = OcclusionMaskPlan<std::uint8_t>(mask_metadata(metadata_[direction]), f_, 1, time256)
                           .template generate<true>(grid);
    const auto& g = backward_.geometry();
    OverwriteVector<std::uint8_t> output(dense_detail::count(g.width, g.height));
    masks_.template resize<std::uint8_t, true>(dense_detail::plane(small.data(), g.blocks_x, g.blocks_y, small.size()),
                                               dense_detail::plane(output.data(), g.width, g.height, output.size()), 8);
    return output;
  }

public:
  DenseInterpolationPlan(AnalysisMetadata backward, AnalysisMetadata forward, int ratio_x, int ratio_y, double ml = 100)
      : metadata_{backward, forward}, backward_(backward, ratio_x, ratio_y), forward_(forward, ratio_x, ratio_y),
        masks_(backward_.geometry()), f_(normalization(ml)) {
    interpolation_detail::validate_pair_metadata(backward, forward);
    // Validate all required constants at creation, including paths whose
    // eventual output falls back or whose interpolation time is an endpoint.
    OcclusionMaskPlan<std::uint8_t>(mask_metadata(backward), f_, 1, 0);
    OcclusionMaskPlan<std::uint8_t>(mask_metadata(forward), f_, 1, 0);
  }
  const GridResamplingGeometry& geometry() const { return backward_.geometry(); }
  float normalization() const { return f_; }
  template <bool Validated = false>
  DenseInterpolationFields generate(const MotionGrid& backward, const MotionGrid& forward, int time256,
                                    const MotionGrid* extra_backward = nullptr,
                                    const MotionGrid* extra_forward = nullptr) const {
    validate_inputs<Validated>(backward, forward, time256, extra_backward, extra_forward);
    const auto& g = geometry();
    DenseInterpolationFields result{g.width,
                                    g.height,
                                    backward_.template generate<true>(backward, 0),
                                    forward_.template generate<true>(forward, 0),
                                    {},
                                    {},
                                    mask(backward, 0, 256 - time256),
                                    mask(forward, 1, time256)};
    if (extra_backward) {
      result.BB = backward_.template generate<true>(*extra_backward, 0);
      result.FF = forward_.template generate<true>(*extra_forward, 0);
    }
    return result;
  }

  template <bool Validated = false>
  SharedInterpolationFields generate_reusing(const MotionGrid& backward, const MotionGrid& forward, int time256,
                                             const MotionGrid* extra_backward = nullptr,
                                             const MotionGrid* extra_forward = nullptr) const {
    // Admission precedes lookup, including SAD values that do not affect motion.
    validate_inputs<Validated>(backward, forward, time256, extra_backward, extra_forward);
    const std::array<const MotionGrid*, 4> grids{&backward, &forward, extra_backward, extra_forward};
    const auto& g = geometry();
    const std::uint64_t fields = extra_backward ? 4 : 2;
    // Bound retained data per plane plan. In-flight requests own their snapshots.
    constexpr std::uint64_t budget = 64 * 1024 * 1024;
    const auto samples = dense_detail::count(g.width, g.height);
    const bool retain =
        samples <= budget / (fields * 2 * sizeof(std::int16_t)) &&
        backward.values.size() <= (budget / fields - samples * 2 * sizeof(std::int16_t)) / sizeof(MotionVector);
    std::shared_ptr<const InterpolationMotionFields> motion;
    if (retain) {
      std::shared_ptr<const CachedMotion> previous;
      {
        std::lock_guard<std::mutex> lock(cache_->mutex);
        previous = cache_->entry;
      }
      if (previous && matches(*previous, grids))
        motion = previous->fields;
    }
    if (!motion) {
      auto generated = std::make_shared<InterpolationMotionFields>();
      generated->B = backward_.template generate<true>(backward, 0);
      generated->F = forward_.template generate<true>(forward, 0);
      if (extra_backward) {
        generated->BB = backward_.template generate<true>(*extra_backward, 0);
        generated->FF = forward_.template generate<true>(*extra_forward, 0);
      }
      motion = std::move(generated);
      if (retain) {
        auto entry = std::make_shared<CachedMotion>();
        entry->fields = motion;
        for (std::size_t k = 0; k < grids.size(); ++k)
          if (grids[k]) {
            entry->key[k].reserve(grids[k]->values.size());
            for (const auto& value : grids[k]->values)
              entry->key[k].push_back(value.vector);
          }
        // Generation and comparison stay outside the lock. Duplicate concurrent
        // misses are harmless; publishing never invalidates an in-flight result.
        std::shared_ptr<const CachedMotion> replaced = std::move(entry);
        {
          std::lock_guard<std::mutex> lock(cache_->mutex);
          replaced.swap(cache_->entry);
        }
      }
    }
    return {std::move(motion), mask(backward, 0, 256 - time256), mask(forward, 1, time256)};
  }
};

} // namespace neo_mv
