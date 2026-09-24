#pragma once

#include "core/motion/block_sampling.hpp"
#include <vector>

namespace neo_mv {

// MVTools FFT DCT-II distance: quantize each image independently, restore
// the DC weight, then scale the coefficient L1 distance. Workspaces belong
// to one block search; no mutable transform state is shared between frames.
class DctWorkspace {
  int width_, height_, maximum_, midpoint_, scale_;
  std::vector<float> input_, transformed_;
  std::vector<int> source_, reference_;
  void transform(std::vector<int>& output);
  template <class T>
  void load(span2d::Plane<const T> plane) {
    if (plane.width() != width_ || plane.height() != height_)
      throw std::invalid_argument("DCT rectangle size mismatch");
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        const auto value = plane.row(y)[x];
        if (!std::isfinite(double(value)) || value < 0 || value > maximum_)
          throw std::invalid_argument("DCT sample outside integer pixel range");
        input_[std::size_t(y) * width_ + x] = float(value);
      }
  }

public:
  DctWorkspace(int width, int height, int bits);
  template <class T>
  void set_source(span2d::Plane<const T> plane) {
    load(plane);
    transform(source_);
  }
  template <class T>
  std::int64_t compare(span2d::Plane<const T> plane) {
    load(plane);
    transform(reference_);
    std::int64_t distance = 3LL * std::abs(source_[0] - reference_[0]);
    for (std::size_t i = 0; i < source_.size(); ++i)
      distance += std::abs(source_[i] - reference_[i]);
    return distance * scale_ / 2;
  }
};

// Both search backends use this evaluator. In particular, DCT must never
// enter a SAD-only bounded or fused search selected by the Highway backend.
template <class T>
class DctBlockError {
  const SamplingGeometry& geometry_;
  BlockRegion block_;
  const SamplingFrames<T>& frames_;
  DctWorkspace workspace_;

public:
  DctBlockError(const SamplingGeometry& geometry, BlockRegion block, const SamplingFrames<T>& frames, int bits)
      : geometry_(geometry), block_(block), frames_(frames), workspace_(block.width, block.height, bits) {
    if constexpr (std::is_same_v<T, float>)
      throw std::invalid_argument("DCT requires integer samples");
    workspace_.set_source(frames.current[0].subplane(geometry.planes[0].pad_x + block.x,
                                                     geometry.planes[0].pad_y + block.y, block.width, block.height));
  }
  BlockError operator()(MotionVector vector) {
    std::array<std::int64_t, 3> errors{};
    for (int k = 0; k < (geometry_.chroma ? 3 : 1); ++k) {
      const int rx = k == 0 ? 1 : geometry_.ratio_x, ry = k == 0 ? 1 : geometry_.ratio_y;
      const auto tx = std::int64_t(vector.x) / rx, ty = std::int64_t(vector.y) / ry;
      const auto qx = sampling_detail::floor_div(tx, geometry_.pel), qy = sampling_detail::floor_div(ty, geometry_.pel);
      const auto phase = (ty - geometry_.pel * qy) * geometry_.pel + tx - geometry_.pel * qx;
      const int x = geometry_.planes[k].pad_x + block_.x / rx, y = geometry_.planes[k].pad_y + block_.y / ry;
      const auto ref = frames_.reference[k][std::size_t(phase)].subplane(int(x + qx), int(y + qy), block_.width / rx,
                                                                         block_.height / ry);
      if (k == 0)
        errors[k] = workspace_.compare(ref);
      else
        errors[k] = block_metric<T, true>(frames_.current[k].subplane(x, y, block_.width / rx, block_.height / ry), ref,
                                          BlockMetric::sad);
    }
    const auto chroma = metric_detail::accumulate(errors[1], errors[2]);
    return {errors[0], chroma, metric_detail::accumulate(errors[0], chroma)};
  }
};
} // namespace neo_mv
