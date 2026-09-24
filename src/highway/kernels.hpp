#pragma once
#include "core/motion/block_sampling.hpp"
#include "core/motion/analyse.hpp"
#include "core/super/border_extension.hpp"
#include "core/super/pyramid_reduction.hpp"
#include "core/super/subpixel.hpp"
#include "highway/rows.hpp"
#include "highway/prediction.hpp"

#include <optional>

namespace neo_mv::simd {
template <class T>
constexpr std::int64_t storage_max() {
  if constexpr (std::is_same_v<T, float>)
    return 0;
  else
    return std::numeric_limits<T>::max();
}
template <class T, bool Validated = false>
void extend_border(span2d::Plane<const T> src, span2d::Plane<T> dst, int ww, int hh, int hp, int vp) {
  if constexpr (!Validated) {
    validate_plane(src);
    validate_plane(dst);
    if (ww < src.width() || hh < src.height() || hp < 0 || vp < 0)
      throw std::invalid_argument("invalid border geometry");
    const auto w = std::int64_t(ww) + 2LL * hp, h = std::int64_t(hh) + 2LL * vp;
    if (w > INT32_MAX || h > INT32_MAX)
      throw std::overflow_error("padded dimensions exceed view range");
    if (w != dst.width() || h != dst.height() || active_rows_overlap(src, dst))
      throw std::invalid_argument("invalid border output");
    if constexpr (std::is_same_v<T, float>)
      for (int y = 0; y < src.height(); ++y)
        detail::scan(src.row(y).data(), src.width(), 0);
  }
  for (int y = 0; y < dst.height(); ++y) {
    const auto* row = src.row(std::clamp(y - vp, 0, src.height() - 1)).data();
    auto* out = dst.row(y).data();
    detail::fill(row[0], out, hp);
    detail::copy(row, out + hp, src.width());
    detail::fill(row[src.width() - 1], out + hp + src.width(), dst.width() - hp - src.width());
  }
}
template <class T, bool Validated = false>
std::int64_t block_metric(span2d::Plane<const T> a, span2d::Plane<const T> b, BlockMetric op) {
  if constexpr (!Validated) {
    validate_plane(a);
    validate_plane(b);
    if (a.width() != b.width() || a.height() != b.height() || (op != BlockMetric::sad && op != BlockMetric::satd))
      throw std::invalid_argument("invalid metric input");
    if (op == BlockMetric::satd && (a.width() % 4 || a.height() % 4))
      throw std::invalid_argument("invalid SATD rectangle");
  }
  return detail::metric(a.data(), a.stride_bytes() / sizeof(T), b.data(), b.stride_bytes() / sizeof(T), a.width(),
                        a.height(), op == BlockMetric::satd);
}
template <class T, bool Validated = false>
void reduce_pyramid(span2d::Plane<const T> src, int ox, int oy, span2d::Plane<T> dst, int filter,
                    span2d::Plane<T> tmp = {}) {
  if constexpr (!Validated) {
    validate_plane(src);
    validate_plane(dst);
    if (filter < 0 || filter > 2 || ox < 0 || oy < 0 || ox + 2LL * dst.width() > src.width() ||
        oy + 2LL * dst.height() > src.height())
      throw std::invalid_argument("invalid reduction geometry");
    if (active_rows_overlap(src, dst))
      throw std::invalid_argument("reduction overlaps input");
    if (filter) {
      validate_plane(tmp);
      if (tmp.width() != 2LL * dst.width() || tmp.height() != dst.height() || active_rows_overlap(src, tmp) ||
          active_rows_overlap(dst, tmp))
        throw std::invalid_argument("invalid reduction scratch");
    }
    if constexpr (std::is_same_v<T, float>)
      for (int y = 0; y < 2 * dst.height(); ++y)
        detail::scan(src.row(oy + y).data() + ox, 2 * dst.width(), 0);
  }
  const auto maximum = storage_max<T>();
  if (filter == 0) {
    for (int y = 0; y < dst.height(); ++y) {
      const auto* a = src.row(oy + 2 * y).data() + ox;
      const auto* b = src.row(oy + 2 * y + 1).data() + ox;
      const T* taps[]{a, a + 1, b + 1, b};
      // The last odd tap lacks the extra deinterleave partner: scalar final pixel.
      detail::formula(taps, 2, dst.row(y).data(), dst.width() - 1, detail::Formula::four_average, maximum);
      const int x = 2 * (dst.width() - 1);
      using A = reduction_detail::Accumulator<T>;
      dst.row(y)[dst.width() - 1] = reduction_detail::rounded<T>(((A(a[x]) + A(a[x + 1])) + A(b[x + 1])) + A(b[x]), 2);
    }
    return;
  }
  for (int y = 0; y < dst.height(); ++y) {
    const T* taps[6]{};
    detail::Formula op;
    if (y == 0 || y == dst.height() - 1) {
      op = detail::Formula::average;
      taps[0] = src.row(oy + 2 * y).data() + ox;
      taps[1] = src.row(oy + 2 * y + 1).data() + ox;
    } else {
      const int count = filter == 1 ? 4 : 6, first = filter == 1 ? -1 : -2;
      op = filter == 1 ? detail::Formula::reduce4 : detail::Formula::reduce6;
      for (int i = 0; i < count; ++i)
        taps[i] = src.row(oy + 2 * y + first + i).data() + ox;
    }
    detail::formula(taps, 1, tmp.row(y).data(), tmp.width(), op, maximum);
  }
  for (int y = 0; y < dst.height(); ++y) {
    const auto* row = tmp.row(y).data();
    auto* out = dst.row(y).data();
    const int w = dst.width();
    auto scalar = [&](int x) {
      out[x] = reduction_detail::filter_axis<T>([&](int i) { return row[i]; }, x, w, filter);
    };
    scalar(0);
    const int end = std::max(1, w - 2);
    const int count = filter == 1 ? 4 : 6, first = filter == 1 ? -1 : -2;
    if (end > 1) {
      const T* taps[6]{};
      for (int i = 0; i < count; ++i)
        taps[i] = row + 2 + first + i;
      detail::formula(taps, 2, out + 1, end - 1, filter == 1 ? detail::Formula::reduce4 : detail::Formula::reduce6,
                      maximum);
    }
    for (int x = end; x < w; ++x)
      scalar(x);
  }
}
namespace detail {
template <class T>
void half_horizontal(span2d::Plane<const T> src, span2d::Plane<T> dst, int sharp, std::int64_t max) {
  for (int y = 0; y < src.height(); ++y) {
    const auto* row = src.row(y).data();
    auto* out = dst.row(y).data();
    const int w = src.width();
    int first = sharp == 0 ? 0 : sharp, end = sharp == 0 ? w - 1 : w - (sharp + 2);
    auto scalar = [&](int x) {
      out[x] = subpixel_detail::half<T>([&](int i) { return row[i]; }, x, w, sharp, false, max);
    };
    for (int x = 0; x < first; ++x)
      scalar(x);
    const T* taps[6]{};
    const int count = sharp == 0 ? 2 : sharp == 1 ? 4 : 6;
    const int offset = sharp == 0 ? 0 : -sharp;
    for (int i = 0; i < count; ++i)
      taps[i] = row + first + offset + i;
    if (end > first)
      formula(taps, 1, out + first, end - first,
              sharp == 0   ? Formula::average
              : sharp == 1 ? Formula::sharp4h
                           : Formula::sharp6,
              max);
    for (int x = std::max(first, end); x < w; ++x)
      scalar(x);
  }
}
template <class T>
void half_vertical(span2d::Plane<const T> src, span2d::Plane<T> dst, int sharp, std::int64_t max) {
  for (int y = 0; y < src.height(); ++y) {
    if (y == src.height() - 1) {
      copy(src.row(y).data(), dst.row(y).data(), src.width());
      continue;
    }
    const bool interior = sharp > 0 && y >= sharp && y < src.height() - (sharp + 2);
    const int count = interior ? (sharp == 1 ? 4 : 6) : 2, offset = interior ? -sharp : 0;
    const T* taps[6]{};
    for (int i = 0; i < count; ++i)
      taps[i] = src.row(y + offset + i).data();
    formula(taps, 1, dst.row(y).data(), src.width(),
            !interior    ? Formula::average
            : sharp == 1 ? Formula::sharp4v
                         : Formula::sharp6,
            max);
  }
}
} // namespace detail
template <class T, bool Validated = false>
SubpixelPhases<T> interpolate_subpixels(span2d::Plane<const T> base, int pel, int sharp, int bits,
                                        const std::array<span2d::Plane<T>, 16>& storage) {
  const auto max = subpixel_detail::sample_max<T>(bits);
  const int w = base.width(), h = base.height();
  if constexpr (!Validated) {
    subpixel_detail::validate_storage(base, pel, storage);
    if (sharp < 0 || sharp > 2)
      throw std::invalid_argument("invalid sharpness");
    if (pel > 1 && (w < 2 * (sharp + 1) || h < 2 * (sharp + 1)))
      throw std::invalid_argument("small interpolation plane");
    for (int y = 0; y < h; ++y)
      detail::scan(base.row(y).data(), w, max);
  }
  SubpixelPhases<T> result{pel, {}};
  result.planes[0] = base;
  if (pel == 1)
    return result;
  const int half = pel / 2;
  auto horizontal = storage[half], vertical = storage[half * pel], diag = storage[half * pel + half];
  detail::half_horizontal(base, horizontal, sharp, max);
  detail::half_vertical(base, vertical, sharp, max);
  if (sharp)
    detail::half_horizontal(span2d::Plane<const T>(vertical), diag, sharp, max);
  else
    for (int y = 0; y < h; ++y) {
      const auto* a = base.row(y).data();
      const auto* b = base.row(std::min(y + 1, h - 1)).data();
      const T* taps[]{a, a + 1, b, b + 1};
      if (y == h - 1)
        detail::formula(taps, 1, diag.row(y).data(), w - 1, detail::Formula::average, max);
      else
        detail::formula(taps, 1, diag.row(y).data(), w - 1, detail::Formula::four_average, max);
      diag.row(y)[w - 1] = y == h - 1 ? a[w - 1] : subpixel_detail::average(a[w - 1], b[w - 1], max);
    }
  if (pel == 4) {
    auto phase = [&](int x, int y) -> span2d::Plane<const T> {
      return x == 0 && y == 0 ? base : span2d::Plane<const T>(storage[y * 4 + x]);
    };
    auto blend = [&](int x, int y, int lx, int ly, int rx, int ry, int dx = 0, int dy = 0) {
      auto left = phase(lx, ly), right = phase(rx, ry);
      auto dst = storage[y * 4 + x];
      for (int row = 0; row < h - (y == 3); ++row) {
        const T* taps[]{left.row(row + dy).data() + dx, right.row(row).data()};
        detail::formula(taps, 1, dst.row(row).data(), w - (x == 3), detail::Formula::average, max);
      }
    };
    blend(1, 0, 0, 0, 2, 0);
    blend(1, 2, 0, 2, 2, 2);
    blend(0, 1, 0, 0, 0, 2);
    blend(2, 1, 2, 0, 2, 2);
    blend(1, 1, 0, 1, 2, 1);
    blend(3, 0, 0, 0, 2, 0, 1);
    blend(3, 2, 0, 2, 2, 2, 1);
    blend(3, 1, 0, 1, 2, 1, 1);
    blend(0, 3, 0, 0, 0, 2, 0, 1);
    blend(2, 3, 2, 0, 2, 2, 0, 1);
    blend(1, 3, 0, 3, 2, 3);
    blend(3, 3, 0, 3, 2, 3, 1);
  }
  for (int y = 0; y < pel; ++y)
    for (int x = 0; x < pel; ++x)
      if (x || y)
        result.planes[y * pel + x] =
            storage[y * pel + x].subplane(0, 0, w - (pel == 4 && x == 3), h - (pel == 4 && y == 3));
  return result;
}
template <class T, bool Validated = false>
BlockError block_error(const SamplingGeometry& g, BlockRegion b, const SamplingFrames<T>& frames, MotionVector vector,
                       BlockMetric metric) {
  if constexpr (!Validated) {
    validate_sampling_domain(g, b, sampling_detail::singleton(vector));
    if ((metric != BlockMetric::sad && metric != BlockMetric::satd) ||
        (metric == BlockMetric::satd && (b.width % 4 || b.height % 4)))
      throw std::invalid_argument("invalid block error metric or SATD dimensions");
    validate_sampling_frames(g, frames);
  }
  std::array<std::int64_t, 3> errors{};
  std::array<detail::MetricRequest<T>, 3> requests{};
  const auto phase_quotient = [pel = g.pel](std::int64_t value) {
    switch (pel) {
    case 1: return value;
    case 2: return value >= 0 ? value / 2 : -((-value + 1) / 2);
    case 4: return value >= 0 ? value / 4 : -((-value + 3) / 4);
    default: return sampling_detail::floor_div(value, pel);
    }
  };
  const auto evaluate_plane = [&](int k, int bx, int by, int bw, int bh, std::int64_t qx, std::int64_t qy,
                                  std::size_t phase) {
    const int x = g.planes[k].pad_x + bx;
    const int y = g.planes[k].pad_y + by;
    const auto& current = frames.current[k];
    const auto& reference = frames.reference[k][phase];
    if constexpr (Validated) {
      // The plan admits every candidate footprint and the frame views before search.
      const auto* source = current.row(y).data() + x;
      const auto* target = reference.row(static_cast<int>(y + qy)).data() + static_cast<int>(x + qx);
      requests[k] = {source, current.stride(), target, reference.stride(), bw, bh,
                     k == 0 && metric == BlockMetric::satd};
    } else {
      const auto source = current.subplane(x, y, bw, bh);
      const auto target = reference.subplane(static_cast<int>(x + qx), static_cast<int>(y + qy), bw, bh);
      errors[k] = neo_mv::simd::block_metric<T, true>(source, target, k == 0 ? metric : BlockMetric::sad);
    }
  };
  const auto luma_x = phase_quotient(vector.x);
  const auto luma_y = phase_quotient(vector.y);
  const auto luma_phase = std::size_t((vector.y - g.pel * luma_y) * g.pel + vector.x - g.pel * luma_x);
  evaluate_plane(0, b.x, b.y, b.width, b.height, luma_x, luma_y, luma_phase);
  if (g.chroma) {
    const int bx = g.ratio_x == 2 ? b.x / 2 : b.x, by = g.ratio_y == 2 ? b.y / 2 : b.y;
    const int bw = g.ratio_x == 2 ? b.width / 2 : b.width, bh = g.ratio_y == 2 ? b.height / 2 : b.height;
    const auto tx = g.ratio_x == 2 ? std::int64_t(vector.x) / 2 : vector.x;
    const auto ty = g.ratio_y == 2 ? std::int64_t(vector.y) / 2 : vector.y;
    const auto qx = phase_quotient(tx), qy = phase_quotient(ty);
    const auto phase = std::size_t((ty - g.pel * qy) * g.pel + tx - g.pel * qx);
    evaluate_plane(1, bx, by, bw, bh, qx, qy, phase);
    evaluate_plane(2, bx, by, bw, bh, qx, qy, phase);
  }
  if constexpr (Validated)
    detail::metric_batch(requests.data(), g.chroma ? 3 : 1, errors.data());
  const auto chroma = metric_detail::accumulate(errors[1], errors[2]);
  return {errors[0], chroma, metric_detail::accumulate(errors[0], chroma)};
}

// Immutable phase pointers and strides are shared by all blocks in one layer.
template <class T>
struct PreparedSamplingFrames : detail::MetricReferenceFrames<T> {
  std::array<span2d::Plane<const T>, 3> current{};

  PreparedSamplingFrames(const SamplingGeometry& g, const SamplingFrames<T>& frames) : current(frames.current) {
    for (int k = 0; k < (g.chroma ? 3 : 1); ++k)
      for (int a = 0; a < g.pel * g.pel; ++a) {
        this->references[k][a] = frames.reference[k][a].data();
        this->strides[k][a] = frames.reference[k][a].stride();
      }
  }
};

// One search block owns this evaluator; its caller has already admitted the
// complete candidate domain and all frame views. Search invokes it serially.
template <class T, bool OwnsFrames = true>
class PreparedBlockError {
  using FrameInput = std::conditional_t<OwnsFrames, SamplingFrames<T>, PreparedSamplingFrames<T>>;
  using FrameStorage = std::conditional_t<OwnsFrames, PreparedSamplingFrames<T>, const PreparedSamplingFrames<T>*>;
  int ratio_x_, ratio_y_;
  bool chroma_;
  FrameStorage frames_;
  detail::MotionMetricRequest<T> block_;
  detail::MetricBatchFunction<T> metric_batch_;
  detail::BoundedMotionMetricFunction<T> bounded_metric_batch_ = nullptr;

  static FrameStorage store_frames(const SamplingGeometry& g, const FrameInput& frames) {
    if constexpr (OwnsFrames)
      return PreparedSamplingFrames<T>(g, frames);
    else
      return &frames;
  }
  const PreparedSamplingFrames<T>& frames() const {
    if constexpr (OwnsFrames)
      return frames_;
    else
      return *frames_;
  }

  std::int64_t quotient(std::int64_t value) const {
    switch (block_.pel) {
    case 1: return value;
    case 2: return value >= 0 ? value / 2 : -((-value + 1) / 2);
    case 4: return value >= 0 ? value / 4 : -((-value + 3) / 4);
    default: return sampling_detail::floor_div(value, block_.pel);
    }
  }
  void reference(int k, std::int64_t qx, std::int64_t qy, std::size_t phase) {
    block_.planes[k].reference = frames().references[k][phase] +
        (std::ptrdiff_t(block_.y[k]) + qy) * frames().strides[k][phase] + (std::ptrdiff_t(block_.x[k]) + qx);
    block_.planes[k].reference_stride = frames().strides[k][phase];
  }
  void reference_pair(int first, int last, std::int64_t vx, std::int64_t vy) {
    const auto qx = quotient(vx), qy = quotient(vy);
    const auto phase = std::size_t((vy - block_.pel * qy) * block_.pel + vx - block_.pel * qx);
    for (int k = first; k < last; ++k)
      reference(k, qx, qy, phase);
  }
  void prepare_references(MotionVector vector) {
    if (block_.pel == 2 && chroma_ && ratio_x_ == 2 && ratio_y_ == 2) {
      const auto qx = (std::int64_t(vector.x) - (vector.x < 0)) / 2;
      const auto qy = (std::int64_t(vector.y) - (vector.y < 0)) / 2;
      reference(0, qx, qy, std::size_t((vector.y - 2 * qy) * 2 + vector.x - 2 * qx));
      const auto tx = std::int64_t(vector.x) / 2, ty = std::int64_t(vector.y) / 2;
      const auto cx = (tx - (tx < 0)) / 2, cy = (ty - (ty < 0)) / 2;
      const auto phase = std::size_t((ty - 2 * cy) * 2 + tx - 2 * cx);
      reference(1, cx, cy, phase);
      reference(2, cx, cy, phase);
    } else {
      reference_pair(0, 1, vector.x, vector.y);
      if (chroma_) {
        const auto tx = ratio_x_ == 2 ? std::int64_t(vector.x) / 2 : vector.x;
        const auto ty = ratio_y_ == 2 ? std::int64_t(vector.y) / 2 : vector.y;
        reference_pair(1, 3, tx, ty);
      }
    }
  }

public:
  PreparedBlockError(const SamplingGeometry& g, BlockRegion block, const FrameInput& frame_input, BlockMetric metric)
      : ratio_x_(g.ratio_x), ratio_y_(g.ratio_y), chroma_(g.chroma),
        frames_(store_frames(g, frame_input)) {
    block_.pel = g.pel;
    if constexpr (std::is_same_v<T, float>)
      metric_batch_ = detail::metric_batch_function(static_cast<T *>(nullptr));
    else if (chroma_ && ratio_x_ == 2 && ratio_y_ == 2 && metric == BlockMetric::sad && block.width == block.height) {
      metric_batch_ = block.width == 16 ? detail::metric_batch_420_function(static_cast<T *>(nullptr))
                     : block.width == 8 ? detail::metric_batch_420_small_function(static_cast<T *>(nullptr))
                                        : detail::metric_batch_function(static_cast<T *>(nullptr));
      if (block.width == 16 || block.width == 8)
        bounded_metric_batch_ = detail::motion_metric_420_bounded_function(static_cast<T *>(nullptr), block.width);
    } else
      metric_batch_ = detail::metric_batch_function(static_cast<T *>(nullptr));
    for (int k = 0; k < (chroma_ ? 3 : 1); ++k) {
      const int rx = k == 0 ? 1 : ratio_x_, ry = k == 0 ? 1 : ratio_y_;
      block_.x[k] = g.planes[k].pad_x + block.x / rx;
      block_.y[k] = g.planes[k].pad_y + block.y / ry;
      auto& request = block_.planes[k];
      request.source = frames().current[k].row(block_.y[k]).data() + block_.x[k];
      request.source_stride = frames().current[k].stride();
      request.width = block.width / rx;
      request.height = block.height / ry;
      request.satd = k == 0 && metric == BlockMetric::satd;
    }
  }
  BlockError operator()(MotionVector vector) {
    prepare_references(vector);
    std::array<std::int64_t, 3> errors{};
    metric_batch_(block_.planes.data(), chroma_ ? 3 : 1, errors.data());
    const auto chroma = metric_detail::accumulate(errors[1], errors[2]);
    return {errors[0], chroma, metric_detail::accumulate(errors[0], chroma)};
  }
  std::optional<BlockError> bounded(MotionVector vector, std::int64_t limit, MotionVector predictor,
                                    std::int64_t lambda, int penalty) {
    const auto dx = std::int64_t(vector.x) - predictor.x, dy = std::int64_t(vector.y) - predictor.y;
    // The selected 420 blocks have at most 384 samples. With these bounds,
    // distance, full SAD and both penalties fit int64; otherwise retain the
    // ordinary path and its overflow errors.
    if (!bounded_metric_batch_ || lambda < 0 || lambda > INT32_MAX || penalty < 0 || penalty > 256 ||
        dx < -32767 || dx > 32767 || dy < -32767 || dy > 32767)
      return operator()(vector);
    if (limit <= 0)
      return std::nullopt;
    std::array<std::int64_t, 3> errors{};
    if (!bounded_metric_batch_(block_, frames(), vector.x, vector.y, limit, errors.data()))
      return std::nullopt;
    const auto chroma = metric_detail::accumulate(errors[1], errors[2]);
    return BlockError{errors[0], chroma, metric_detail::accumulate(errors[0], chroma)};
  }
  SearchResult analyse(MotionTriple predictor, const SpatialPredictors& spatial, MotionVector zero,
                       CandidateDomain omega, int layer, int pel, std::int64_t lambda,
                       std::int64_t bad_threshold, AnalyseControls controls) {
    if constexpr (std::is_integral_v<T>) {
      // Prove the cost bound once for all candidates, including out-of-domain
      // seeds. Integer 6/16-square 420 SAD, integer 6-square gray SAD and byte 4/8/16-square gray SAD are at most
      // 384*65535; with these vector and lambda bounds the distance product
      // is below 2^62. Penalties and
      // their sum cannot overflow. Other cases retain checked evaluation.
      const auto near = [&](std::int64_t x, std::int64_t y) {
        const auto dx = x - predictor.vector.x, dy = y - predictor.vector.y;
        return dx >= -32767 && dx <= 32767 && dy >= -32767 && dy <= 32767;
      };
      const bool gray = !chroma_ &&
          !block_.planes[0].satd && block_.planes[0].width == block_.planes[0].height &&
          (block_.planes[0].width == 6 || (std::is_same_v<T, std::uint8_t> &&
           (block_.planes[0].width == 4 || block_.planes[0].width == 8 || block_.planes[0].width == 16)));
      const bool six420 = chroma_ && ratio_x_ == 2 && ratio_y_ == 2 &&
          !block_.planes[0].satd && block_.planes[0].width == 6 && block_.planes[0].height == 6;
      bool admitted = (gray || six420 || (bounded_metric_batch_ && block_.planes[0].width == 16)) &&
          pel == block_.pel && lambda >= 0 && lambda <= INT32_MAX &&
          omega.left >= INT32_MIN && omega.top >= INT32_MIN && omega.right <= std::int64_t(INT32_MAX) + 1 &&
          omega.bottom <= std::int64_t(INT32_MAX) + 1 && omega.left < omega.right && omega.top < omega.bottom &&
          near(omega.left, omega.top) && near(omega.right - 1, omega.bottom - 1) && near(zero.x, zero.y) &&
          near(spatial.global.x, spatial.global.y) && controls.search >= 0 && controls.search <= 5 &&
          controls.pelsearch > 0 && controls.pnew >= 0 && controls.pnew <= 256 &&
          controls.pzero >= 0 && controls.pzero <= 256 && controls.pglobal >= 0 && controls.pglobal <= 256;
      for (const auto& p : spatial.p)
        admitted = admitted && near(p.vector.x, p.vector.y);
      if (admitted) {
        if (gray) {
          const auto execute = detail::analyse_block_gray_function(static_cast<T*>(nullptr));
          return execute(block_, frames(), predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
        }
        const auto execute = detail::analyse_block_420_function(static_cast<T*>(nullptr));
        return execute(block_, frames(), predictor, spatial, zero, omega, layer, lambda, bad_threshold, controls);
      }
    }
    return analyse_detail::block(predictor, spatial, zero, omega, layer, pel, lambda, bad_threshold, controls, *this);
  }
};

template <class T>
SubpixelPhases<T> extract_external_subpixels(span2d::Plane<const T> base, span2d::Plane<const T> external,
                                             int actual_width, int actual_height, int pad_x, int pad_y, int pel,
                                             int bits, const std::array<span2d::Plane<T>, 16>& storage) {
  using namespace subpixel_detail;
  validate_storage(base, pel, storage);
  const auto maximum = sample_max<T>(bits);
  if (actual_width <= 0 || actual_height <= 0 || pad_x < 0 || pad_y < 0 ||
      static_cast<std::int64_t>(actual_width) + 2LL * pad_x > base.width() ||
      static_cast<std::int64_t>(actual_height) + 2LL * pad_y > base.height())
    throw std::invalid_argument("invalid external phase geometry");
  SubpixelPhases<T> result{pel, {}};
  result.planes[0] = base;
  for (int y = 0; y < base.height(); ++y)
    detail::scan(base.row(y).data(), base.width(), maximum);
  if (pel == 1)
    return result; // The external pixels are unused for pel=1.
  validate_plane(external);
  if (external.width() != static_cast<std::int64_t>(pel) * actual_width ||
      external.height() != static_cast<std::int64_t>(pel) * actual_height)
    throw std::invalid_argument("incorrect external pel dimensions");
  for (int i = 1; i < pel * pel; ++i)
    if (active_rows_overlap(external, storage[i]))
      throw std::invalid_argument("phase output overwrites external input");
  for (int ay = 0; ay < pel; ++ay)
    for (int ax = 0; ax < pel; ++ax) {
      if (ax == 0 && ay == 0)
        continue;
      auto dst = storage[ay * pel + ax];
      for (int y = 0; y < dst.height(); ++y) {
        const int sy = pel * std::clamp(y - pad_y, 0, actual_height - 1) + ay;
        const auto* input = external.row(sy).data();
        auto* output = dst.row(y).data();
        detail::extract(input, output + pad_x, actual_width, pel, ax);
        detail::scan(output + pad_x, actual_width, maximum);
        detail::fill(output[pad_x], output, pad_x);
        detail::fill(output[pad_x + actual_width - 1], output + pad_x + actual_width,
                     dst.width() - pad_x - actual_width);
      }
      result.planes[ay * pel + ax] = dst;
    }
  return result;
}

} // namespace neo_mv::simd

namespace neo_mv {
template <class T>
struct HighwayKernels {
  static constexpr auto interpolate_predictions = &simd::interpolate_predictions;
  static decltype(auto) prepare_frames(const SamplingGeometry& geometry, const SamplingFrames<T>& frames) {
    if constexpr (std::is_same_v<T, std::uint8_t>)
      return simd::PreparedSamplingFrames<T>{geometry, frames};
    else
      return (frames);
  }
  static simd::PreparedBlockError<T> prepare_block_error(const SamplingGeometry& geometry, BlockRegion block,
                                                         const SamplingFrames<T>& frames, BlockMetric metric) {
    return {geometry, block, frames, metric};
  }
  static simd::PreparedBlockError<T, false> prepare_block_error(const SamplingGeometry& geometry, BlockRegion block,
                                                                const simd::PreparedSamplingFrames<T>& frames,
                                                                BlockMetric metric) {
    return {geometry, block, frames, metric};
  }
  static void validate_samples(const T* samples, int count, std::int64_t maximum) {
    simd::detail::scan(samples, count, maximum);
  }
  static constexpr auto extend_border = &simd::extend_border<T>;
  static constexpr auto reduce_pyramid = &simd::reduce_pyramid<T>;
  static constexpr auto interpolate_subpixels = &simd::interpolate_subpixels<T>;
  // SuperPlan admits geometry; SuperPyramid owns disjoint output buffers and
  // admits source samples once. Each producer still checks new arithmetic.
  static constexpr auto extend_border_validated = &simd::extend_border<T, true>;
  static constexpr auto reduce_pyramid_validated = &simd::reduce_pyramid<T, true>;
  static constexpr auto interpolate_subpixels_validated = &simd::interpolate_subpixels<T, true>;
  static constexpr auto extract_external_subpixels = &simd::extract_external_subpixels<T>;
  static BlockError block_error(const SamplingGeometry& geometry, BlockRegion block, const SamplingFrames<T>& frames,
                                MotionVector vector, BlockMetric metric) {
    return simd::block_error(geometry, block, frames, vector, metric);
  }
  // Geometry/domain and frame storage have already been admitted by the plan.
  static BlockError block_error_validated(const SamplingGeometry& geometry, BlockRegion block,
                                          const SamplingFrames<T>& frames, MotionVector vector, BlockMetric metric) {
    return simd::block_error<T, true>(geometry, block, frames, vector, metric);
  }
};
} // namespace neo_mv
