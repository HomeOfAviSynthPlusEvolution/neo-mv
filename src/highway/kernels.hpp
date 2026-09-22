#pragma once
#include "core/motion/block_sampling.hpp"
#include "core/super/border_extension.hpp"
#include "core/super/pyramid_reduction.hpp"
#include "core/super/subpixel.hpp"
#include "highway/rows.hpp"

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

// One search block owns this evaluator; its caller has already admitted the
// complete candidate domain and all frame views. Search invokes it serially.
template <class T>
class PreparedBlockError {
  int pel_, ratio_x_, ratio_y_;
  bool chroma_;
  std::array<int, 3> x_{}, y_{};
  std::array<std::array<const T*, 16>, 3> references_{};
  std::array<std::array<std::ptrdiff_t, 16>, 3> strides_{};
  std::array<detail::MetricRequest<T>, 3> requests_{};

  std::int64_t quotient(std::int64_t value) const {
    switch (pel_) {
    case 1: return value;
    case 2: return value >= 0 ? value / 2 : -((-value + 1) / 2);
    case 4: return value >= 0 ? value / 4 : -((-value + 3) / 4);
    default: return sampling_detail::floor_div(value, pel_);
    }
  }
  void reference(int k, std::int64_t qx, std::int64_t qy, std::size_t phase) {
    requests_[k].reference = references_[k][phase] +
        (std::ptrdiff_t(y_[k]) + qy) * strides_[k][phase] + (std::ptrdiff_t(x_[k]) + qx);
    requests_[k].reference_stride = strides_[k][phase];
  }
  void reference_pair(int first, int last, std::int64_t vx, std::int64_t vy) {
    const auto qx = quotient(vx), qy = quotient(vy);
    const auto phase = std::size_t((vy - pel_ * qy) * pel_ + vx - pel_ * qx);
    for (int k = first; k < last; ++k)
      reference(k, qx, qy, phase);
  }

public:
  PreparedBlockError(const SamplingGeometry& g, BlockRegion block, const SamplingFrames<T>& frames, BlockMetric metric)
      : pel_(g.pel), ratio_x_(g.ratio_x), ratio_y_(g.ratio_y), chroma_(g.chroma) {
    for (int k = 0; k < (chroma_ ? 3 : 1); ++k) {
      const int rx = k == 0 ? 1 : ratio_x_, ry = k == 0 ? 1 : ratio_y_;
      x_[k] = g.planes[k].pad_x + block.x / rx;
      y_[k] = g.planes[k].pad_y + block.y / ry;
      auto& request = requests_[k];
      request.source = frames.current[k].row(y_[k]).data() + x_[k];
      request.source_stride = frames.current[k].stride();
      request.width = block.width / rx;
      request.height = block.height / ry;
      request.satd = k == 0 && metric == BlockMetric::satd;
      for (int a = 0; a < pel_ * pel_; ++a) {
        const auto plane = frames.reference[k][a];
        references_[k][a] = plane.data();
        strides_[k][a] = plane.stride();
      }
    }
  }
  BlockError operator()(MotionVector vector) {
    reference_pair(0, 1, vector.x, vector.y);
    if (chroma_) {
      const auto tx = ratio_x_ == 2 ? std::int64_t(vector.x) / 2 : vector.x;
      const auto ty = ratio_y_ == 2 ? std::int64_t(vector.y) / 2 : vector.y;
      reference_pair(1, 3, tx, ty);
    }
    std::array<std::int64_t, 3> errors{};
    detail::metric_batch(requests_.data(), chroma_ ? 3 : 1, errors.data());
    const auto chroma = metric_detail::accumulate(errors[1], errors[2]);
    return {errors[0], chroma, metric_detail::accumulate(errors[0], chroma)};
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
  static simd::PreparedBlockError<T> prepare_block_error(const SamplingGeometry& geometry, BlockRegion block,
                                                         const SamplingFrames<T>& frames, BlockMetric metric) {
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
