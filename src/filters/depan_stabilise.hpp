#pragma once
#include "filters/depan.hpp"
#include "core/depan/stabilise_layers.hpp"
#include <map>

namespace neo_mv::ds2 {
struct DepanStabiliseFilter {
  static constexpr const char* name = "DepanStabilise";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::fresh(0);
  struct State {
    ds::VideoInputInfo clip;
    depan::stabilise::Parameters parameters;
    depan::stabilise::Coefficients coefficients;
  };
  struct RequestState {
    int stage = 0;
    std::int64_t cursor = 0;
    depan::stabilise::Interval initial{}, interval{};
    std::map<int, depan::Motion> motions;
    depan::stabilise::Correction correction{};
    depan::stabilise::Layers layers{};
  };
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.params, "host services required");
    selected_backend();
    require(ctx.inputs.size() == 2, "DepanStabilise requires clip and data");
    const auto clip = ctx.inputs[0];
    validate_format(clip);
    require(clip.format.sample_format != ds::SampleFormat::Float32 &&
                clip.format.subsampling_h <= clip.format.subsampling_w,
            "DepanStabilise requires integer GRAY/YUV420/422/444");
    require(ctx.inputs[1].num_frames >= clip.num_frames, "Depan data is shorter than clip");
    const Params args{*ctx.params};
    depan::stabilise::Parameters p;
    auto floating = [&](const char* key, float& destination) {
      destination = depan::f32(unwrap(ctx.params->get_double(key, destination)));
    };
    floating("cutoff", p.cutoff);
    floating("damping", p.damping);
    floating("initzoom", p.initzoom);
    floating("dxmax", p.dxmax);
    floating("dymax", p.dymax);
    floating("zoommax", p.zoommax);
    floating("rotmax", p.rotmax);
    floating("pixaspect", p.pixaspect);
    floating("tzoom", p.tzoom);
    p.prev = args.integer("prev", 0);
    p.next = args.integer("next", 0);
    p.mirror = args.integer("mirror", 0);
    p.blur = args.integer("blur", 0);
    p.subpixel = args.integer("subpixel", 2);
    p.fitlast = args.integer("fitlast", 0);
    p.method = args.integer("method", 0);
    p.addzoom = args.boolean("addzoom", false);
    p.info = args.boolean("info", false);
    p.fields = args.boolean("fields", false);
    auto c = depan::stabilise::normalize(p, clip.width, clip.height, clip.fps.numerator, clip.fps.denominator);
    require(p.subpixel != 2 || (clip.height >> clip.format.subsampling_h) >= 2,
            "bicubic Depan needs at least two rows per plane");
    return ds::Result<ds::VideoInitStateResult<State>>::success({output_info(clip), {clip, p, std::move(c)}});
  }
  static ds::VideoRequestPattern request_pattern(int, const State&) { return ds::VideoRequestPattern::General; }
  static ds::Result<ds::VideoStageResult> advance(ds::VideoStageContext& ctx, RequestState& r) {
    namespace st = depan::stabilise;
    const auto& s = ctx.state<State>();
    const auto& p = s.parameters;
    const int n = ctx.output_frame, frames = s.clip.num_frames;
    auto requested = [] {
      return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::RequestFrames);
    };
    auto acquire = [&](int k) {
      if (r.motions.count(k))
        return true;
      auto available = ctx.frames.get(1, k);
      if (!available.has_value()) {
        ctx.request_frame(1, k);
        return false;
      }
      r.motions.emplace(k, read_motion(properties(available.value().frame)));
      return true;
    };
    if (r.stage == 0) {
      r.initial = st::initial_interval(n, frames, p, s.coefficients);
      r.interval = r.initial;
      r.cursor = n;
      r.stage = 1;
    }
    if (r.stage == 1) {
      while (r.cursor >= r.initial.begin) {
        const int k = static_cast<int>(r.cursor);
        if (k != 0 && !acquire(k))
          return requested();
        if (k == 0 || !r.motions.at(k).good) {
          r.interval.begin = k;
          break;
        }
        --r.cursor;
      }
      r.cursor = std::int64_t(n) + 1;
      r.stage = 2;
    }
    if (r.stage == 2) {
      if (p.method == 1) {
        while (r.cursor <= r.initial.end) {
          const int k = static_cast<int>(r.cursor);
          if (!acquire(k))
            return requested();
          if (!r.motions.at(k).good) {
            r.interval.end = k - 1;
            break;
          }
          ++r.cursor;
        }
        const int half = (std::min)(n - r.interval.begin, r.interval.end - n);
        r.interval = {n - half, n + half};
      }
      auto maps = st::cumulative(r.interval, s.coefficients, [&](int k) { return r.motions.at(k); });
      r.correction = st::correction(maps, r.interval, n, frames, s.clip.width, s.clip.height, p, s.coefficients);
      r.cursor = std::int64_t(n) + 1;
      r.stage = 3;
    }
    if (r.stage == 3) {
      const auto end = (std::min)(std::int64_t(frames - 1), std::int64_t(n) + p.next);
      while (r.cursor <= end) {
        if (!acquire(static_cast<int>(r.cursor)))
          return requested();
        ++r.cursor;
      }
      r.layers = st::select_layers(r.correction, n, frames, p, s.coefficients, [&](int k) { return r.motions.at(k); });
      ctx.request_frame(0, n);
      if (r.layers.previous)
        ctx.request_frame(0, r.layers.previous->frame);
      if (r.layers.next)
        ctx.request_frame(0, r.layers.next->frame);
      r.stage = 4;
      return requested();
    }
    return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::Ready);
  }
  template <class T>
  static void render(ds::VideoProcessContext& ctx, const State& s, const RequestState& r) {
    bool first = true;
    auto layer = [&](depan::stabilise::Layer selected, bool current) {
      const auto source = frame(ctx.frames, 0, selected.frame);
      validate_frame(source.frame, s.clip);
      const int bits = ds::bits_per_sample(s.clip.format.sample_format);
      for (int k = 0; k < s.clip.format.plane_count; ++k) {
        auto src = plane<T>(source.frame.plane(k));
        auto dst = plane<T>(ctx.dst.plane(k));
        const int rx = k ? 1 << s.clip.format.subsampling_w : 1, ry = k ? 1 << s.clip.format.subsampling_h : 1;
        const auto map = depan::plane_transform(selected.map, rx, ry);
        const int mode = current ? s.parameters.subpixel : 0, mirror = first ? s.parameters.mirror : 0;
        const int border = k ? 1 << (bits - 1) : 0, blur = s.parameters.blur / rx;
#if NEO_MV_ENABLE_HIGHWAY
        if (selected_backend() == KernelBackend::highway)
          depan::HighwaySamplingPlan(src.width(), src.height(), bits, mode, mirror, blur, border, map)
              .render(src, dst, !first);
        else
#endif
          depan::SamplingPlan(src.width(), src.height(), bits, mode, mirror, blur, border, map)
              .render(src, dst, !first);
      }
      first = false;
    };
    if (r.layers.previous)
      layer(*r.layers.previous, false);
    if (r.layers.next)
      layer(*r.layers.next, false);
    layer(r.layers.current, true);
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx, RequestState& r) {
    const auto& s = ctx.state<State>();
    if (s.clip.format.sample_format == ds::SampleFormat::UInt8)
      render<std::uint8_t>(ctx, s, r);
    else
      render<std::uint16_t>(ctx, s, r);
    if (s.parameters.info)
      write_diagnostic(properties(ctx.dst), "DepanStabilise_info",
                       depan::stabilise::diagnostic(ctx.output_frame, r.correction, s.coefficients));
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
} // namespace neo_mv::ds2
