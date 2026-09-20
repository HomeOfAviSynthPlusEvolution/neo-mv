#pragma once
#include "filters/common.hpp"
#include "core/depan/analysis.hpp"
#include "core/depan/diagnostics.hpp"
#include "core/depan/temporal.hpp"
#include "core/depan/sampling.hpp"
#include "kernels/selection.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/depan.hpp"
#endif

namespace neo_mv::ds2 {
template <class T>
T depan_property(const ds::FrameProperties& props, const char* key) {
  auto values = property_array<T>(props, key);
  require(!values.empty(), "empty Depan property");
  return values[0];
}
inline depan::Motion read_motion(const ds::FrameProperties& props) {
  return depan::decode_motion(depan_property<double>(props, "Depan_dx"), depan_property<double>(props, "Depan_dy"),
                              depan_property<double>(props, "Depan_rot"), depan_property<double>(props, "Depan_zoom"),
                              depan_property<std::int64_t>(props, "Depan_goodmotion"));
}
inline void write_motion(ds::FrameProperties& props, depan::Motion m) {
  const char* keys[] = {"Depan_dx", "Depan_dy", "Depan_rot", "Depan_zoom"};
  const float values[] = {m.dx, m.dy, m.rotation, m.zoom};
  for (int i = 0; i < 4; ++i) {
    props.erase(keys[i]);
    props.set(keys[i], std::vector<double>{double(values[i])});
  }
  props.erase("Depan_goodmotion");
  set_scalar(props, "Depan_goodmotion", m.good ? 1 : 0);
}
inline void write_diagnostic(ds::FrameProperties& props, const char* key, std::string value) {
  props.erase(key);
  props.set(key, std::vector<ds::PropertyData>{{std::move(value), ds::DataHint::Utf8}});
}
inline void depan_description(const ds::VideoInputInfo& in) {
  require(in.width > 0 && in.height > 0 && in.num_frames > 0, "Depan requires constant video description");
}
struct DepanAnalysisRequest {
  int stage = 0, k = 0;
  AnalysisField field;
  bool eligible = false;
};
struct DepanAnalysisFilter {
  static constexpr const char* name = "DepanAnalyse";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::copy_from_input(0);
  using RequestState = DepanAnalysisRequest;
  struct State {
    ds::VideoInputInfo clip;
    std::shared_ptr<const depan::AnalysisInputPlan> input;
    depan::FitParameters fit;
    float cx, cy;
    bool info, fields;
    std::optional<bool> tff;
  };
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params && ctx.frames, "VS frame services required");
    selected_backend();
    require(ctx.inputs.size() == 2 || ctx.inputs.size() == 3, "DepanAnalyse requires clip, vectors and optional mask");
    for (const auto& group : ctx.input_groups)
      if (group.name == "mask")
        require(!group.provided || group.count == 1, "supplied Depan mask must contain one node");
    for (const auto& in : ctx.inputs)
      depan_description(in);
    const auto clip = ctx.inputs[0];
    const Params p{*ctx.params};
    std::optional<depan::MaskDescription> mask;
    if (ctx.inputs.size() == 3) {
      const auto& in = ctx.inputs[2];
      require(in.format.plane_count > 0 && in.format.sample_format == ds::SampleFormat::UInt8,
              "Depan mask must be integer8");
      mask = depan::MaskDescription{in.width, in.height, in.num_frames};
    }
    const auto field = read_field(frame(*ctx.frames, 1, 0), "MVUtensils", false);
    require(field.state != FieldState::invalid_metadata, "invalid Depan creation metadata");
    depan::AnalysisInputPlan input(field.metadata, clip.width, clip.height, clip.num_frames, ctx.inputs[1].num_frames,
                                   unwrap(ctx.params->get_int64("thscd1", 400)),
                                   unwrap(ctx.params->get_double("thscd2", 51)), mask);
    const bool fields = p.boolean("fields", false);
    auto aspect = depan::f32(unwrap(ctx.params->get_double("pixaspect", 1)));
    require(aspect > 0, "Depan pixel aspect must be positive");
    aspect = depan::div(aspect, fields ? 2.0f : 1.0f);
    require(aspect > 0 && depan::mul(aspect, aspect) != 0, "Depan squared aspect must be nonzero");
    depan::FitParameters fit{aspect,
                             depan::f32(unwrap(ctx.params->get_double("error", 15))),
                             depan::f32(unwrap(ctx.params->get_double("wrong", 10))),
                             depan::f32(unwrap(ctx.params->get_double("zerow", .05))),
                             p.boolean("zoom", true),
                             p.boolean("rot", true)};
    depan::mul(2, fit.error);
    State state{clip,
                std::make_shared<const depan::AnalysisInputPlan>(std::move(input)),
                fit,
                depan::div(depan::f32(clip.width), 2),
                depan::div(depan::f32(clip.height), 2),
                p.boolean("info", false),
                fields,
                p.tff()};
    return ds::Result<ds::VideoInitStateResult<State>>::success({output_info(clip), std::move(state)});
  }
  static ds::VideoRequestPattern request_pattern(int, const State&) { return ds::VideoRequestPattern::General; }
  static ds::Result<ds::VideoStageResult> advance(ds::VideoStageContext& ctx, RequestState& r) {
    const auto& s = ctx.state<State>();
    if (r.stage == 0) {
      r.k = static_cast<int>(s.input->vector_index(ctx.output_frame));
      ctx.request_frame(1, r.k);
      ctx.request_frame(0, ctx.output_frame);
      if (s.input->masked())
        ctx.request_frame(2, ctx.output_frame);
      r.stage = 1;
      return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::RequestFrames);
    }
    if (r.stage == 1) {
      r.field = read_field(frame(ctx.frames, 1, r.k), "MVUtensils");
      r.eligible = s.input->eligible(r.field);
      r.stage = 2;
    }
    return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::Ready);
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx, RequestState& r) {
    const auto& s = ctx.state<State>();
    std::optional<span2d::Plane<const std::uint8_t>> mask;
    std::optional<ds::RequestedVideoFrame> owner;
    if (s.input->masked()) {
      owner = frame(ctx.frames, 2, ctx.output_frame);
      if (r.eligible)
        mask = plane<std::uint8_t>(owner->frame.plane(0));
    }
    const auto observations = s.input->observe(r.field, mask);
#if NEO_MV_ENABLE_HIGHWAY
    const auto result = selected_backend() == KernelBackend::highway
                            ? depan::fit<depan::HighwayResiduals>(observations, s.fit)
                            : depan::fit(observations, s.fit);
#else
    const auto result = depan::fit(observations, s.fit);
#endif
    depan::Motion m;
    if (result.good) {
      const bool forward = s.input->metadata().delta < 0;
      m = depan::motion(forward ? result.map : depan::analysis_inverse(result.map), s.fit.aspect, s.cx, s.cy, forward);
      if (s.fields)
        m.dy = depan::add(m.dy,
                          parity(frame(ctx.frames, 0, ctx.output_frame).frame, ctx.output_frame, s.tff) ? 1.0f : -1.0f);
    }
    auto& props = properties(ctx.dst);
    write_motion(props, m);
    if (s.info)
      write_diagnostic(props, "DepanAnalyse_info",
                       depan::analysis_info(ctx.output_frame, result.iteration, result.error, m));
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
struct DepanCompensationRequest {
  int stage = 0, next = 0;
  depan::TemporalPosition position;
  depan::Transform map;
};
struct DepanCompensationFilter {
  static constexpr const char* name = "DepanCompensate";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::fresh_without_props();
  using RequestState = DepanCompensationRequest;
  struct State {
    ds::VideoInputInfo clip;
    std::shared_ptr<const depan::TemporalPlan> temporal;
    int mode, mirror, blur;
    bool info;
    std::optional<bool> tff;
  };
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params, "VS services required");
    selected_backend();
    require(ctx.inputs.size() == 2, "DepanCompensate requires clip and data");
    const auto clip = ctx.inputs[0];
    validate_format(clip);
    require(clip.format.sample_format != ds::SampleFormat::Float32 &&
                clip.format.subsampling_h <= clip.format.subsampling_w,
            "DepanCompensate requires integer GRAY/YUV420/422/444");
    require(ctx.inputs[1].num_frames >= clip.num_frames, "Depan data is shorter than clip");
    const Params p{*ctx.params};
    State state{clip,
                std::make_shared<const depan::TemporalPlan>(clip.num_frames, clip.width, clip.height,
                                                            unwrap(ctx.params->get_double("offset", 0)),
                                                            unwrap(ctx.params->get_double("pixaspect", 1)),
                                                            p.boolean("fields", false), p.boolean("matchfields", true)),
                p.integer("subpixel", 2),
                p.integer("mirror", 0),
                p.integer("blur", 0),
                p.boolean("info", false),
                p.tff()};
    require(state.mode >= 0 && state.mode <= 2 && state.mirror >= 0 && state.mirror <= 15 && state.blur >= 0,
            "invalid Depan sampling parameters");
    require(state.mode != 2 || (clip.height >> clip.format.subsampling_h) >= 2,
            "bicubic Depan needs at least two rows per plane");
    return ds::Result<ds::VideoInitStateResult<State>>::success({output_info(clip), std::move(state)});
  }
  static ds::VideoRequestPattern request_pattern(int, const State&) { return ds::VideoRequestPattern::General; }
  static ds::Result<ds::VideoStageResult> advance(ds::VideoStageContext& ctx, RequestState& r) {
    const auto& s = ctx.state<State>();
    auto requested = [] {
      return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::RequestFrames);
    };
    if (r.stage == 0) {
      r.position = s.temporal->position(ctx.output_frame);
      r.next = r.position.next;
      if (r.position.bypass) {
        ctx.request_frame(0, ctx.output_frame);
        r.stage = 3;
        return requested();
      }
      ctx.request_frame(1, r.next);
      r.stage = 1;
      return requested();
    }
    if (r.stage == 1) {
      const auto m = read_motion(properties(frame(ctx.frames, 1, r.next).frame));
      if (!m.good)
        r.map = {};
      else {
        r.map = s.temporal->append(r.map, m, r.position);
        if (r.next < r.position.last) {
          ctx.request_frame(1, ++r.next);
          return requested();
        }
      }
      ctx.request_frame(0, r.position.source);
      if (s.temporal->needs_parity() && !s.tff && r.position.source != ctx.output_frame)
        ctx.request_frame(0, ctx.output_frame);
      r.stage = 2;
      return requested();
    }
    if (r.stage == 2) {
      bool top = false;
      if (s.temporal->needs_parity())
        top = s.tff ? (*s.tff ^ ((ctx.output_frame & 1) != 0))
                    : parity(frame(ctx.frames, 0, ctx.output_frame).frame, ctx.output_frame, {});
      r.map = s.temporal->align(r.map, top);
      r.stage = 3;
    }
    ctx.origin = r.position.bypass ? ds::OutputOrigin::copy_from_input(0) : ds::OutputOrigin::fresh(0);
    ctx.origin.pixel_frame = r.position.source;
    ctx.origin.prop_frame = r.position.source;
    return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::Ready);
  }
  template <class T>
  static void render(ds::VideoProcessContext& ctx, const State& s, const RequestState& r) {
    const auto source = frame(ctx.frames, 0, r.position.source);
    validate_frame(source.frame, s.clip);
    const int bits = ds::bits_per_sample(s.clip.format.sample_format);
    for (int k = 0; k < s.clip.format.plane_count; ++k) {
      const auto src = plane<T>(source.frame.plane(k));
      auto dst = plane<T>(ctx.dst.plane(k));
      const int rx = k ? (1 << s.clip.format.subsampling_w) : 1, ry = k ? (1 << s.clip.format.subsampling_h) : 1;
      depan::SamplingPlan plan(src.width(), src.height(), bits, s.mode, s.mirror, s.blur / rx,
                               k ? (1 << (bits - 1)) : 0, depan::plane_transform(r.map, rx, ry));
#if NEO_MV_ENABLE_HIGHWAY
      if (selected_backend() == KernelBackend::highway) {
        depan::HighwaySamplingPlan optimized(src.width(), src.height(), bits, s.mode, s.mirror, s.blur / rx,
                                             k ? (1 << (bits - 1)) : 0, depan::plane_transform(r.map, rx, ry));
        optimized.render(src, dst);
      } else
#endif
        plan.render(src, dst);
    }
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx, RequestState& r) {
    const auto& s = ctx.state<State>();
    if (!r.position.bypass) {
      if (s.clip.format.sample_format == ds::SampleFormat::UInt8)
        render<std::uint8_t>(ctx, s, r);
      else
        render<std::uint16_t>(ctx, s, r);
      if (s.info)
        write_diagnostic(properties(ctx.dst), "DepanCompensate_info",
                         depan::compensation_info(s.temporal->offset(), r.position.source, ctx.output_frame,
                                                  depan::motion(r.map, s.temporal->aspect(), s.temporal->cx(),
                                                                s.temporal->cy(), r.position.forward)));
    }
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
} // namespace neo_mv::ds2
