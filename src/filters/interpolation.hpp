#pragma once

#include "core/interpolation/frame.hpp"
#include "core/interpolation/rate.hpp"
#include "filters/phase2.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/interpolation.hpp"
#endif

namespace neo_mv::ds2 {
enum class TemporalKind { Inter, FPS, Blur };
struct TemporalRequest {
  int stage = 0, time = 0, property_frame = 0;
  std::int64_t left = 0, right = 0;
  bool endpoint = false, motion = false, extra = false;
  AnalysisField B, F, BB, FF;
};
struct TemporalRuntime {
  virtual ~TemporalRuntime() = default;
  virtual ds::VideoOutputInfo output() const = 0;
  virtual ds::VideoStageResult advance(ds::VideoStageContext&, TemporalRequest&) const = 0;
  virtual void process(ds::VideoProcessContext&, const TemporalRequest&) const = 0;
};

template <class T, TemporalKind Kind, class Kernels = ScalarInterpolationKernels<T>>
class TypedTemporalRuntime final : public TemporalRuntime {
  using Plan =
      std::conditional_t<Kind == TemporalKind::Blur, BlurFramePlan<T, Kernels>, InterpolationFramePlan<T, Kernels>>;
  ds::VideoInputInfo clip_, super_;
  std::string prefix_;
  bool blend_, extras_;
  int time_ = 0;
  Plan plan_;
  std::optional<FrameRatePlan> rate_;

  static Plan make_plan(ds::VideoInitContext& ctx, const FrameSuper<T>& super, const std::string& prefix) {
    std::array<AnalysisMetadata, 2> descriptors;
    for (int k = 0; k < 2; ++k) {
      const auto f = read_field(frame(*ctx.frames, k + 2, 0), prefix, false);
      require(f.state != FieldState::invalid_metadata, "invalid temporal creation analysis metadata");
      descriptors[k] = f.metadata;
    }
    const auto video = render_video(ctx.inputs[0]);
    InterpolationInputPlan<T> input(
        video, super.plan, ctx.inputs[1].num_frames, descriptors, {ctx.inputs[2].num_frames, ctx.inputs[3].num_frames},
        unwrap(ctx.params->get_int64("thscd1", 400)), unwrap(ctx.params->get_double("thscd2", 51)));
    input.validate_image(render_image(super));
    if constexpr (Kind == TemporalKind::Blur)
      return Plan(std::move(input), video, unwrap(ctx.params->get_double("blur", 50)),
                  unwrap(ctx.params->get_int64("prec", 1)));
    else
      return Plan(std::move(input), video, unwrap(ctx.params->get_double("ml", 100)));
  }
  int clamp_frame(std::int64_t n) const {
    return static_cast<int>(std::clamp(n, std::int64_t{0}, std::int64_t(clip_.num_frames - 1)));
  }
  void finish_origin(ds::VideoStageContext& ctx, const TemporalRequest& r) const {
    ctx.origin = ds::OutputOrigin::fresh(0);
    ctx.origin.prop_frame = r.property_frame;
  }
  ds::VideoStageResult request_fallback(ds::VideoStageContext& ctx, TemporalRequest& r) const {
    r.stage = 4;
    if constexpr (Kind != TemporalKind::Blur) {
      if (blend_ && !r.endpoint && clamp_frame(r.right) != r.property_frame) {
        ctx.request_frame(0, clamp_frame(r.right));
        return ds::VideoStageResult::RequestFrames;
      }
    }
    finish_origin(ctx, r);
    return ds::VideoStageResult::Ready;
  }
  ds::VideoStageResult request_images(ds::VideoStageContext& ctx, TemporalRequest& r) const {
    r.stage = 4;
    if constexpr (Kind == TemporalKind::Blur) {
      ctx.request_frame(1, ctx.output_frame);
    } else {
      ctx.request_frame(1, static_cast<int>(r.left));
      ctx.request_frame(1, static_cast<int>(r.right));
    }
    return ds::VideoStageResult::RequestFrames;
  }
  RenderPixels<T> pixels(ds::VideoProcessContext& ctx, int n) const {
    const auto f = frame(ctx.frames, 0, n);
    validate_frame(f.frame, clip_);
    RenderPixels<T> result{};
    for (int k = 0; k < clip_.format.plane_count; ++k)
      result.at(static_cast<std::size_t>(k)) = plane<T>(f.frame.plane(k));
    return result;
  }

public:
  TypedTemporalRuntime(ds::VideoInitContext& ctx, const FrameSuper<T>& super)
      : clip_(ctx.inputs[0]), super_(ctx.inputs[1]), prefix_(Params{*ctx.params}.prefix()),
        blend_(Kind == TemporalKind::Blur ? false : Params{*ctx.params}.boolean("blend", true)),
        extras_(Kind == TemporalKind::Inter ||
                (Kind == TemporalKind::FPS && Params{*ctx.params}.boolean("extramask", true))),
        plan_(make_plan(ctx, super, prefix_)) {
    if constexpr (Kind == TemporalKind::Inter)
      time_ = interpolation_time_coefficient(unwrap(ctx.params->get_double("time", 50)));
    if constexpr (Kind == TemporalKind::FPS)
      rate_.emplace(clip_.num_frames, clip_.fps.numerator, clip_.fps.denominator,
                    unwrap(ctx.params->get_int64("num", 25)), unwrap(ctx.params->get_int64("den", 1)),
                    plan_.input().distance());
  }
  ds::VideoOutputInfo output() const override {
    auto result = output_info(clip_);
    if (rate_) {
      result.num_frames = static_cast<int>(rate_->output_frames());
      result.fps = {rate_->output_num(), rate_->output_den()};
    }
    return result;
  }
  ds::VideoStageResult advance(ds::VideoStageContext& ctx, TemporalRequest& r) const override {
    if (r.stage == 0) {
      r.left = ctx.output_frame;
      r.right = r.left + plan_.input().distance();
      r.time = time_;
      r.property_frame = ctx.output_frame;
      if constexpr (Kind == TemporalKind::Blur)
        r.left -= plan_.input().distance();
      if (rate_) {
        const auto position = rate_->position(ctx.output_frame);
        r.left = position.left;
        r.right = position.right;
        r.time = position.time;
        r.endpoint = position.endpoint.has_value();
        r.property_frame = clamp_frame(position.endpoint ? *position.endpoint : r.left);
      }
      ctx.request_frame(0, r.property_frame);
      r.stage = 1;
      if (!r.endpoint && plan_.input().in_range(r.left, r.right)) {
        ctx.request_frame(2, static_cast<int>(r.left));
        ctx.request_frame(3, static_cast<int>(r.right));
      }
      return ds::VideoStageResult::RequestFrames;
    }
    if (r.stage == 1) {
      if (r.endpoint || !plan_.input().in_range(r.left, r.right))
        return request_fallback(ctx, r);
      r.B = read_field(frame(ctx.frames, 2, static_cast<int>(r.left)), prefix_);
      r.F = read_field(frame(ctx.frames, 3, static_cast<int>(r.right)), prefix_);
      r.motion = plan_.main_eligible(r.B, r.F);
      if (!r.motion)
        return request_fallback(ctx, r);
      if (extras_) {
        r.stage = 2;
        ctx.request_frame(2, static_cast<int>(r.right));
        ctx.request_frame(3, static_cast<int>(r.left));
        return ds::VideoStageResult::RequestFrames;
      }
      return request_images(ctx, r);
    }
    if (r.stage == 2) {
      r.BB = read_field(frame(ctx.frames, 2, static_cast<int>(r.right)), prefix_);
      r.FF = read_field(frame(ctx.frames, 3, static_cast<int>(r.left)), prefix_);
      r.extra = plan_.extra_eligible(r.BB, r.FF);
      return request_images(ctx, r);
    }
    finish_origin(ctx, r);
    return ds::VideoStageResult::Ready;
  }
  void process(ds::VideoProcessContext& ctx, const TemporalRequest& r) const override {
    const auto visible = pixels(ctx, r.property_frame);
    RenderOutput<T> output;
    if (!r.motion) {
      if constexpr (Kind == TemporalKind::Blur) {
        output = plan_.copy(visible);
      } else {
        output = blend_ && !r.endpoint ? plan_.blend(visible, pixels(ctx, clamp_frame(r.right)), r.time)
                                       : plan_.copy(visible);
      }
    } else if constexpr (Kind == TemporalKind::Blur) {
      const auto source = frame(ctx.frames, 1, ctx.output_frame);
      const FrameSuper<T> image(source.frame, super_, prefix_);
      output = plan_.motion(r.B, r.F, render_image(image));
    } else {
      const auto l = frame(ctx.frames, 1, static_cast<int>(r.left));
      const auto rr = frame(ctx.frames, 1, static_cast<int>(r.right));
      const FrameSuper<T> left(l.frame, super_, prefix_), right(rr.frame, super_, prefix_);
      output = plan_.motion(r.B, r.F, r.extra ? &r.BB : nullptr, r.extra ? &r.FF : nullptr, render_image(left),
                            render_image(right), r.time);
    }
    for (int k = 0; k < clip_.format.plane_count; ++k) {
      const auto src = output[k].view();
      auto dst = plane<T>(ctx.dst.plane(k));
      require(dst.width() == src.width() && dst.height() == src.height(), "temporal output storage mismatch");
      for (int y = 0; y < src.height(); ++y)
        std::memcpy(dst.row(y).data(), src.row(y).data(), std::size_t(src.width()) * sizeof(T));
    }
    if (rate_) {
      set_scalar(properties(ctx.dst), "_DurationNum", rate_->output_den());
      set_scalar(properties(ctx.dst), "_DurationDen", rate_->output_num());
    }
  }
};

template <TemporalKind Kind>
struct TemporalFilter {
  static constexpr const char* name = Kind == TemporalKind::Inter ? "FlowInter"
                                      : Kind == TemporalKind::FPS ? "FlowFPS"
                                                                  : "FlowBlur";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::fresh_without_props();
  using RequestState = TemporalRequest;
  struct State {
    std::shared_ptr<const TemporalRuntime> runtime;
  };
  template <class T>
  static std::shared_ptr<const TemporalRuntime> make(ds::VideoInitContext& ctx) {
    const auto first = frame(*ctx.frames, 1, 0);
    const FrameSuper<T> super(first.frame, ctx.inputs[1], Params{*ctx.params}.prefix());
#if NEO_MV_ENABLE_HIGHWAY
    if (selected_backend() == KernelBackend::highway)
      return std::make_shared<TypedTemporalRuntime<T, Kind, HighwayInterpolationKernels<T>>>(ctx, super);
#endif
    return std::make_shared<TypedTemporalRuntime<T, Kind>>(ctx, super);
  }
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params && ctx.frames && ctx.frame_factory,
            "VS frame services required");
    selected_backend();
    require(ctx.inputs.size() == 4, "temporal filter requires clip, super and exactly [bw,fw]");
    validate_format(ctx.inputs[0]);
    validate_format(ctx.inputs[1]);
    std::shared_ptr<const TemporalRuntime> runtime;
    const auto sample = ctx.inputs[0].format.sample_format;
    if (sample == ds::SampleFormat::UInt8)
      runtime = make<std::uint8_t>(ctx);
    else if (sample == ds::SampleFormat::Float32)
      runtime = make<float>(ctx);
    else
      runtime = make<std::uint16_t>(ctx);
    const auto output = runtime->output();
    return ds::Result<ds::VideoInitStateResult<State>>::success({output, {std::move(runtime)}});
  }
  static ds::VideoRequestPattern request_pattern(int, const State&) { return ds::VideoRequestPattern::General; }
  static ds::Result<ds::VideoStageResult> advance(ds::VideoStageContext& ctx, RequestState& request) {
    return ds::Result<ds::VideoStageResult>::success(ctx.state<State>().runtime->advance(ctx, request));
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx, RequestState& request) {
    ctx.state<State>().runtime->process(ctx, request);
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
} // namespace neo_mv::ds2
