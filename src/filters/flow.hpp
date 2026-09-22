#pragma once

#include "core/flow/frame.hpp"
#include "filters/phase2.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/flow.hpp"
#endif

namespace neo_mv::ds2 {
inline FlowParameters flow_parameters(const ds::ParamValues& values) {
  const Params p{values};
  return {unwrap(values.get_double("time", 100)), p.boolean("fields", false), unwrap(values.get_int64("thscd1", 400)),
          unwrap(values.get_double("thscd2", 51)), p.tff()};
}

struct FlowRequest {
  int stage = 0;
  AnalysisField field;
  std::optional<std::int64_t> reference;
};
struct FlowRuntime {
  virtual ~FlowRuntime() = default;
  virtual ds::VideoStageResult advance(ds::VideoStageContext&, FlowRequest&) const = 0;
  virtual void process(ds::VideoProcessContext&, const FlowRequest&) const = 0;
};

template <class T, class Kernels>
class TypedFlowRuntime final : public FlowRuntime {
  ds::VideoInputInfo clip_, super_;
  std::string prefix_;
  FlowParameters parameters_;
  FlowFramePlan<T, DecodedFieldKernels<Kernels>> plan_;

  static AnalysisMetadata metadata(ds::VideoInitContext& ctx, const std::string& prefix) {
    const auto field = read_field(frame(*ctx.frames, 2, 0), prefix, false);
    require(field.state != FieldState::invalid_metadata, "invalid Flow creation analysis metadata");
    return field.metadata;
  }

public:
  TypedFlowRuntime(ds::VideoInitContext& ctx, const FrameSuper<T>& super)
      : clip_(ctx.inputs[0]), super_(ctx.inputs[1]), prefix_(Params{*ctx.params}.prefix()),
        parameters_(flow_parameters(*ctx.params)),
        plan_(render_video(clip_), super.plan, super_.num_frames, metadata(ctx, prefix_), ctx.inputs[2].num_frames,
              parameters_) {
    plan_.validate_reference(render_image(super));
  }
  ds::VideoStageResult advance(ds::VideoStageContext& ctx, FlowRequest& request) const override {
    if (request.stage++ == 0) {
      ctx.request_frame(0, ctx.output_frame);
      ctx.request_frame(2, ctx.output_frame);
      return ds::VideoStageResult::RequestFrames;
    }
    if (request.stage == 2) {
      request.field = read_field(frame(ctx.frames, 2, ctx.output_frame), prefix_);
      request.reference = plan_.reference(request.field, ctx.output_frame);
      if (request.reference) {
        ctx.request_frame(1, static_cast<int>(*request.reference));
        if (plan_.field_correction_active() && !parameters_.tff && *request.reference != ctx.output_frame)
          ctx.request_frame(1, ctx.output_frame);
        return ds::VideoStageResult::RequestFrames;
      }
    }
    return ds::VideoStageResult::Ready;
  }
  void process(ds::VideoProcessContext& ctx, const FlowRequest& request) const override {
    const auto visible = frame(ctx.frames, 0, ctx.output_frame);
    validate_frame(visible.frame, clip_);
    RenderPixels<T> pixels{};
    for (int k = 0; k < visible.frame.plane_count; ++k)
      pixels[k] = plane<T>(visible.frame.plane(k));
    std::unique_ptr<FrameSuper<T>> owner;
    RenderImage<T> reference{};
    std::optional<bool> current_top, reference_top;
    if (request.reference) {
      const int n = static_cast<int>(*request.reference);
      const auto source = frame(ctx.frames, 1, n);
      owner = std::make_unique<FrameSuper<T>>(source.frame, super_, prefix_);
      reference = render_image(*owner);
      if (plan_.field_correction_active() && !parameters_.tff) {
        const auto current = frame(ctx.frames, 1, ctx.output_frame);
        const FrameSuper<T> current_payload(current.frame, super_, prefix_);
        plan_.validate_reference(render_image(current_payload));
        current_top = parity(current.frame, ctx.output_frame, {});
        reference_top = parity(source.frame, n, {});
      }
    }
    const auto destination = fresh_render_destination<T>(ctx);
    const auto output = plan_.render(pixels, request.field, ctx.output_frame, request.reference ? &reference : nullptr,
                                     current_top, reference_top, &destination);
  }
};

struct FlowFilter {
  static constexpr const char* name = "Flow";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::fresh(0);
  using RequestState = FlowRequest;
  struct State {
    std::shared_ptr<const FlowRuntime> runtime;
  };
  template <class T>
  static std::shared_ptr<const FlowRuntime> make(ds::VideoInitContext& ctx) {
    const auto first = frame(*ctx.frames, 1, 0);
    const FrameSuper<T> super(first.frame, ctx.inputs[1], Params{*ctx.params}.prefix());
#if NEO_MV_ENABLE_HIGHWAY
    if (selected_backend() == KernelBackend::highway)
      return std::make_shared<TypedFlowRuntime<T, HighwayFlowKernels<T>>>(ctx, super);
#endif
    return std::make_shared<TypedFlowRuntime<T, ScalarFlowKernels<T>>>(ctx, super);
  }
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params && ctx.frames && ctx.frame_factory,
            "VS frame services required");
    selected_backend();
    require(ctx.inputs.size() == 3, "Flow requires clip, super and vectors");
    validate_format(ctx.inputs[0]);
    validate_format(ctx.inputs[1]);
    std::shared_ptr<const FlowRuntime> runtime;
    const auto sample = ctx.inputs[0].format.sample_format;
    if (sample == ds::SampleFormat::UInt8)
      runtime = make<std::uint8_t>(ctx);
    else if (sample == ds::SampleFormat::Float32)
      runtime = make<float>(ctx);
    else
      runtime = make<std::uint16_t>(ctx);
    return ds::Result<ds::VideoInitStateResult<State>>::success({output_info(ctx.inputs[0]), {std::move(runtime)}});
  }
  static ds::VideoRequestPattern request_pattern(int input, const State&) {
    return input == 1 ? ds::VideoRequestPattern::General : ds::VideoRequestPattern::StrictSpatial;
  }
  static ds::Result<ds::VideoStageResult> advance(ds::VideoStageContext& ctx, RequestState& request) {
    return ds::Result<ds::VideoStageResult>::success(ctx.state<State>().runtime->advance(ctx, request));
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx, RequestState& request) {
    ctx.state<State>().runtime->process(ctx, request);
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
} // namespace neo_mv::ds2
