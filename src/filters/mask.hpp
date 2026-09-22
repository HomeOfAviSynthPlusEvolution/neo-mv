#pragma once

#include "core/mask/frame.hpp"
#include "filters/common.hpp"
#include "kernels/selection.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/mask.hpp"
#endif

namespace neo_mv::ds2 {

inline MaskParameters mask_parameters(const ds::ParamValues& values) {
  return {unwrap(values.get_double("ml", 100)),    unwrap(values.get_double("gamma", 1)),
          unwrap(values.get_double("time", 100)),  unwrap(values.get_double("scval", 0)),
          unwrap(values.get_int64("thscd1", 400)), unwrap(values.get_double("thscd2", 51))};
}

template <class T, class Kernels>
class MaskRuntime final : public Runtime {
  std::string prefix_;
  MaskFramePlan<T, DecodedFieldKernels<Kernels>> plan_;

public:
  MaskRuntime(ds::VideoInitContext& ctx, MaskKind kind, AnalysisMetadata metadata)
      : Runtime(ctx.inputs[0]), prefix_(Params{*ctx.params}.prefix()),
        plan_(kind, metadata, source.num_frames, mask_parameters(*ctx.params)) {}
  ds::VideoRequestPattern pattern(int) const override { return ds::VideoRequestPattern::StrictSpatial; }
  void request(ds::VideoRequestContext& ctx) const override { ctx.request_frame(0, ctx.output_frame); }
  void process(ds::VideoProcessContext& ctx) const override {
    // Only the public properties of vectors[n] participate; carrier pixels and
    // temporal references are not mask dependencies.
    const auto input = frame(ctx.frames, 0, ctx.output_frame);
    const auto field = read_field(input, prefix_);
    plan_.render(field, plane<T>(ctx.dst.plane(0)));
    set_scalar(properties(ctx.dst), "_Range", 1);
  }
};

template <MaskKind Kind>
struct MaskFilter {
  static constexpr const char* name = Kind == MaskKind::VectorLength ? "VectorLengthMask"
                                      : Kind == MaskKind::SAD        ? "SADMask"
                                                                     : "OcclusionMask";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::fresh_without_props();
  struct State {
    std::shared_ptr<const Runtime> runtime;
  };
  template <class T>
  static std::shared_ptr<const Runtime> make(ds::VideoInitContext& ctx, const AnalysisMetadata& metadata) {
#if NEO_MV_ENABLE_HIGHWAY
    if (selected_backend() == KernelBackend::highway)
      return std::make_shared<MaskRuntime<T, HighwayMaskKernels<T>>>(ctx, Kind, metadata);
#endif
    return std::make_shared<MaskRuntime<T, ScalarMaskKernels<T>>>(ctx, Kind, metadata);
  }
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params && ctx.frames && ctx.frame_factory,
            "VS frame services required");
    require(ctx.inputs.size() == 1, "mask requires one vectors node");
    selected_backend();
    const auto first = read_field(frame(*ctx.frames, 0, 0), Params{*ctx.params}.prefix(), false);
    require(first.state != FieldState::invalid_metadata, "invalid mask creation analysis metadata");
    const auto& m = first.metadata;
    std::shared_ptr<const Runtime> runtime;
    if (m.bits == 8)
      runtime = make<std::uint8_t>(ctx, m);
    else if (m.bits == 32)
      runtime = make<float>(ctx, m);
    else
      runtime = make<std::uint16_t>(ctx, m);
    auto output = output_info(ctx.inputs[0]);
    output.width = m.real_width;
    output.height = m.real_height;
    output.format = unwrap(ds::make_video_format(ds::ColorFamily::Gray, m.bits == 32, m.bits, 1, 0, 0));
    return ds::Result<ds::VideoInitStateResult<State>>::success({output, {std::move(runtime)}});
  }
  static ds::VideoRequestPattern request_pattern(int, const State&) { return ds::VideoRequestPattern::StrictSpatial; }
  static ds::Result<ds::VideoRequestResult> request(ds::VideoRequestContext& ctx) {
    ctx.state<State>().runtime->request(ctx);
    return ds::Result<ds::VideoRequestResult>::success({});
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx) {
    ctx.state<State>().runtime->process(ctx);
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
} // namespace neo_mv::ds2
