#pragma once

#include "filters/super_payload.hpp"
#include "core/render/frame.hpp"
#include "kernels/selection.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/render_ops.hpp"
#endif
#include <dualsynth/staged_video.hpp>

namespace neo_mv::ds2 {

inline CompensateParameters compensate_parameters(const ds::ParamValues& values) {
  const Params p{values};
  return {unwrap(values.get_int64("thsad", 10000)),
          unwrap(values.get_int64("thscd1", 400)),
          unwrap(values.get_double("time", 100)),
          unwrap(values.get_double("thscd2", 51)),
          p.boolean("fields", false),
          p.tff()};
}
template <class T>
std::array<T, 2> render_pair(const std::vector<T>& values, std::array<T, 2> fallback) {
  require(values.size() <= 2, "render parameter requires at most two values");
  return values.empty() ? fallback : std::array<T, 2>{values[0], values.size() == 1 ? values[0] : values[1]};
}
inline DegrainParameters degrain_parameters(const ds::ParamValues& values, std::size_t members) {
  DegrainParameters p;
  p.near = render_pair(unwrap(values.get_int_array("thsad", {})), p.near);
  p.far = render_pair(unwrap(values.get_int_array("thsad2", {})), p.near);
  p.limit = render_pair(unwrap(values.get_double_array("limit", {})), p.limit);
  auto planes = unwrap(values.get_int_array("planes", {}));
  if (!planes.empty()) {
    p.planes = {false, false, false};
    for (auto k : planes) {
      require(k >= 0 && k <= 2, "invalid render plane index");
      require(!p.planes[static_cast<std::size_t>(k)], "duplicate render plane index");
      p.planes[static_cast<std::size_t>(k)] = true;
    }
  }
  p.weights = unwrap(values.get_int_array("weights", std::vector<std::int64_t>(members + 1, 1)));
  require(p.weights.size() == members + 1, "Degrain weights requires 2R+1 elements");
  p.thscd1 = unwrap(values.get_int64("thscd1", 400));
  p.thscd2 = unwrap(values.get_double("thscd2", 51));
  return p;
}
inline RenderVideo render_video(const ds::VideoInputInfo& in) {
  return {in.width,
          in.height,
          ds::bits_per_sample(in.format.sample_format),
          in.format.color_family == ds::ColorFamily::Yuv,
          1 << in.format.subsampling_w,
          1 << in.format.subsampling_h,
          in.num_frames};
}
template <class T>
RenderImage<T> render_image(const FrameSuper<T>& source) {
  RenderImage<T> result{&source.plan, {}};
  for (int k = 0; k < source.plan.geometry().plane_count; ++k) {
    result.planes[k].pel = source.plan.params().pel;
    for (int a = 0; a < result.planes[k].pel * result.planes[k].pel; ++a)
      result.planes[k].planes[a] = source.phase(k, 0, a);
  }
  return result;
}

struct RenderRequest {
  int stage = 0;
  std::vector<AnalysisField> fields;
  std::vector<std::optional<std::int64_t>> references;
};
struct RenderRuntime {
  virtual ~RenderRuntime() = default;
  virtual ds::VideoStageResult advance(ds::VideoStageContext&, RenderRequest&) const = 0;
  virtual void process(ds::VideoProcessContext&, const RenderRequest&) const = 0;
};

template <class T, bool Degrain, class Kernels>
class TypedRenderRuntime final : public RenderRuntime {
  using Plan = std::conditional_t<Degrain, DegrainFramePlan<T, Kernels>, CompensateFramePlan<T, Kernels>>;
  ds::VideoInputInfo clip_, super_;
  std::string prefix_;
  CompensateParameters compensation_;
  Plan plan_;

  static Plan make_plan(ds::VideoInitContext& ctx, const FrameSuper<T>& super, const std::string& prefix) {
    std::vector<AnalysisMetadata> metadata;
    std::vector<std::int64_t> lengths;
    for (std::size_t i = 2; i < ctx.inputs.size(); ++i) {
      const auto f = read_field(frame(*ctx.frames, static_cast<int>(i), 0), prefix, false);
      require(f.state != FieldState::invalid_metadata, "invalid render creation analysis metadata");
      metadata.push_back(f.metadata);
      lengths.push_back(ctx.inputs[i].num_frames);
    }
    if constexpr (Degrain)
      return Plan(render_video(ctx.inputs[0]), super.plan, ctx.inputs[1].num_frames, metadata, lengths,
                  degrain_parameters(*ctx.params, metadata.size()));
    else
      return Plan(render_video(ctx.inputs[0]), super.plan, ctx.inputs[1].num_frames, metadata.at(0), lengths.at(0),
                  compensate_parameters(*ctx.params));
  }

public:
  TypedRenderRuntime(ds::VideoInitContext& ctx, const FrameSuper<T>& super)
      : clip_(ctx.inputs[0]), super_(ctx.inputs[1]), prefix_(Params{*ctx.params}.prefix()),
        compensation_(Degrain ? CompensateParameters{} : compensate_parameters(*ctx.params)),
        plan_(make_plan(ctx, super, prefix_)) {
    plan_.validate_current(render_image(super));
  }
  ds::VideoStageResult advance(ds::VideoStageContext& ctx, RenderRequest& request) const override {
    if (request.stage++ == 0) {
      for (std::size_t i = 0; i < ctx.inputs.size(); ++i)
        ctx.request_frame(static_cast<int>(i), ctx.output_frame);
      return ds::VideoStageResult::RequestFrames;
    }
    if (request.stage == 2) {
      const auto current = frame(ctx.frames, 1, ctx.output_frame);
      FrameSuper<T> payload(current.frame, super_, prefix_);
      plan_.validate_current(render_image(payload));
      for (std::size_t i = 2; i < ctx.inputs.size(); ++i)
        request.fields.push_back(read_field(frame(ctx.frames, static_cast<int>(i), ctx.output_frame), prefix_));
      if constexpr (Degrain)
        request.references = plan_.references(request.fields, ctx.output_frame);
      else
        request.references = {plan_.reference(request.fields.at(0), ctx.output_frame)};
      for (auto n : request.references)
        if (n && !ctx.frames.get(1, static_cast<int>(*n)).has_value())
          ctx.request_frame(1, static_cast<int>(*n));
      // d=0 reuses the first-stage frame. A batch of only duplicate requests
      // must finish here rather than trip DS2's no-progress protection.
      if (!ctx.requests.empty())
        return ds::VideoStageResult::RequestFrames;
    }
    return ds::VideoStageResult::Ready;
  }
  void process(ds::VideoProcessContext& ctx, const RenderRequest& request) const override {
    const auto visible = frame(ctx.frames, 0, ctx.output_frame);
    validate_frame(visible.frame, clip_);
    RenderPixels<T> pixels{};
    for (int k = 0; k < visible.frame.plane_count; ++k)
      pixels[k] = plane<T>(visible.frame.plane(k));
    const auto current = frame(ctx.frames, 1, ctx.output_frame);
    FrameSuper<T> centre(current.frame, super_, prefix_);
    std::vector<std::unique_ptr<FrameSuper<T>>> owners;
    std::vector<RenderImage<T>> images;
    for (auto n : request.references) {
      if (!n) {
        images.emplace_back();
      } else if (*n == ctx.output_frame) {
        images.push_back(render_image(centre));
      } else {
        auto reference = frame(ctx.frames, 1, static_cast<int>(*n));
        owners.push_back(std::make_unique<FrameSuper<T>>(reference.frame, super_, prefix_));
        images.push_back(render_image(*owners.back()));
      }
    }
    RenderOutput<T> output;
    if constexpr (Degrain) {
      output = plan_.render(ctx.output_frame, request.fields, pixels, render_image(centre), images);
    } else {
      std::optional<bool> current_top, reference_top;
      if (request.references.at(0) && compensation_.fields && !compensation_.tff &&
          request.fields.at(0).metadata.delta % 2 != 0) {
        current_top = parity(current.frame, ctx.output_frame, {});
        const auto n = static_cast<int>(*request.references[0]);
        reference_top = parity(frame(ctx.frames, 1, n).frame, n, {});
      }
      output = plan_.render(ctx.output_frame, request.fields.at(0), pixels, render_image(centre),
                            request.references.at(0) ? &images.at(0) : nullptr, current_top, reference_top);
    }
    for (int k = 0; k < clip_.format.plane_count; ++k) {
      const auto src = std::as_const(output.at(k)).view();
      auto dst = plane<T>(ctx.dst.plane(k));
      require(dst.width() == src.width() && dst.height() == src.height(), "render output storage mismatch");
      for (int y = 0; y < src.height(); ++y)
        std::memcpy(dst.row(y).data(), src.row(y).data(), std::size_t(src.width()) * sizeof(T));
    }
  }
};

template <bool Degrain>
struct RenderFilter {
  static constexpr const char* name = Degrain ? "Degrain" : "Compensate";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::fresh(0);
  using RequestState = RenderRequest;
  struct State {
    std::shared_ptr<const RenderRuntime> runtime;
  };
  template <class T>
  static std::shared_ptr<const RenderRuntime> make(ds::VideoInitContext& ctx) {
    const auto first = frame(*ctx.frames, 1, 0);
    FrameSuper<T> super(first.frame, ctx.inputs[1], Params{*ctx.params}.prefix());
#if NEO_MV_ENABLE_HIGHWAY
    if (selected_backend() == KernelBackend::highway)
      return std::make_shared<TypedRenderRuntime<T, Degrain, HighwayRenderKernels<T>>>(ctx, super);
#endif
    return std::make_shared<TypedRenderRuntime<T, Degrain, ScalarRenderKernels<T>>>(ctx, super);
  }
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params && ctx.frames && ctx.frame_factory,
            "VS frame services required");
    selected_backend();
    const auto count = ctx.inputs.size();
    require(Degrain ? count >= 4 && count <= 52 && count % 2 == 0 : count == 3, "invalid render node count");
    validate_format(ctx.inputs[0]);
    validate_format(ctx.inputs[1]);
    std::shared_ptr<const RenderRuntime> runtime;
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
