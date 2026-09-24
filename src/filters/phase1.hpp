#pragma once
#include "super_payload.hpp"
#include "kernels/selection.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/kernels.hpp"
#include "highway/scene.hpp"
#endif

namespace neo_mv::ds2 {
template <class T>
SuperPlan<T> super_plan(const ds::VideoInitContext& ctx) {
  const Params args{*ctx.params};
  const auto& info = ctx.inputs[0];
  validate_format(info);
  auto b = args.pair("blksize", {8, 8}), o = args.pair("overlap", {0, 0}), pad = args.pair("pad", {16, 16});
  SuperGeometryParams p{info.width,
                        info.height,
                        b[0],
                        b[1],
                        o[0],
                        o[1],
                        pad[0],
                        pad[1],
                        1 << info.format.subsampling_w,
                        1 << info.format.subsampling_h,
                        args.integer("pel", 2),
                        info.format.color_family == ds::ColorFamily::Yuv,
                        args.boolean("onelevel", false)};
  const bool external = ctx.inputs.size() == 2;
  if (external) {
    const auto& other = ctx.inputs[1];
    validate_format(other);
    require(other.format == info.format, "pelclip sample format mismatch");
    if (p.pel > 1)
      require(other.width == std::int64_t(p.pel) * info.width && other.height == std::int64_t(p.pel) * info.height &&
                  other.num_frames == info.num_frames,
              "pelclip dimensions or length mismatch");
  }
  return {p, ds::bits_per_sample(info.format.sample_format), args.integer("sharp", 2), args.integer("rfilter", 1),
          external};
}
template <class T, class Kernels>
struct SuperRuntime final : Runtime {
  const SuperPlan<T> plan;
  const std::string prefix;
  std::optional<ds::VideoInputInfo> external;
  explicit SuperRuntime(ds::VideoInitContext& ctx)
      : Runtime(ctx.inputs[0]), plan(super_plan<T>(ctx)), prefix(Params{*ctx.params}.prefix()) {
    if (ctx.inputs.size() == 2)
      external = ctx.inputs[1];
    for (int k = 0; k < plan.geometry().plane_count; ++k)
      for (const auto& level : plan.geometry().planes[k].levels)
        ds::validate_frame_dimensions({ds::ColorFamily::Gray, source.format.sample_format, 1, 0, 0}, level.padded_width,
                                      level.padded_height);
  }
  ds::VideoRequestPattern pattern(int) const override { return ds::VideoRequestPattern::StrictSpatial; }
  void request(ds::VideoRequestContext& ctx) const override {
    ctx.request_frame(0, ctx.output_frame);
    if (external && plan.params().pel > 1)
      ctx.request_frame(1, ctx.output_frame);
  }
  void process(ds::VideoProcessContext& ctx) const override {
    auto src = frame(ctx.frames, 0, ctx.output_frame);
    validate_frame(src.frame, source);
    std::array<span2d::Plane<const T>, 3> inputs{}, pel{};
    for (int k = 0; k < src.frame.plane_count; ++k)
      inputs.at(k) = plane<T>(src.frame.plane(k));
    std::optional<ds::RequestedVideoFrame> extra;
    if (external && plan.params().pel > 1) {
      extra = frame(ctx.frames, 1, ctx.output_frame);
      validate_frame(extra->frame, *external);
      for (int k = 0; k < extra->frame.plane_count; ++k)
        pel.at(k) = plane<T>(extra->frame.plane(k));
    }
    require(ctx.frame_factory != nullptr, "Super requires frame factory");
    build_host_super(plan, inputs, pel, Kernels{}, *ctx.frame_factory, properties(ctx.dst), prefix,
                     source.format.sample_format);
  }
};
inline AnalyseControls analyse_controls(Params p, int pel) {
  AnalyseControls c;
  c.levels = p.integer("levels", 0);
  c.search = p.integer("search", 2);
  c.searchparam = p.integer("searchparam", 2);
  c.pelsearch = p.integer("pelsearch", pel);
  c.mvlambda = p.integer("mvlambda", 1000);
  c.lsad = p.integer("lsad", 400);
  c.plevel = p.integer("plevel", 1);
  c.pnew = p.integer("pnew", 25);
  c.pzero = p.integer("pzero", c.pnew);
  c.pglobal = p.integer("pglobal", 0);
  c.badsad = p.integer("badsad", 10000);
  c.badrange = p.integer("badrange", 24);
  c.trymany = p.integer("trymany", 0);
  c.globalmv = p.boolean("globalmv", true);
  c.meander = p.boolean("meander", true);
  c.fields = p.boolean("fields", false);
  c.satd = p.boolean("satd", false);
  return c;
}
inline void target_axes(AnalysisMetadata& m, Params args) {
  const auto b = args.pair("blksize", {m.block_width, m.block_height}),
             o = args.pair("overlap", {m.overlap_x, m.overlap_y});
  m.block_width = b[0];
  m.block_height = b[1];
  m.overlap_x = o[0];
  m.overlap_y = o[1];
  require(geometry_detail::block_pair(b[0], b[1]) && o[0] >= 0 && o[1] >= 0 && o[0] <= b[0] / 2 && o[1] <= b[1] / 2,
          "invalid target block or overlap");
  const auto sx = b[0] - o[0], sy = b[1] - o[1];
  m.blocks_x = geometry_detail::dimension((std::int64_t(m.real_width) - o[0] + sx - 1) / sx);
  m.blocks_y = geometry_detail::dimension((std::int64_t(m.real_height) - o[1] + sy - 1) / sy);
}
template <class T, class Kernels>
struct AnalyseRuntime final : Runtime {
  std::string prefix;
  SuperPlan<T> plan;
  AnalyseControls controls;
  AnalysisMetadata metadata;
  std::vector<AnalysisLayer> layers;
  std::optional<bool> tff;
  AnalyseRuntime(ds::VideoInitContext& ctx, const FrameSuper<T>& first)
      : Runtime(ctx.inputs[0]), prefix(Params{*ctx.params}.prefix()), plan(first.plan) {
    const Params args{*ctx.params};
    const bool chroma = args.boolean("chroma", true) && plan.params().chroma;
    controls = analyse_controls(args, plan.params().pel);
    tff = args.tff();
    metadata = super_analysis_metadata(plan, args.integer("delta", 1), chroma);
    target_axes(metadata, args);
    layers = plan_analysis(metadata, super_sampling_geometry(plan, chroma), controls);
    metadata = layers.front().metadata;
  }
  void request(ds::VideoRequestContext& ctx) const override {
    ctx.request_frame(0, ctx.output_frame);
    const auto ref = std::int64_t(ctx.output_frame) + metadata.delta;
    if (ref >= 0 && ref < source.num_frames)
      ctx.request_frame(0, int(ref));
  }
  void process(ds::VideoProcessContext& ctx) const override {
    const int n = ctx.output_frame;
    auto src = frame(ctx.frames, 0, n);
    FrameSuper<T> current(src.frame, source, prefix);
    validate_super_pair(plan, current.plan);
    const bool top = controls.fields ? parity(src.frame, n, tff) : false;
    AnalysisField field;
    field.metadata = metadata;
    field.state = FieldState::metadata_only;
    const auto ref = std::int64_t(n) + metadata.delta;
    if (ref >= 0 && ref < source.num_frames) {
      auto other = frame(ctx.frames, 0, int(ref));
      FrameSuper<T> reference(other.frame, source, prefix);
      auto frames = sample_frames(current, reference, metadata.chroma);
      int shift = 0;
      if (controls.fields && metadata.pel > 1 && metadata.delta % 2 != 0) {
        const bool other_top = parity(other.frame, int(ref), tff);
        if (top != other_top)
          shift = top ? metadata.pel / 2 : -metadata.pel / 2;
      }
      field.grid = analyse_vectors_planned<T, Kernels>(layers, frames, controls, shift);
      field.state = FieldState::complete;
    }
    write_field(properties(ctx.dst), field, prefix);
  }
};
template <class T, class Kernels>
struct RecalculateRuntime final : Runtime {
  const std::string prefix;
  SuperPlan<T> plan;
  AnalysisMetadata old_metadata, target;
  SamplingGeometry geometry;
  RecalculateControls controls;
  bool fields;
  std::optional<bool> tff;
  RecalculateRuntime(ds::VideoInitContext& ctx, const FrameSuper<T>& first)
      : Runtime(ctx.inputs[0]), prefix(Params{*ctx.params}.prefix()), plan(first.plan) {
    const Params args{*ctx.params};
    auto old = frame(*ctx.frames, 1, 0);
    auto field = read_field(old, prefix, false);
    require(field.state != FieldState::invalid_metadata, "invalid vector frame-0 metadata");
    old_metadata = field.metadata;
    require(old_metadata.bits == plan.bits(), "old vector precision differs from Super");
    const bool chroma = args.boolean("chroma", true) && plan.params().chroma;
    target = super_analysis_metadata(plan, old_metadata.delta, chroma);
    target_axes(target, args);
    target.levels = 1;
    geometry = super_sampling_geometry(plan, chroma).front();
    validate_motion_layer(target, geometry, true);
    controls.thsad = args.integer("thsad", 200);
    controls.mvlambda = args.integer("mvlambda", 1000);
    controls.search = args.integer("search", 2);
    controls.searchparam = args.integer("searchparam", 2);
    controls.pnew = args.integer("pnew", 25);
    controls.smooth = args.boolean("smooth", true);
    controls.satd = args.boolean("satd", false);
    controls.meander = args.boolean("meander", true);
    require(controls.mvlambda >= 0 && controls.search >= 0 && controls.search <= 5 && controls.pnew >= 0 &&
                controls.pnew <= 256,
            "invalid Recalculate controls");
    require(!controls.satd || (target.block_width % 4 == 0 && target.block_height % 4 == 0),
            "SATD requires block width and height divisible by 4");
    fields = args.boolean("fields", false);
    tff = args.tff();
    require(!fields || old_metadata.pel > 1, "fields requires old pel > 1");
  }
  int reference(int n) const {
    return int(std::clamp(std::int64_t(n) + old_metadata.delta, std::int64_t(0), std::int64_t(source.num_frames - 1)));
  }
  ds::VideoRequestPattern pattern(int input) const override {
    return input == 1 ? ds::VideoRequestPattern::StrictSpatial : ds::VideoRequestPattern::General;
  }
  void request(ds::VideoRequestContext& ctx) const override {
    require(ctx.output_frame < ctx.inputs[1].num_frames, "vectors does not provide the requested frame");
    ctx.request_frame(0, ctx.output_frame);
    ctx.request_frame(0, reference(ctx.output_frame));
    ctx.request_frame(1, ctx.output_frame);
  }
  void process(ds::VideoProcessContext& ctx) const override {
    const int n = ctx.output_frame, k = reference(n);
    auto a = frame(ctx.frames, 0, n), b = frame(ctx.frames, 0, k), old = frame(ctx.frames, 1, n);
    FrameSuper<T> current(a.frame, source, prefix), ref(b.frame, source, prefix);
    validate_super_pair(plan, current.plan);
    auto input = read_field(old, prefix);
    require(input.state != FieldState::invalid_metadata && same_metadata(input.metadata, old_metadata, false),
            "old analysis geometry changed");
    if (fields) {
      parity(a.frame, n, tff);
      if (target.pel > 1 && old_metadata.delta % 2 != 0)
        parity(b.frame, k, tff);
    }
    auto borrowed = sample_frames(current, ref, target.chroma);
    AnalysisField output{
        target, FieldState::complete,
        recalculate_vectors<T, Kernels, true, true>(input, target, geometry, borrowed.front(), controls)};
    write_field(properties(ctx.dst), output, prefix);
  }
};
struct SceneRuntime final : Runtime {
  std::string prefix;
  SceneClassifier classifier;
  SceneRuntime(ds::VideoInitContext& ctx, AnalysisMetadata metadata)
      : Runtime(ctx.inputs[0]), prefix(Params{*ctx.params}.prefix()),
        classifier(scene_descriptor(metadata), unwrap(ctx.params->get_int64("thscd1", 400)),
                   unwrap(ctx.params->get_double("thscd2", 51.0))) {}
  ds::VideoRequestPattern pattern(int) const override { return ds::VideoRequestPattern::StrictSpatial; }
  void request(ds::VideoRequestContext& ctx) const override {
    require(ctx.output_frame < ctx.inputs[1].num_frames, "vectors does not provide the requested frame");
    ctx.request_frame(0, ctx.output_frame);
    ctx.request_frame(1, ctx.output_frame);
  }
  void process(ds::VideoProcessContext& ctx) const override {
    auto input = frame(ctx.frames, 1, ctx.output_frame);
    auto field = read_field(input, prefix);
    auto count = scalar_scene_count_validated;
#if NEO_MV_ENABLE_HIGHWAY
    if (selected_backend() == KernelBackend::highway)
      count = simd::scene_count_validated;
#endif
    set_scalar(properties(ctx.dst), field.metadata.delta > 0 ? "_SceneChangeNext" : "_SceneChangePrev",
               classifier(field, count));
  }
};
enum class Operation { Super, Analyse, Recalculate, SCDetection };
template <Operation Op>
struct Filter {
  static constexpr const char* name = Op == Operation::Super         ? "Super"
                                      : Op == Operation::Analyse     ? "Analyse"
                                      : Op == Operation::Recalculate ? "Recalculate"
                                                                     : "SCDetection";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::copy_from_input(0);
  struct State {
    std::shared_ptr<const Runtime> runtime;
  };
  template <class T, class Kernels>
  static std::shared_ptr<const Runtime> make_with_kernels(ds::VideoInitContext& ctx) {
    if constexpr (Op == Operation::Super)
      return std::make_shared<SuperRuntime<T, Kernels>>(ctx);
    else {
      validate_format(ctx.inputs[0]);
      auto first = frame(*ctx.frames, 0, 0);
      FrameSuper<T> payload(first.frame, ctx.inputs[0], Params{*ctx.params}.prefix());
      if constexpr (Op == Operation::Analyse)
        return std::make_shared<AnalyseRuntime<T, Kernels>>(ctx, payload);
      else
        return std::make_shared<RecalculateRuntime<T, Kernels>>(ctx, payload);
    }
  }
  template <class T>
  static std::shared_ptr<const Runtime> make(ds::VideoInitContext& ctx) {
#if NEO_MV_ENABLE_HIGHWAY
    if (selected_backend() == KernelBackend::highway)
      return make_with_kernels<T, HighwayKernels<T>>(ctx);
#endif
    return make_with_kernels<T, ScalarKernels<T>>(ctx);
  }
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.params && ctx.frames && ctx.frame_factory,
            "host frame services required");
    selected_backend(); // Freeze selection for every graph, including SCDetection.
    const auto count = ctx.inputs.size();
    require(Op == Operation::Super     ? (count == 1 || count == 2)
            : Op == Operation::Analyse ? count == 1
                                       : count == 2,
            "incorrect input node count");
    std::shared_ptr<const Runtime> runtime;
    if constexpr (Op == Operation::SCDetection) {
      auto first = frame(*ctx.frames, 1, 0);
      auto metadata = read_field(first, Params{*ctx.params}.prefix(), false);
      require(metadata.state != FieldState::invalid_metadata, "invalid scene creation metadata");
      runtime = std::make_shared<SceneRuntime>(ctx, metadata.metadata);
    } else {
      const auto sample = ctx.inputs[0].format.sample_format;
      if (sample == ds::SampleFormat::UInt8)
        runtime = make<std::uint8_t>(ctx);
      else if (sample == ds::SampleFormat::Float32)
        runtime = make<float>(ctx);
      else
        runtime = make<std::uint16_t>(ctx);
    }
    return ds::Result<ds::VideoInitStateResult<State>>::success({output_info(ctx.inputs[0]), {std::move(runtime)}});
  }
  static ds::VideoRequestPattern request_pattern(int input, const State& state) {
    return state.runtime->pattern(input);
  }
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
