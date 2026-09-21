#pragma once
#include "filters/depan.hpp"
#include "core/depan/estimate_fft.hpp"
#include "core/depan/estimate_geometry.hpp"
#include "core/depan/estimate_image.hpp"
#include "core/depan/estimate_motion.hpp"
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/depan_estimate.hpp"
#include "highway/estimate_image.hpp"
#endif

namespace neo_mv::ds2 {
inline depan::estimate::FftProfile estimate_fft_profile() {
  return selected_backend() == KernelBackend::highway ? depan::estimate::FftProfile::native
                                                      : depan::estimate::FftProfile::scalar;
}
struct DepanEstimateFilter {
  static constexpr const char* name = "DepanEstimate";
  static constexpr int input_count = ds::dynamic_video_inputs;
  static constexpr ds::HostRequirements host_requirements{true, 0, 0};
  static constexpr ds::OutputOrigin output_origin = ds::OutputOrigin::copy_from_input(0);
  struct RequestState {
    bool requested = false;
  };
  struct State {
    ds::VideoInputInfo clip;
    depan::estimate::WindowGeometry geometry;
    std::shared_ptr<const depan::estimate::FftPlan> fft;
    float trust, zoommax, stab, aspect;
    bool info, show, fields;
    std::optional<bool> tff;
  };
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.host == ds::HostKind::VapourSynth && ctx.params, "VS services required");
    selected_backend();
    require(ctx.inputs.size() == 1, "DepanEstimate requires one clip");
    const auto clip = ctx.inputs[0];
    require(clip.width > 0 && clip.height > 0 && clip.num_frames > 0 && clip.num_frames <= INT32_MAX,
            "DepanEstimate requires constant video description");
    const auto& format = clip.format;
    require((format.color_family == ds::ColorFamily::Gray && format.plane_count == 1) ||
                (format.color_family == ds::ColorFamily::Yuv && format.plane_count == 3),
            "DepanEstimate requires GRAY or YUV");
    const int bits = ds::bits_per_sample(format.sample_format);
    require((bits >= 8 && bits <= 16) || format.sample_format == ds::SampleFormat::Float32,
            "DepanEstimate requires integer8..16 or float32");
    const Params p{*ctx.params};
    auto number = [&](const char* key, double value) {
      return depan::f32(unwrap(ctx.params->get_double(key, value)));
    };
    const float trust = number("trust", 4), zoommax = number("zoommax", 1);
    const float stab = number("stab", 1), aspect = number("pixaspect", 1);
    require(trust >= 0 && trust <= 100 && aspect > 0, "invalid DepanEstimate trust or aspect");
    const auto geometry = depan::estimate::make_geometry(clip.width, clip.height,
                                                         {p.integer("winx", 0), p.integer("winy", 0),
                                                          p.integer("wleft", -1), p.integer("wtop", -1),
                                                          p.integer("dxmax", -1), p.integer("dymax", -1), zoommax});
    State state{
        clip,
        geometry,
        std::make_shared<const depan::estimate::FftPlan>(geometry.width, geometry.height, estimate_fft_profile()),
        trust,
        zoommax,
        stab,
        aspect,
        p.boolean("info", false),
        p.boolean("show", false),
        p.boolean("fields", false),
        p.tff()};
    return ds::Result<ds::VideoInitStateResult<State>>::success({output_info(clip), std::move(state)});
  }
  static ds::VideoRequestPattern request_pattern(int, const State&) { return ds::VideoRequestPattern::General; }
  static ds::Result<ds::VideoStageResult> advance(ds::VideoStageContext& ctx, RequestState& r) {
    if (!r.requested) {
      for (int n : depan::estimate::required_source_indices(ctx.output_frame, ctx.state<State>().clip.num_frames))
        ctx.request_frame(0, n);
      r.requested = true;
      return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::RequestFrames);
    }
    return ds::Result<ds::VideoStageResult>::success(ds::VideoStageResult::Ready);
  }
  template <class T>
  static void estimate(ds::VideoProcessContext& ctx, const State& s) {
    namespace est = depan::estimate;
    const auto& g = s.geometry;
    const auto indices = est::required_source_indices(ctx.output_frame, s.clip.num_frames);
    std::vector<ds::RequestedVideoFrame> owners;
    for (int n : indices) {
      owners.push_back(frame(ctx.frames, 0, n));
      validate_frame(owners.back().frame, s.clip);
    }
    auto source = [&](int n) -> const ds::VideoFrameView& {
      require(n >= indices.front() && n <= indices.back(), "unrequested DepanEstimate source");
      return owners[std::size_t(n - indices.front())].frame;
    };
    const int bits = ds::bits_per_sample(s.clip.format.sample_format);
    std::vector<est::BasicMotion> basic;
    const auto observations = est::required_basic_indices(ctx.output_frame, s.clip.num_frames);
    std::vector<float> display[2];
    for (int n : observations) {
      const auto& current = source(n);
      const bool top = s.fields ? parity(current, n, s.tff) : false;
      const auto& previous = source((std::max)(0, n - 1));
      auto window = [&](int left, int slot) {
        auto extract = [&](const ds::VideoFrameView& frame) {
#if NEO_MV_ENABLE_HIGHWAY
          if (selected_backend() == KernelBackend::highway)
            return simd::estimate::extract_window(plane<T>(frame.plane(0)), left, g.top, g.width, g.height, bits);
#endif
          return est::extract_window(plane<T>(frame.plane(0)), left, g.top, g.width, g.height, bits);
        };
        auto a = extract(current);
        auto b = extract(previous);
        std::vector<float> correlation;
#if NEO_MV_ENABLE_HIGHWAY
        if (selected_backend() == KernelBackend::highway)
          correlation = simd::estimate::correlate(*s.fft, a, b);
        else
#endif
          correlation = s.fft->correlate(a, b);
        auto surface =
            checked_plane<const float>(correlation.data(), g.width, g.height, std::ptrdiff_t(g.width) * sizeof(float),
                                       correlation.size() * sizeof(float));
        auto compute_motion = [&] {
#if NEO_MV_ENABLE_HIGHWAY
          if (selected_backend() == KernelBackend::highway) {
            const auto peak = est::find_peak<simd::estimate::MotionScan>(surface, g.mx, g.my, s.stab, s.trust);
            return est::refine_motion<simd::estimate::MotionScan>(surface, peak, g.mx, g.my, s.aspect, s.fields, top);
          }
#endif
          const auto peak = est::find_peak(surface, g.mx, g.my, s.stab, s.trust);
          return est::refine_motion(surface, peak, g.mx, g.my, s.aspect, s.fields, top);
        };
        const auto motion = compute_motion();
        if (s.show && n == ctx.output_frame)
          display[slot] = std::move(correlation);
        return motion;
      };
      const auto first = window(g.left, 0);
      const auto second = g.two ? std::optional<est::WindowMotion>(window(g.left2, 1)) : std::nullopt;
      basic.push_back(est::combine(first, second, s.zoommax, g.left2 - g.left, n));
    }
    const auto current = std::size_t(ctx.output_frame - observations.front());
    const auto previous = current ? std::optional<float>(basic[current - 1].confidence) : std::nullopt;
    const auto next = current + 1 < basic.size() ? std::optional<float>(basic[current + 1].confidence) : std::nullopt;
    const auto motion = est::temporal_motion(basic[current], previous, next, s.trust);
    if (s.show) {
      auto output = plane<T>(ctx.dst.plane(0));
      auto show = [&](int slot, int left) {
#if NEO_MV_ENABLE_HIGHWAY
        if (selected_backend() == KernelBackend::highway) {
          simd::estimate::display_surface(output, display[slot], left, g.top, g.width, g.height, bits);
          return;
        }
#endif
        est::display_surface(output, display[slot], left, g.top, g.width, g.height, bits);
      };
      show(0, g.left);
      if (g.two)
        show(1, g.left2);
    }
    auto& props = properties(ctx.dst);
    write_motion(props, motion);
    for (const char* key : {"DepanEstimateFFT", "DepanEstimateFFT2", "DepanEstimateX", "DepanEstimateY",
                            "DepanEstimateZoom", "DepanEstimateGood", "DepanEstimateTrust"})
      props.erase(key);
    if (s.info)
      write_diagnostic(props, "DepanEstimate_info",
                       est::diagnostic(ctx.output_frame, motion, basic[current].confidence));
  }
  static ds::Result<ds::VideoProcessResult> process(ds::VideoProcessContext& ctx, RequestState&) {
    const auto& s = ctx.state<State>();
    if (s.clip.format.sample_format == ds::SampleFormat::UInt8)
      estimate<std::uint8_t>(ctx, s);
    else if (s.clip.format.sample_format == ds::SampleFormat::Float32)
      estimate<float>(ctx, s);
    else
      estimate<std::uint16_t>(ctx, s);
    return ds::Result<ds::VideoProcessResult>::success({});
  }
};
} // namespace neo_mv::ds2
