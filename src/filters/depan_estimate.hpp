#pragma once
#include "filters/depan.hpp"
#include "core/depan/estimate_fft.hpp"
#include "core/depan/estimate_geometry.hpp"
#include "core/depan/estimate_image.hpp"
#include "core/depan/estimate_motion.hpp"
#include <array>
#include <complex>
#include <cstring>
#include <mutex>
#include <new>
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
  struct CachedWindow {
    int frame, left;
    bool top;
    std::shared_ptr<const std::vector<float>> current, previous;
    depan::estimate::WindowMotion motion;
  };
  struct CachedSpectrum {
    int frame, left;
    std::shared_ptr<const std::vector<float>> input;
    std::shared_ptr<const std::vector<std::complex<float>>> spectrum;
  };
  struct WindowCache {
    std::mutex mutex;
    std::array<std::optional<CachedWindow>, 4> entries;
    std::array<std::optional<CachedSpectrum>, 4> spectra;
    std::size_t next = 0;
    std::size_t next_spectrum = 0;
  };
  struct State {
    ds::VideoInputInfo clip;
    depan::estimate::WindowGeometry geometry;
    std::shared_ptr<const depan::estimate::FftPlan> fft;
    std::shared_ptr<WindowCache> cache;
    float trust, zoommax, stab, aspect;
    bool info, show, fields;
    std::optional<bool> tff;
  };
  static ds::Result<ds::VideoInitStateResult<State>> init(ds::VideoInitContext& ctx) {
    require(ctx.params, "host services required");
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
        std::make_shared<WindowCache>(),
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
    // Each source owner is stable for this request. Adjacent motion windows
    // reuse its admitted pixels; cross-request reuse still compares contents.
    std::array<std::vector<std::shared_ptr<const std::vector<float>>>, 2> extracted;
    for (auto& slot : extracted)
      slot.resize(owners.size());
    for (int n : observations) {
      const auto& current = source(n);
      const bool top = s.fields ? parity(current, n, s.tff) : false;
      auto window = [&](int left, int slot) {
        auto extract = [&](int frame_index) {
          auto& cached = extracted[slot][std::size_t(frame_index - indices.front())];
          if (cached)
            return cached;
          const auto& view = source(frame_index);
          std::vector<float> values;
#if NEO_MV_ENABLE_HIGHWAY
          if (selected_backend() == KernelBackend::highway) {
            std::shared_ptr<const std::vector<float>> candidate;
            {
              std::lock_guard<std::mutex> lock(s.cache->mutex);
              for (const auto& entry : s.cache->spectra)
                if (entry && entry->frame == frame_index && entry->left == left) {
                  candidate = entry->input;
                  break;
                }
            }
            // A content match reuses admission without assuming that equal
            // frame numbers identify equal pixels across separate requests.
            if (candidate && simd::estimate::matches_window(plane<T>(view.plane(0)), left, g.top,
                                                            g.width, g.height, *candidate)) {
              cached = std::move(candidate);
              return cached;
            }
            values = simd::estimate::extract_window(plane<T>(view.plane(0)), left, g.top, g.width, g.height, bits);
          } else
#endif
            values = est::extract_window(plane<T>(view.plane(0)), left, g.top, g.width, g.height, bits);
          cached = std::make_shared<const std::vector<float>>(std::move(values));
          return cached;
        };
        auto a = extract(n);
        auto b = extract((std::max)(0, n - 1));
        const bool needs_display = s.show && n == ctx.output_frame;
        // A frame index alone is insufficient: an upstream clip may return
        // different samples for the same request. Compare the admitted inputs.
        // Four result entries plus four half-spectrum entries share a 64 MiB
        // sample-payload budget. Conservatively count shared inputs separately.
        const std::size_t slot_budget = (64u * 1024u * 1024u) / s.cache->entries.size();
        const std::size_t complex_count = std::size_t(g.height) * (std::size_t(g.width) / 2 + 1);
        const bool cacheable = a->size() <= slot_budget / (3 * sizeof(float)) &&
            complex_count <= (slot_budget - 3 * a->size() * sizeof(float)) / sizeof(std::complex<float>);
        if (cacheable && !needs_display) {
          std::lock_guard<std::mutex> lock(s.cache->mutex);
          for (const auto& entry : s.cache->entries) {
            if (entry && entry->frame == n && entry->left == left && entry->top == top &&
                entry->current->size() == a->size() && entry->previous->size() == b->size() &&
                (entry->current == a || std::memcmp(entry->current->data(), a->data(), a->size() * sizeof(float)) == 0) &&
                (entry->previous == b || std::memcmp(entry->previous->data(), b->data(), b->size() * sizeof(float)) == 0))
              return entry->motion;
          }
        }
        std::vector<float> correlation;
#if NEO_MV_ENABLE_HIGHWAY
        if (selected_backend() == KernelBackend::highway) {
          const auto forward = [&](int frame_index, const std::shared_ptr<const std::vector<float>>& input) {
            if (cacheable) {
              std::lock_guard<std::mutex> lock(s.cache->mutex);
              for (const auto& entry : s.cache->spectra)
                if (entry && entry->frame == frame_index && entry->left == left &&
                    entry->input->size() == input->size() &&
                    (entry->input == input ||
                     std::memcmp(entry->input->data(), input->data(), input->size() * sizeof(float)) == 0))
                  return entry->spectrum;
            }
            auto result = std::make_shared<const std::vector<std::complex<float>>>(
                s.fft->forward_admitted(*input, simd::estimate::samples_finite));
            if (cacheable) {
              std::lock_guard<std::mutex> lock(s.cache->mutex);
              s.cache->spectra[s.cache->next_spectrum].emplace(CachedSpectrum{frame_index, left, input, result});
              s.cache->next_spectrum = (s.cache->next_spectrum + 1) % s.cache->spectra.size();
            }
            return result;
          };
          auto current_spectrum = cacheable ? *forward(n, a) : s.fft->forward_admitted(*a, simd::estimate::samples_finite);
          const auto previous_spectrum = forward((std::max)(0, n - 1), b);
          simd::estimate::product(current_spectrum.data(), previous_spectrum->data(), current_spectrum.size());
          correlation = s.fft->inverse_admitted(current_spectrum, simd::estimate::samples_finite);
        } else
#endif
          correlation = s.fft->correlate(*a, *b);
        auto surface =
            checked_plane<const float>(correlation.data(), g.width, g.height, std::ptrdiff_t(g.width) * sizeof(float),
                                       correlation.size() * sizeof(float));
        auto compute_motion = [&] {
#if NEO_MV_ENABLE_HIGHWAY
          if (selected_backend() == KernelBackend::highway) {
            const auto peak = est::find_peak<simd::estimate::MotionScan, true>(surface, g.mx, g.my, s.stab, s.trust);
            return est::refine_motion<simd::estimate::MotionScan, true>(surface, peak, g.mx, g.my, s.aspect, s.fields, top);
          }
#endif
          const auto peak = est::find_peak<est::motion_detail::ScalarScan, true>(surface, g.mx, g.my, s.stab, s.trust);
          return est::refine_motion<est::motion_detail::ScalarScan, true>(surface, peak, g.mx, g.my, s.aspect, s.fields, top);
        };
        const auto motion = compute_motion();
        if (needs_display)
          display[slot] = std::move(correlation);
        if (cacheable) {
          std::lock_guard<std::mutex> lock(s.cache->mutex);
          s.cache->entries[s.cache->next].emplace(CachedWindow{n, left, top, std::move(a), std::move(b), motion});
          s.cache->next = (s.cache->next + 1) % s.cache->entries.size();
        }
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
