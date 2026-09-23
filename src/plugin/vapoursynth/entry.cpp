#include "plugin/bridges.hpp"
#include "neo_mv_version.hpp"
#include <dualsynth/vapoursynth/video_bridge.hpp>

namespace neo_mv::ds2 {
namespace {
using MapOwner = std::unique_ptr<VSMap, decltype(VSAPI::freeMap)>;
void VS_CC kernel_info(const VSMap*, VSMap* out, void*, VSCore*, const VSAPI* api) {
  try {
    const auto fft = estimate_fft_profile();
    require(api->mapSetData(out, "backend", selected_backend_name(), -1, dtUtf8, maReplace) == 0 &&
                api->mapSetData(out, "target", selected_target_name(), -1, dtUtf8, maReplace) == 0 &&
                api->mapSetData(out, "fft", depan::estimate::fft_profile_name(fft), -1, dtUtf8, maReplace) == 0 &&
                api->mapSetInt(out, "fft_lanes", depan::estimate::fft_lanes(fft), maReplace) == 0,
            "cannot report kernel selection");
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: kernel selection failed");
  }
}
// Validate the length-delimited prefix before DS2's current string reader.
void check_prefix(const VSMap* in, const VSAPI* api) {
  if (api->mapNumElements(in, "prefix") < 0)
    return;
  int error = 0;
  const char* data = api->mapGetData(in, "prefix", 0, &error);
  require(!error && data, "prefix must be data");
  const int size = api->mapGetDataSize(in, "prefix", 0, &error);
  require(!error && size >= 0 && std::memchr(data, 0, std::size_t(size)) == nullptr,
          "prefix cannot contain embedded NUL");
}
MapOwner copy_call(const VSMap* in, const VSAPI* api) {
  MapOwner result(api->createMap(), api->freeMap);
  if (!result)
    throw std::bad_alloc();
  api->copyMap(in, result.get());
  return result;
}
int integer(const VSMap* in, const char* key, int fallback, const VSAPI* api) {
  if (api->mapNumElements(in, key) < 0)
    return fallback;
  int error = 0;
  auto value = api->mapGetInt(in, key, 0, &error);
  require(!error, "invalid integer parameter");
  return saturate(value);
}
template <Operation Op>
void VS_CC create(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    ds::vapoursynth::create_video_filter_bridge<Bridge<Op>>(in, out, core, api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: filter creation failed");
  }
}
void VS_CC many(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    const int radius = integer(in, "radius", 1, api), step = integer(in, "delta", 1, api);
    require(radius > 0 && step > 0 && radius <= INT32_MAX / 2 && std::int64_t(radius) * step <= INT32_MAX,
            "invalid AnalyseMany radius or delta product");
    std::vector<MapOwner> owners;
    std::vector<const VSMap*> calls;
    const auto count = std::size_t(radius) * 2;
    require(count <= owners.max_size() && count <= calls.max_size(), "AnalyseMany output count unrepresentable");
    owners.reserve(count);
    calls.reserve(count);
    for (int r = 1; r <= radius; ++r)
      for (int sign : {1, -1}) {
        const auto index = owners.size();
        try {
          auto call = copy_call(in, api);
          api->mapDeleteKey(call.get(), "radius");
          require(api->mapSetInt(call.get(), "delta", std::int64_t(r) * step * sign, maReplace) == 0,
                  "cannot write member delta");
          calls.push_back(call.get());
          owners.push_back(std::move(call));
        } catch (const std::exception& e) {
          throw std::runtime_error("AnalyseMany member[" + std::to_string(index) + "]: " + e.what());
        }
      }
    ds::vapoursynth::create_video_filter_bundle<Bridge<Operation::Analyse>>({calls.data(), calls.size()}, out, core,
                                                                            api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: AnalyseMany creation failed");
  }
}
void VS_CC recalculate(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    const int count = api->mapNumElements(in, "vectors");
    require(count > 0, "Recalculate requires nonempty vectors");
    std::vector<MapOwner> owners;
    std::vector<const VSMap*> calls;
    owners.reserve(count);
    calls.reserve(count);
    for (int i = 0; i < count; ++i) {
      try {
        auto call = copy_call(in, api);
        int error = 0;
        std::unique_ptr<VSNode, decltype(api->freeNode)> node(api->mapGetNode(in, "vectors", i, &error), api->freeNode);
        require(!error && bool(node), "invalid vectors node");
        api->mapDeleteKey(call.get(), "vectors");
        require(api->mapSetNode(call.get(), "vectors", node.get(), maReplace) == 0, "cannot set vectors member");
        calls.push_back(call.get());
        owners.push_back(std::move(call));
      } catch (const std::exception& e) {
        throw std::runtime_error("Recalculate member[" + std::to_string(i) + "]: " + e.what());
      }
    }
    ds::vapoursynth::create_video_filter_bundle<Bridge<Operation::Recalculate>>({calls.data(), calls.size()}, out, core,
                                                                                api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: Recalculate creation failed");
  }
}
template <bool Degrain>
void VS_CC create_render(const VSMap* in, VSMap* out, void* user_data, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    if constexpr (Degrain) {
      const auto radius = reinterpret_cast<std::intptr_t>(user_data);
      require(radius == 0 || api->mapNumElements(in, "vectors") == 2 * radius,
              "named Degrain requires exactly 2R vector members");
    }
    ds::vapoursynth::create_video_filter_bridge<RenderBridge<Degrain>>(in, out, core, api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: render creation failed");
  }
}
void VS_CC create_flow(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    ds::vapoursynth::create_video_filter_bridge<FlowBridge>(in, out, core, api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: Flow creation failed");
  }
}
template <TemporalKind Kind>
void VS_CC create_temporal(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    ds::vapoursynth::create_video_filter_bridge<TemporalBridge<Kind>>(in, out, core, api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: temporal creation failed");
  }
}
template <class Adapter>
void VS_CC create_depan(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    // Use the native wrapper only for the host text-rendering dependency.
    const bool info = integer(in, "info", 0, api) != 0;
    MapOwner base(api->createMap(), api->freeMap);
    if (!base)
      throw std::bad_alloc();
    ds::vapoursynth::create_video_filter_bridge<Adapter>(in, base.get(), core, api);
    if (const char* error = api->mapGetError(base.get()))
      throw std::runtime_error(error);
    if (!info) {
      api->copyMap(base.get(), out);
      return;
    }
    auto* text = api->getPluginByNamespace("text", core);
    require(text != nullptr, "Depan info requires text.FrameProps");
    int error = 0;
    std::unique_ptr<VSNode, decltype(api->freeNode)> node(api->mapGetNode(base.get(), "clip", 0, &error),
                                                          api->freeNode);
    require(!error && node, "Depan base node unavailable");
    MapOwner args(api->createMap(), api->freeMap);
    if (!args)
      throw std::bad_alloc();
    require(api->mapSetNode(args.get(), "clip", node.get(), maReplace) == 0 &&
                api->mapSetData(args.get(), "props", Adapter::diagnostic_property, -1, dtUtf8, maReplace) == 0,
            "cannot create Depan text arguments");
    MapOwner rendered(api->invoke(text, "FrameProps", args.get()), api->freeMap);
    require(bool(rendered), "Depan text invocation failed");
    if (const char* message = api->mapGetError(rendered.get()))
      throw std::runtime_error(message);
    api->copyMap(rendered.get(), out);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: Depan creation failed");
  }
}
template <MaskKind Kind>
void VS_CC create_mask(const VSMap* in, VSMap* out, void*, VSCore* core, const VSAPI* api) {
  try {
    check_prefix(in, api);
    ds::vapoursynth::create_video_filter_bridge<MaskBridge<Kind>>(in, out, core, api);
  } catch (const std::exception& e) {
    api->mapSetError(out, e.what());
  } catch (...) {
    api->mapSetError(out, "neo-mv: mask creation failed");
  }
}
} // namespace
} // namespace neo_mv::ds2

VS_EXTERNAL_API(void) VapourSynthPluginInit2(VSPlugin* plugin, const VSPLUGINAPI* api) {
  using namespace neo_mv::ds2;
  using neo_mv::MaskKind;
  api->configPlugin("org.neofilters.neo_mv", "neomv", "neo-mv",
                    VS_MAKE_VERSION(NEO_MV_VERSION_MAJOR, NEO_MV_VERSION_MINOR), VAPOURSYNTH_API_VERSION, 0, plugin);
  api->registerFunction("Super", super_signature, "clip:vnode;", create<Operation::Super>, nullptr, plugin);
  api->registerFunction("Analyse", analyse_signature, "clip:vnode;", create<Operation::Analyse>, nullptr, plugin);
  api->registerFunction("AnalyseMany", many_signature, "clip:vnode[];", many, nullptr, plugin);
  api->registerFunction("Recalculate", recalculate_signature, "clip:vnode[];", recalculate, nullptr, plugin);
  api->registerFunction("SCDetection", scene_signature, "clip:vnode;", create<Operation::SCDetection>, nullptr, plugin);
  api->registerFunction("Compensate", compensate_signature, "clip:vnode;", create_render<false>, nullptr, plugin);
  api->registerFunction("Flow", flow_signature, "clip:vnode;", create_flow, nullptr, plugin);
  api->registerFunction("FlowInter", flow_inter_signature, "clip:vnode;", create_temporal<TemporalKind::Inter>, nullptr,
                        plugin);
  api->registerFunction("FlowFPS", flow_fps_signature, "clip:vnode;", create_temporal<TemporalKind::FPS>, nullptr,
                        plugin);
  api->registerFunction("FlowBlur", flow_blur_signature, "clip:vnode;", create_temporal<TemporalKind::Blur>, nullptr,
                        plugin);
  api->registerFunction("Degrain", degrain_signature, "clip:vnode;", create_render<true>, nullptr, plugin);
  api->registerFunction("VectorLengthMask", mask_signature, "clip:vnode;", create_mask<MaskKind::VectorLength>, nullptr,
                        plugin);
  api->registerFunction("SADMask", mask_signature, "clip:vnode;", create_mask<MaskKind::SAD>, nullptr, plugin);
  api->registerFunction("OcclusionMask", mask_signature, "clip:vnode;", create_mask<MaskKind::Occlusion>, nullptr,
                        plugin);
  for (std::intptr_t radius = 1; radius <= 25; ++radius) {
    const auto name = "Degrain" + std::to_string(radius);
    api->registerFunction(name.c_str(), degrain_signature, "clip:vnode;", create_render<true>,
                          reinterpret_cast<void*>(radius), plugin);
  }
  api->registerFunction("KernelInfo", "", "backend:data;target:data;fft:data;fft_lanes:int;", kernel_info, nullptr,
                        plugin);
  api->registerFunction("DepanAnalyse", depan_analysis_signature, "clip:vnode;", create_depan<DepanBridge<true>>,
                        nullptr, plugin);
  api->registerFunction("DepanCompensate", depan_compensation_signature, "clip:vnode;",
                        create_depan<DepanBridge<false>>, nullptr, plugin);
  api->registerFunction("DepanEstimate", depan_estimate_signature, "clip:vnode;", create_depan<DepanEstimateBridge>,
                        nullptr, plugin);
  api->registerFunction("DepanStabilise", depan_stabilise_signature, "clip:vnode;", create_depan<DepanStabiliseBridge>,
                        nullptr, plugin);
}
