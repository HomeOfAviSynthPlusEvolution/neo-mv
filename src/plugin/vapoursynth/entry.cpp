#include "plugin/bridges.hpp"
#include <dualsynth/vapoursynth/video_bridge.hpp>

namespace neo_mv::ds2 {
namespace {
using MapOwner = std::unique_ptr<VSMap, decltype(VSAPI::freeMap)>;
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
} // namespace
} // namespace neo_mv::ds2

VS_EXTERNAL_API(void) VapourSynthPluginInit2(VSPlugin* plugin, const VSPLUGINAPI* api) {
  using namespace neo_mv::ds2;
  api->configPlugin("org.neofilters.neo_mv", "neomv", "neo-mv", VS_MAKE_VERSION(0, 1), VAPOURSYNTH_API_VERSION, 0,
                    plugin);
  api->registerFunction("Super", super_signature, "clip:vnode;", create<Operation::Super>, nullptr, plugin);
  api->registerFunction("Analyse", analyse_signature, "clip:vnode;", create<Operation::Analyse>, nullptr, plugin);
  api->registerFunction("AnalyseMany", many_signature, "clip:vnode[];", many, nullptr, plugin);
  api->registerFunction("Recalculate", recalculate_signature, "clip:vnode[];", recalculate, nullptr, plugin);
  api->registerFunction("SCDetection", scene_signature, "clip:vnode;", create<Operation::SCDetection>, nullptr, plugin);
}
