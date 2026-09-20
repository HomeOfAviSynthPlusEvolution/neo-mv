#include "filters/common.hpp"
#include <dualsynth/vapoursynth/video_bridge.hpp>

namespace neo_mv::ds2 {
AnalysisField read_field(const ds::RequestedVideoFrame& requested, const std::string& prefix, bool vectors) {
  // Keep this dependency on DS2 internals isolated until DS2 offers public
  // property type/count inspection. No foreign properties are materialized.
  require(bool(requested.owner), "analysis decoding requires owning VS frame");
  using Native = ds::detail::NativeFrame<ds::vapoursynth::FrameTraits>;
  const auto* storage = dynamic_cast<const Native*>(&requested.owner.storage());
  require(storage != nullptr, "analysis property reader requires a native VS frame");
  const auto* api = storage->traits().api;
  const auto* map = api->getFramePropertiesRO(storage->frame());
  return read_analysis_field(
      [&](const std::string& key) -> IntegerPropertyView {
        const int count = api->mapNumElements(map, key.c_str());
        if (count < 0)
          return {};
        if (api->mapGetType(map, key.c_str()) != ptInt)
          return {false, std::uint64_t(count), nullptr};
        if (count == 0)
          return {true, 0, nullptr};
        int error = 0;
        const auto* data = api->mapGetIntArray(map, key.c_str(), &error);
        require(!error && data, "cannot read native integer property array");
        return {true, std::uint64_t(count), data};
      },
      vectors, prefix);
}
} // namespace neo_mv::ds2
