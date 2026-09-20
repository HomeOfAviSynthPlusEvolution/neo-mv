#pragma once
#include "common.hpp"

namespace neo_mv::ds2 {
inline std::string descriptor_key(const std::string& prefix) {
  return prefix + "NeoMVSuperDescriptorV1";
}
inline std::string planes_key(const std::string& prefix) {
  return prefix + "NeoMVSuperPlanesV1";
}

template <class T>
std::vector<std::int64_t> describe(const SuperPlan<T>& plan) {
  const auto& p = plan.params();
  return {1,           p.width,     p.height,    p.block_width, p.block_height, p.overlap_x,
          p.overlap_y, p.pad_x,     p.pad_y,     p.ratio_x,     p.ratio_y,      p.pel,
          p.chroma,    p.one_level, plan.bits(), plan.sharp(),  plan.filter(),  plan.external()};
}
template <class T>
SuperPlan<T> restore_plan(const ds::FrameProperties& props, const std::string& prefix) {
  auto d = property_array<std::int64_t>(props, descriptor_key(prefix));
  require(d.size() == 18 && d[0] == 1, "unsupported or missing private Super descriptor");
  for (auto value : d)
    require(value >= INT32_MIN && value <= INT32_MAX, "Super descriptor exceeds int32");
  for (auto i : {12, 13, 17})
    require(d[i] == 0 || d[i] == 1, "invalid Super descriptor boolean");
  SuperGeometryParams p{int(d[1]), int(d[2]), int(d[3]),  int(d[4]),  int(d[5]),  int(d[6]), int(d[7]),
                        int(d[8]), int(d[9]), int(d[10]), int(d[11]), d[12] != 0, d[13] != 0};
  return SuperPlan<T>(p, int(d[14]), int(d[15]), int(d[16]), d[17] != 0);
}
template <class T>
AnalysisProperties super_properties(const SuperPlan<T>& plan, const std::string& prefix) {
  const auto& p = plan.params();
  const auto& g = plan.geometry();
  AnalysisProperties result;
  auto set = [&](const char* suffix, std::int64_t value) {
    result[prefix + suffix] = {value};
  };
  set("SuperWidth", g.planes[0].levels[0].width);
  set("SuperHeight", g.planes[0].levels[0].height);
  set("SuperRealWidth", p.width);
  set("SuperRealHeight", p.height);
  set("SuperHPad", p.pad_x);
  set("SuperVPad", p.pad_y);
  set("SuperPel", p.pel);
  set("SuperLevels", g.planes[0].levels.size());
  set("SuperChroma", p.chroma);
  set("SuperXRatioUV", p.ratio_x);
  set("SuperYRatioUV", p.ratio_y);
  set("SuperBitsPerSample", plan.bits());
  set("SuperBlkSizeX", p.block_width);
  set("SuperBlkSizeY", p.block_height);
  set("SuperOverlapX", p.overlap_x);
  set("SuperOverlapY", p.overlap_y);
  return result;
}
template <class T>
void publish_super(const SuperPyramid<T>& pyramid, ds::FrameFactory& factory, ds::FrameProperties& props,
                   const std::string& prefix, ds::SampleFormat sample) {
  const auto& plan = pyramid.plan();
  const auto& g = plan.geometry();
  std::vector<ds::FrameRef> owners;
  for (int k = 0; k < g.plane_count; ++k)
    for (std::size_t l = 0; l < g.planes[k].levels.size(); ++l) {
      const int pel = l == 0 ? plan.params().pel : 1;
      for (int ay = 0; ay < pel; ++ay)
        for (int ax = 0; ax < pel; ++ax) {
          const auto input = pyramid.phase(k, int(l), ax, ay);
          auto aux = factory.allocate({ds::ColorFamily::Gray, sample, 1, 0, 0}, input.width(), input.height());
          {
            auto output = plane<T>(aux.view().plane(0));
            for (int y = 0; y < input.height(); ++y)
              std::memcpy(output.row(y).data(), input.row(y).data(), std::size_t(input.width()) * sizeof(T));
          }
          owners.push_back(std::move(aux).publish());
        }
    }
  // Replace both private keys; native frame properties own auxiliary frame refs.
  props.set(planes_key(prefix), owners);
  props.set(descriptor_key(prefix), describe(plan));
  for (const auto& entry : super_properties(plan, prefix))
    props.set(entry.first, entry.second);
}

template <class T>
class FrameSuper {
  std::vector<ds::FrameRef> owners_;
  std::array<std::vector<std::array<span2d::Plane<const T>, 16>>, 3> planes_;

public:
  SuperPlan<T> plan;
  FrameSuper(const ds::VideoFrameView& carrier, const ds::VideoInputInfo& info, const std::string& prefix)
      : plan(restore_plan<T>(properties(carrier), prefix)) {
    validate_frame(carrier, info);
    const auto& p = plan.params();
    require(p.width == info.width && p.height == info.height &&
                plan.bits() == ds::bits_per_sample(info.format.sample_format) &&
                p.chroma == (info.format.color_family == ds::ColorFamily::Yuv) &&
                p.ratio_x == (1 << info.format.subsampling_w) && p.ratio_y == (1 << info.format.subsampling_h),
            "Super descriptor disagrees with video format");
    const auto& props = properties(carrier);
    for (const auto& entry : super_properties(plan, prefix))
      require(scalar(props, entry.first) == entry.second[0], "Super public metadata disagrees with payload");
    owners_ = property_array<ds::FrameRef>(props, planes_key(prefix));
    const auto all = super_sampling_geometry(plan);
    std::size_t expected = 0;
    for (int k = 0; k < plan.geometry().plane_count; ++k)
      for (const auto& level : plan.geometry().planes[k].levels)
        expected += level.phase_count;
    require(owners_.size() == expected, "Super auxiliary frame count mismatch");
    std::size_t index = 0;
    for (int k = 0; k < plan.geometry().plane_count; ++k) {
      planes_[k].resize(all.size());
      for (std::size_t l = 0; l < all.size(); ++l)
        for (int a = 0; a < all[l].pel * all[l].pel; ++a) {
          const auto f = owners_[index++].view();
          const auto extent = all[l].planes[k].reference[a];
          require(f.format == ds::VideoFormat{ds::ColorFamily::Gray, info.format.sample_format, 1, 0, 0} &&
                      f.plane_count == 1,
                  "Super auxiliary sample format mismatch");
          auto view = plane<T>(f.plane(0));
          require(view.width() == extent.width && view.height() == extent.height,
                  "Super auxiliary read domain mismatch");
          planes_[k][l][a] = view;
        }
    }
  }
  span2d::Plane<const T> phase(int k, int l, int a = 0) const { return planes_.at(k).at(l).at(a); }
};
template <class T>
std::vector<SamplingFrames<T>> sample_frames(const FrameSuper<T>& a, const FrameSuper<T>& b, bool chroma) {
  validate_super_pair(a.plan, b.plan);
  auto geometry = super_sampling_geometry(a.plan, chroma);
  std::vector<SamplingFrames<T>> result(geometry.size());
  for (std::size_t l = 0; l < geometry.size(); ++l)
    for (int k = 0; k < (geometry[l].chroma ? 3 : 1); ++k) {
      result[l].current[k] = a.phase(k, int(l));
      for (int phase = 0; phase < geometry[l].pel * geometry[l].pel; ++phase)
        result[l].reference[k][phase] = b.phase(k, int(l), phase);
    }
  return result;
}
} // namespace neo_mv::ds2
