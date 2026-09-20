#pragma once
#include "core/motion/composition.hpp"

// Synthetic logical Super buffers only; no host or legacy motion implementation.
template <class T>
struct MotionFixture {
  neo_mv::AnalysisMetadata metadata;
  neo_mv::SamplingGeometry geometry;
  neo_mv::SamplingFrames<T> frames;
  std::array<std::vector<T>, 3> source;
  std::array<std::array<std::vector<T>, 16>, 3> reference;

  MotionFixture(int width, int height, int block_width, int block_height, int pad = 4, int pel = 1, bool chroma = false,
                int chroma_ratio = 2) {
    auto& m = metadata;
    m.width = m.real_width = width;
    m.height = m.real_height = height;
    m.block_width = block_width;
    m.block_height = block_height;
    m.blocks_x = (width + block_width - 1) / block_width;
    m.blocks_y = (height + block_height - 1) / block_height;
    m.pad_x = m.pad_y = pad;
    m.pel = pel;
    m.levels = 1;
    m.delta = 1;
    m.chroma = chroma;
    m.ratio_x = m.ratio_y = chroma ? chroma_ratio : 1;
    m.bits = std::is_same_v<T, float> ? 32 : sizeof(T) == 1 ? 8 : 16;
    geometry.pel = pel;
    geometry.ratio_x = m.ratio_x;
    geometry.ratio_y = m.ratio_y;
    geometry.chroma = chroma;
    for (int k = 0; k < (chroma ? 3 : 1); ++k) {
      const int ratio = k == 0 ? 1 : chroma_ratio;
      auto& g = geometry.planes[k];
      g.pad_x = g.pad_y = pad / ratio;
      g.current = {width / ratio + 2 * g.pad_x, height / ratio + 2 * g.pad_y};
      const auto make = [&](std::vector<T>& data) {
        data.assign((g.current.width + 1) * g.current.height, T(0));
        return neo_mv::checked_plane<const T>(data.data(), g.current.width, g.current.height,
                                              (g.current.width + 1) * sizeof(T), data.size() * sizeof(T));
      };
      frames.current[k] = make(source[k]);
      for (int a = 0; a < pel * pel; ++a) {
        g.reference[a] = g.current;
        frames.reference[k][a] = make(reference[k][a]);
      }
    }
  }
  neo_mv::AnalysisField old_field(std::int64_t error = 0) const {
    return {metadata,
            neo_mv::FieldState::complete,
            {metadata.blocks_x, metadata.blocks_y,
             std::vector<neo_mv::MotionTriple>(std::size_t(metadata.blocks_x) * metadata.blocks_y, {{0, 0}, error})}};
  }
};
