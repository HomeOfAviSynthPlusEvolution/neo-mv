#include "core/flow/frame.hpp"
#include "highway/flow.hpp"

#include <iostream>
#include <random>

namespace {
using namespace neo_mv;
void check(bool condition) {
  if (!condition)
    throw std::runtime_error("Flow scalar/Highway mismatch");
}
void dense_fields() {
  std::mt19937 rng(261003);
  for (int width : {1, 7, 16, 33, 129})
    for (int pel : {1, 2, 4})
      for (int rx : {1, 2})
        for (int ry : {1, 2}) {
          AnalysisMetadata m;
          m.block_width = ((width + 2) / 3 + 5) / 2 * 2;
          m.block_height = 8;
          m.overlap_x = m.overlap_y = 2;
          m.blocks_x = m.blocks_y = 3;
          m.width = 3 * (m.block_width - 2) + 2;
          m.height = 20;
          m.real_width = (width + rx - 1) / rx * rx;
          m.real_height = 16;
          m.pad_x = m.pad_y = 60000;
          m.pel = pel;
          m.levels = m.ratio_x = m.ratio_y = 1;
          m.bits = 8;
          m.delta = -1;
          MotionGrid grid{3, 3, std::vector<MotionTriple>(9)};
          for (auto& v : grid.values)
            v = {{int(rng() % 100001) - 50000, int(rng() % 100001) - 50000}, 0};
          const DenseFlowPlan<> scalar(m, rx, ry);
          const DenseFlowPlan<simd::GridResamplingPlan> highway(m, rx, ry);
          for (int shift : {0, pel == 1 ? 0 : pel / 2, pel == 1 ? 0 : -pel / 2}) {
            const auto a = scalar.generate(grid, shift), b = highway.generate(grid, shift);
            check(a.width == b.width && a.height == b.height && a.x == b.x && a.y == b.y);
          }
        }
}
template <class T>
void frames(int bits) {
  std::mt19937 rng(984761);
  for (int width : {8, 16, 24, 40})
    for (int pel : {1, 2, 4}) {
      const SuperPlan<T> super({width, 16, 8, 8, 0, 0, 8, 8, 2, 2, pel, true}, bits);
      const auto m = super_analysis_metadata(super, 1, false);
      const auto g = super_sampling_geometry(super)[0];
      std::vector<super_detail::PlaneBuffer<T>> storage;
      RenderImage<T> ref{&super, {}};
      RenderPixels<T> clip;
      for (int k = 0; k < 3; ++k) {
        ref.planes[k].pel = pel;
        for (int a = 0; a < pel * pel; ++a) {
          const auto e = g.planes[k].reference[a];
          storage.emplace_back(e.width, e.height);
          auto view = storage.back().view();
          for (int y = 0; y < e.height; ++y)
            for (int x = 0; x < e.width; ++x) {
              if constexpr (std::is_same_v<T, float>) {
                const auto pattern = rng(); // Includes arbitrary float representations.
                const std::uint32_t raw = static_cast<std::uint32_t>(pattern);
                std::memcpy(view.row(y).data() + x, &raw, sizeof(raw));
              } else
                view.row(y)[x] = T(rng() & ((1u << bits) - 1));
            }
        }
      }
      std::size_t index = 0;
      for (int k = 0; k < 3; ++k) {
        for (int a = 0; a < pel * pel; ++a)
          ref.planes[k].planes[a] = storage[index++].view();
        clip[k] = ref.planes[k].planes[0].subplane(g.planes[k].pad_x, g.planes[k].pad_y, width / (k ? 2 : 1),
                                                   16 / (k ? 2 : 1));
      }
      AnalysisField field{m,
                          FieldState::complete,
                          {m.blocks_x, m.blocks_y, std::vector<MotionTriple>(std::size_t(m.blocks_x) * m.blocks_y)}};
      for (auto& v : field.grid.values)
        v = {{int(rng() % 9) - 4, int(rng() % 9) - 4}, 0};
      const RenderVideo video{width, 16, bits, true, 2, 2, 2};
      for (double time : {0.0, 50.0, 100.0}) {
        FlowParameters parameters;
        parameters.time = time;
        parameters.fields = true;
        parameters.tff = false;
        const FlowFramePlan<T> scalar(video, super, 2, m, 2, parameters);
        const FlowFramePlan<T, HighwayFlowKernels<T>> highway(video, super, 2, m, 2, parameters);
        for (int n : {0, 1}) {
          const auto a = scalar.render(clip, field, n, n ? nullptr : &ref);
          const auto b = highway.render(clip, field, n, n ? nullptr : &ref);
          check(a.size() == b.size());
          for (std::size_t k = 0; k < a.size(); ++k) {
            const auto av = a[k].view(), bv = b[k].view();
            for (int y = 0; y < av.height(); ++y)
              check(std::memcmp(av.row(y).data(), bv.row(y).data(), std::size_t(av.width()) * sizeof(T)) == 0);
          }
        }
      }
    }
}
} // namespace
int main() {
  try {
    dense_fields();
    frames<std::uint8_t>(8);
    frames<std::uint16_t>(10);
    frames<std::uint16_t>(16);
    frames<float>(32);
    std::cout << "Flow scalar/Highway exact differentials passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
