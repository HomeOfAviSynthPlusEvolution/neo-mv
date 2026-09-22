#include "highway/scene.hpp"
#include "highway/rows.hpp"
#include "hwy/targets.h"
#include <cstring>
#include <iostream>

namespace {
using namespace neo_mv;
void check(bool ok, int line) {
  if (!ok)
    throw std::runtime_error("Highway scene assertion " + std::to_string(line));
}
#define CHECK(x) check((x), __LINE__)
template <class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
AnalysisField field(int width, int pel) {
  AnalysisMetadata m;
  m.width = m.real_width = 4 * width + 4;
  m.height = m.real_height = 16;
  m.pad_x = m.pad_y = 8;
  m.pel = pel;
  m.levels = 1;
  m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = 8;
  m.overlap_x = m.overlap_y = 4;
  m.blocks_x = width;
  m.blocks_y = 3;
  m.bits = 8;
  AnalysisField f{m, FieldState::complete, {width, 3, {}}};
  f.grid.values.resize(std::size_t(width) * 3);
  for (int y = 0; y < 3; ++y)
    for (int x = 0; x < width; ++x) {
      const auto bounds = field_detail::bounds(m, x, y);
      f.grid.values[std::size_t(y) * width + x] = {{static_cast<int>(x % 2 ? bounds.left : bounds.right - 1),
                                                    static_cast<int>(y % 2 ? bounds.top : bounds.bottom - 1)},
                                                   x % 4 == 0   ? INT64_MAX
                                                   : x % 4 == 1 ? 400
                                                   : x % 4 == 2 ? 401
                                                                : 0};
    }
  return f;
}
void run() {
  for (int width : {1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129}) {
    for (int pel : {1, 2, 4}) {
      auto f = field(width, pel);
      for (double percent : {0.0, 33.3, 50.0, 100.0}) {
        const SceneClassifier classifier(scene_descriptor(f.metadata), 400, percent);
        CHECK(classifier(f) == classifier(f, simd::scene_count));
        CHECK(classifier(f) == classifier(f, scalar_scene_count_validated));
        CHECK(classifier(f) == classifier(f, simd::scene_count_validated));
        CHECK(scalar_scene_count(f.metadata, f.grid, 400) == simd::scene_count(f.metadata, f.grid, 400));
      }
      for (const auto threshold : {std::int64_t(0), std::int64_t(400), INT64_MAX}) {
        const auto expected = scalar_scene_count(f.metadata, f.grid, threshold);
        CHECK(scalar_scene_count_validated(f.metadata, f.grid, threshold) == expected);
        CHECK(simd::scene_count_validated(f.metadata, f.grid, threshold) == expected);
      }
      const SceneClassifier classifier(scene_descriptor(f.metadata), 0, 0);
      // Every lane and tail must still be checked after an earlier bad SAD.
      for (std::size_t i = 0; i < f.grid.values.size(); ++i) {
        auto& v = f.grid.values[i];
        const auto saved = v;
        v.error = -1;
        rejects([&] { classifier(f, simd::scene_count); });
        v = saved;
        const auto bounds = field_detail::bounds(f.metadata, static_cast<int>(i % width), static_cast<int>(i / width));
        v.vector.x = static_cast<int>(bounds.right);
        rejects([&] { classifier(f, simd::scene_count); });
        v = saved;
        v.vector.y = static_cast<int>(bounds.bottom);
        rejects([&] { classifier(f, simd::scene_count); });
        v = saved;
      }
      f.grid.values.pop_back();
      rejects([&] { classifier(f, simd::scene_count); });
      f.state = FieldState::metadata_only;
      CHECK(classifier(f, simd::scene_count) == 1);
      ++f.metadata.bits;
      rejects([&] { classifier(f, simd::scene_count); });
      f.state = FieldState::invalid_metadata;
      CHECK(classifier(f, simd::scene_count) == 1);
    }
  }
  auto wide = field(17, 4);
  wide.metadata.pad_x = wide.metadata.pad_y = INT32_MAX;
  for (auto& v : wide.grid.values)
    v.vector = {INT32_MIN, INT32_MAX};
  const SceneClassifier classifier(scene_descriptor(wide.metadata), 400, 50);
  CHECK(classifier(wide) == classifier(wide, simd::scene_count));
}
} // namespace
int main() {
  try {
    for (const auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      CHECK(std::strcmp(simd::detail::target_name(), hwy::TargetName(target)) == 0);
      std::cout << "Testing " << hwy::TargetName(target) << '\n';
      run();
    }
    hwy::SetSupportedTargetsForTest(0);
  } catch (const std::exception& e) {
    hwy::SetSupportedTargetsForTest(0);
    std::cerr << e.what() << '\n';
    return 1;
  }
}
