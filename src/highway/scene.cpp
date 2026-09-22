#include "highway/scene.hpp"
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/scene.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
std::int64_t SceneCount(const AnalysisMetadata& m, const MotionGrid& grid, std::int64_t threshold) {
  const hn::ScalableTag<std::int64_t> d;
  const int lanes = static_cast<int>(hn::Lanes(d));
  HWY_ALIGN std::int64_t vx[hn::MaxLanes(d)], vy[hn::MaxLanes(d)], sad[hn::MaxLanes(d)];
  const auto offsets = hn::Mul(hn::Iota(d, 0), hn::Set(d, std::int64_t(m.block_width - m.overlap_x) * m.pel));
  const auto zero = hn::Zero(d), limit = hn::Set(d, threshold);
  std::int64_t bad = 0;
  for (int y = 0; y < m.blocks_y; ++y) {
    const auto* row = grid.values.data() + std::size_t(y) * m.blocks_x;
    int x = 0;
    for (; x <= m.blocks_x - lanes; x += lanes) {
      // Pack only a bounded register-sized group; never alias MotionTriple
      // storage as an array of integers or allocate a whole-field transpose.
      for (int i = 0; i < lanes; ++i) {
        vx[i] = row[x + i].vector.x;
        vy[i] = row[x + i].vector.y;
        sad[i] = row[x + i].error;
      }
      const auto domain = field_detail::bounds(m, x, y);
      const auto dx = hn::Load(d, vx), dy = hn::Load(d, vy), error = hn::Load(d, sad);
      const auto left = hn::Sub(hn::Set(d, domain.left), offsets);
      const auto right = hn::Sub(hn::Set(d, domain.right), offsets);
      const auto valid_x = hn::And(hn::Ge(dx, left), hn::Lt(dx, right));
      const auto valid_y = hn::And(hn::Ge(dy, hn::Set(d, domain.top)), hn::Lt(dy, hn::Set(d, domain.bottom)));
      if (!hn::AllTrue(d, hn::And(hn::Ge(error, zero), hn::And(valid_x, valid_y))))
        throw std::invalid_argument("malformed analysis vector or error");
      bad += static_cast<std::int64_t>(hn::CountTrue(d, hn::Gt(error, limit)));
    }
    for (; x < m.blocks_x; ++x) {
      field_detail::vector(m, row[x], x, y);
      bad += row[x].error > threshold;
    }
  }
  return bad;
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace neo_mv::simd {
HWY_EXPORT(SceneCount);
std::int64_t scene_count(const AnalysisMetadata& m, const MotionGrid& grid, std::int64_t threshold) {
  return HWY_DYNAMIC_DISPATCH(SceneCount)(m, grid, threshold);
}
} // namespace neo_mv::simd
#endif
