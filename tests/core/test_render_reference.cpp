#include "core/render/reference.hpp"

#include <iostream>

namespace {
using namespace neo_mv;

void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("render reference assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E = std::invalid_argument, class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const E&) {
    caught = true;
  }
  CHECK(caught);
}

SuperPlan<std::uint8_t> plan(bool chroma = false) {
  return SuperPlan<std::uint8_t>({16, 16, 8, 8, 0, 0, 4, 4, chroma ? 2 : 1, chroma ? 2 : 1, 2, chroma}, 8);
}
RenderVideo video(bool chroma = false) {
  return {16, 16, 8, chroma, chroma ? 2 : 1, chroma ? 2 : 1, 5};
}
AnalysisField field(std::int32_t delta = -1) {
  return {super_analysis_metadata(plan(), delta),
          FieldState::complete,
          {2, 2, {{{0, 0}, 400}, {{0, 0}, 401}, {{0, 0}, 900}, {{0, 0}, 0}}}};
}

void geometry() {
  auto super = plan();
  auto clip = video();
  auto m = super_analysis_metadata(super, 0);
  const auto coverage = validate_render_geometry(clip, super, 5, m, 6);
  CHECK(coverage.width == 16 && coverage.height == 16);
  // Frame-zero arrays and vector-carrier pixel format are not geometry inputs.
  m.bits = 10;
  m.levels = 100;
  validate_render_geometry(clip, super, 5, m, 5);
  m.bits = 32;
  validate_render_geometry(clip, super, 5, m, 5);
  // Reverse precision separation: render float or 16-bit samples using an
  // 8-bit analysis descriptor. Thresholds still use the analysis precision.
  const SuperPlan<float> float_super(super.params(), 32);
  auto float_clip = clip;
  float_clip.bits = 32;
  auto eight_bit_analysis = m;
  eight_bit_analysis.bits = 8;
  validate_render_geometry(float_clip, float_super, 5, eight_bit_analysis, 5);
  const SuperPlan<std::uint16_t> deep_super(super.params(), 16);
  auto deep_clip = clip;
  deep_clip.bits = 16;
  validate_render_geometry(deep_clip, deep_super, 5, eight_bit_analysis, 5);
  CHECK(RenderThresholdScale{eight_bit_analysis}(400) == 400);
  rejects([&] { validate_render_geometry(clip, super, 4, m, 5); });
  rejects([&] { validate_render_geometry(clip, super, 5, m, 4); });
  auto wrong_clip = clip;
  wrong_clip.bits = 10;
  rejects([&] { validate_render_geometry(wrong_clip, super, 5, m, 5); });
  wrong_clip = clip;
  wrong_clip.width = 18;
  rejects([&] { validate_render_geometry(wrong_clip, super, 5, m, 5); });
  wrong_clip = clip;
  wrong_clip.frames = 0;
  rejects([&] { validate_render_geometry(wrong_clip, super, 0, m, 5); });
  wrong_clip = clip;
  wrong_clip.chroma = true;
  rejects([&] { validate_render_geometry(wrong_clip, super, 5, m, 5); });
  for (auto member :
       {&AnalysisMetadata::width, &AnalysisMetadata::height, &AnalysisMetadata::real_width,
        &AnalysisMetadata::real_height, &AnalysisMetadata::pad_x, &AnalysisMetadata::pad_y, &AnalysisMetadata::pel}) {
    auto bad = m;
    ++(bad.*member);
    rejects([&] { validate_render_geometry(clip, super, 5, bad, 5); });
  }
  auto bad = m;
  bad.block_width = 6; // metadata permits this; render block-size table does not
  CHECK(valid_analysis_metadata(bad));
  rejects([&] { validate_render_geometry(clip, super, 5, bad, 5); });
  bad = m;
  bad.blocks_x = INT32_MAX;
  rejects([&] { validate_render_geometry(clip, super, 5, bad, 5); });
  bad = m;
  bad.chroma = true;
  rejects([&] { validate_render_geometry(clip, super, 5, bad, 5); });

  // Coverage is independent of Super's block hints: visible 18, grid 20,
  // working 24. Neither an equality requirement nor ceil-grid reconstruction
  // may replace the public coverage inequalities.
  SuperPlan<std::uint8_t> wide({18, 16, 8, 8, 0, 0, 4, 4}, 8);
  auto wide_clip = clip;
  wide_clip.width = 18;
  m = super_analysis_metadata(wide, 0);
  m.overlap_x = 4;
  m.blocks_x = 4;
  CHECK(validate_render_geometry(wide_clip, wide, 5, m, 5).width == 20);
  m.blocks_x = 3;
  rejects([&] { validate_render_geometry(wide_clip, wide, 5, m, 5); });
  m.blocks_x = 6;
  rejects([&] { validate_render_geometry(wide_clip, wide, 5, m, 5); });

  super = plan(true);
  clip = video(true);
  m = super_analysis_metadata(super, 0, false);
  m.ratio_x = m.ratio_y = 1; // luma-only analysis does not choose render ratios
  validate_render_geometry(clip, super, 5, m, 5);
  m.chroma = true;
  rejects([&] { validate_render_geometry(clip, super, 5, m, 5); });
  m.chroma = false;
  m.overlap_x = 1;
  m.blocks_x = 3; // coverage 22 does not fit the original working width either
  // Use working width 24 and visible width 18 to isolate overlap alignment.
  SuperPlan<std::uint8_t> color_wide({18, 16, 8, 8, 0, 0, 4, 4, 2, 2, 2, true}, 8);
  clip.width = 18;
  m.width = 24;
  m.real_width = 18;
  rejects([&] { validate_render_geometry(clip, color_wide, 5, m, 5); });
  validate_render_geometry(clip, color_wide, 5, m, 5, {true, false, false});
  validate_render_geometry(clip, color_wide, 5, m, 5, {false, false, false});
  rejects([&] { validate_render_geometry(clip, color_wide, 5, m, 5, {false, false, true}); });
}

void thresholds() {
  auto m = field().metadata;
  CHECK(RenderThresholdScale{m}(400) == 400);
  CHECK(RenderThresholdScale{m}(0) == 0);
  CHECK(RenderThresholdScale{m}(2147483648LL) == 2147483648LL); // no int32 saturation
  m.bits = 10;
  CHECK(RenderThresholdScale{m}(400) == 1605);
  m.bits = 32;
  CHECK(RenderThresholdScale{m}(400) == 102800);
  m.chroma = true;
  m.ratio_x = m.ratio_y = 2;
  m.bits = 10;
  m.block_width = 16;
  CHECK(RenderThresholdScale{m}(400) == 4814);
  rejects([&] { RenderThresholdScale{m}(-1); });
  rejects<std::overflow_error>([&] { RenderThresholdScale{m}(INT64_MAX); });
  m = field().metadata;
  rejects<std::overflow_error>([&] { RenderThresholdScale{m}(INT64_MAX); });
  CHECK(RenderThresholdScale{m}(INT64_MAX - 1023) == INT64_MAX - 1023);
  m.levels = 0;
  rejects([&] { RenderThresholdScale scale(m); });
}

void availability() {
  auto f = field();
  const ReferenceAvailability available(f.metadata, 5, 400, 50);
  CHECK(available.thresholds().error == 400 && available.thresholds().count == 2);
  CHECK(!available(f, 0));
  CHECK(available(f, 1) == 0);
  CHECK(available(f, 4) == 3);
  f.grid.values[0].error = 401;
  CHECK(!available(f, 2)); // three strictly bad blocks exceed 50 percent
  f.grid.values[3].error = -1;
  rejects([&] { available(f, 0); }); // corrupt complete data still fails at boundary/scene cut
  f = field();
  f.grid.values[3].vector.x = INT32_MAX;
  rejects([&] { available(f, 0); });
  f = field();
  f.grid.values.pop_back();
  rejects([&] { available(f, 0); });
  f = field();
  f.grid.width = 1;
  rejects([&] { available(f, 0); });

  f = field();
  f.metadata.levels = 9;
  CHECK(available(f, 1) == 0);
  for (auto scalar : field_detail::scalars) {
    if (scalar.member == &AnalysisMetadata::levels)
      continue;
    auto changed = field();
    ++(changed.metadata.*(scalar.member));
    changed.state = FieldState::metadata_only;
    rejects([&] { available(changed, 0); });
  }
  f = field();
  f.metadata.chroma = true;
  f.state = FieldState::metadata_only;
  rejects([&] { available(f, 0); });
  f = field();
  f.state = FieldState::metadata_only;
  f.grid = {0, 0, {}};
  CHECK(!available(f, 1));
  f.state = FieldState::invalid_metadata;
  f.metadata = {};
  CHECK(!available(f, 1));
  rejects([&] { available(f, -1); });
  rejects([&] { available(f, 5); });
  f = field(0);
  CHECK(ReferenceAvailability(f.metadata, 5, 400, 50)(f, 3) == 3);
  f = field(1);
  CHECK(!ReferenceAvailability(f.metadata, 5, 400, 50)(f, 4));
  CHECK(ReferenceAvailability(f.metadata, 5, 400, 50)(f, 0) == 1);
  f = field(INT32_MIN);
  CHECK(!ReferenceAvailability(f.metadata, 5, 400, 50)(f, 0));
  f = field(INT32_MAX);
  CHECK(!ReferenceAvailability(f.metadata, 5, 400, 50)(f, 0));
  CHECK(ReferenceAvailability(f.metadata, INT64_MAX, 400, 50)(f, 0) == INT32_MAX);
  rejects<std::overflow_error>([&] { ReferenceAvailability(f.metadata, INT64_MAX, 400, 50)(f, INT64_MAX - 1); });
  rejects([&] { ReferenceAvailability bad(f.metadata, 0, 400, 50); });
  rejects([&] { ReferenceAvailability bad({}, 5, 400, 50); });
}

void member_matching() {
  auto a = field(-1).metadata;
  auto b = a;
  b.levels = 7;
  CHECK(same_render_analysis(a, b));
  b.delta = 1;
  CHECK(!same_render_analysis(a, b));
  CHECK(same_render_analysis(a, b, false));
  b.bits = 10;
  CHECK(!same_render_analysis(a, b, false));
}
} // namespace

int main() {
  try {
    geometry();
    thresholds();
    availability();
    member_matching();
    std::cout << "Render geometry, thresholds and reference availability checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
