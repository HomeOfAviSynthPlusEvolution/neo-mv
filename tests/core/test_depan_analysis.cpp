#include "core/depan/analysis.hpp"

#include <iostream>

namespace {
using namespace neo_mv;
using namespace neo_mv::depan;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Depan analysis assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  } catch (const std::overflow_error&) {
    caught = true;
  }
  CHECK(caught);
}
AnalysisMetadata metadata(int nx = 2, int ny = 2) {
  AnalysisMetadata m;
  m.width = m.real_width = nx * 8;
  m.height = m.real_height = ny * 8;
  m.pad_x = m.pad_y = 16;
  m.pel = 2;
  m.levels = m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = 8;
  m.blocks_x = nx;
  m.blocks_y = ny;
  m.bits = 8;
  m.delta = -1;
  return m;
}
AnalysisField field(AnalysisMetadata m) {
  return {m,
          FieldState::complete,
          {m.blocks_x, m.blocks_y, std::vector<MotionTriple>(std::size_t(m.blocks_x) * m.blocks_y, {{0, 0}, 0})}};
}
Observations observations(int nx, int ny, bool mask = true) {
  Observations out{nx, ny, true, mask, 400, {}};
  for (int by = 0; by < ny; ++by)
    for (int bx = 0; bx < nx; ++bx)
      out.values.push_back({bx * 8 + 4, by * 8 + 4, 0, 0, 0, 1});
  return out;
}

void input_and_mask() {
  auto m = metadata(3, 1);
  m.pad_x = 91;
  const AnalysisInputPlan plan(m, 16, 16, 3, 3, 400, 51, MaskDescription{16, 16, 3});
  auto current = field(m);
  current.grid.values[0].vector = {-3, 2};
  std::vector<std::uint8_t> storage(19 * 16, 213);
  storage[4 * 19 + 4] = 128;
  storage[4 * 19 + 12] = 0;
  const auto mask = checked_plane(static_cast<const std::uint8_t*>(storage.data()), 16, 16, 19, storage.size());
  const auto out = plan.observe(current, mask);
  CHECK(out.eligible && out.masked && out.values.size() == 3);
  CHECK(out.values[0].x == 4 && out.values[0].y == 4);
  CHECK(out.values[1].x == 12 && out.values[2].x == 20);
  CHECK(out.values[0].dx == -1.5f && out.values[0].dy == 1.0f);
  CHECK(out.values[0].base == 128 && out.values[1].base == 0 && out.values[2].base == 1);
  const auto unmasked = AnalysisInputPlan(m, 16, 16, 3, 3).observe(current);
  CHECK(!unmasked.masked);
  for (const auto& value : unmasked.values)
    CHECK(value.base == 1);
  rejects([&] { plan.observe(current); });
  current.state = FieldState::metadata_only;
  CHECK(!plan.observe(current).eligible); // No fitting samples; the host still acquires mask[n].
  current.metadata.levels = 7;
  CHECK(!plan.observe(current).eligible);
  current.metadata.delta = 1;
  CHECK(!plan.observe(current).eligible);
  current.state = FieldState::invalid_metadata;
  CHECK(!plan.observe(current).eligible);
  current = field(m);
  for (auto& value : current.grid.values)
    value.error = 401;
  CHECK(!plan.observe(current).eligible);
  current.grid.values.back().error = -1;
  rejects([&] { plan.observe(current); });
  CHECK(plan.vector_index(0) == 0 && plan.vector_index(2) == 2);
  m.delta = 1;
  const AnalysisInputPlan positive(m, 16, 16, 3, 3);
  CHECK(positive.vector_index(0) == 0 && positive.vector_index(2) == 1);
  rejects([&] { positive.vector_index(3); });
  m.delta = 0;
  rejects([&] { AnalysisInputPlan(m, 16, 16, 3, 3); });
  m = metadata();
  rejects([&] { AnalysisInputPlan(m, 16, 16, 3, 2); });
  rejects([&] { AnalysisInputPlan(m, 16, 16, 3, 3, 400, 51, MaskDescription{8, 16, 3}); });
  rejects([&] { AnalysisInputPlan(m, 16, 16, 3, 3, 400, 51, MaskDescription{16, 16, 2}); });
  rejects([&] { AnalysisInputPlan(m, 16, 16, 3, 3, 400, 51, MaskDescription{16, 16, 3, 16}); });
}

void current_metadata() {
  const auto initial = metadata();
  const AnalysisInputPlan plan(initial, 48, 48, 3, 3);
  auto current = field(metadata(3, 2));
  current.metadata.delta = 1;
  current.metadata.pel = 4;
  current.metadata.block_width = 12;
  current.metadata.overlap_x = 4;
  current.grid.values[0].vector = {4, -2};
  auto result = plan.observe(current);
  CHECK(result.eligible && result.nx == 3 && result.ny == 2 && result.values.size() == 6);
  CHECK(result.values[0].x == 6 && result.values[1].x == 14 && result.values[2].x == 22);
  CHECK(result.values[0].y == 4 && result.values[3].y == 12);
  CHECK(result.values[0].dx == 1 && result.values[0].dy == -0.5f);
  CHECK(plan.vector_index(2) == 2 && plan.metadata().delta == -1);
  CHECK(result.sad_threshold == 400 && plan.thresholds().count == 2.04f);

  // The current six-block grid does not increase the saved count threshold.
  current = field(metadata(3, 2));
  current.grid.values[0].error = current.grid.values[1].error = 401;
  CHECK(plan.observe(current).eligible);
  current.grid.values[2].error = 401;
  CHECK(!plan.observe(current).eligible);
  current.grid.values.back().error = -1;
  rejects([&] { plan.observe(current); }); // Validate even after K exceeds T2.

  for (const int bits : {10, 16, 32}) {
    current = field(initial);
    current.metadata.bits = bits;
    current.metadata.chroma = true;
    current.metadata.ratio_x = current.metadata.ratio_y = 2;
    for (auto& value : current.grid.values)
      value.error = 400;
    CHECK(plan.observe(current).eligible);
    for (auto& value : current.grid.values)
      value.error = 401;
    CHECK(!plan.observe(current).eligible); // Neither depth nor chroma rescales T1.
  }

  current = field(initial);
  current.metadata.pad_x = 17;
  current.grid.values[0].vector.x = -33;
  CHECK(plan.observe(current).eligible);
  current.metadata.pad_x = 16;
  rejects([&] { plan.observe(current); }); // Current bounds, not creation bounds.

  current = field(metadata(3, 2));
  auto properties = encode_analysis_field(current);
  const auto read = [&](const std::string& name) -> IntegerPropertyView {
    const auto found = properties.find(name);
    return found == properties.end() ? IntegerPropertyView{}
                                     : IntegerPropertyView{true, found->second.size(), found->second.data()};
  };
  properties.erase("MVUtensilsAnalysisDeltaFrame");
  CHECK(plan.read(read).eligible && plan.vector_index(2) == 2);
  properties["MVUtensilsAnalysisVectors"].resize(4);
  properties["MVUtensilsAnalysisSAD"].resize(4);
  CHECK(!plan.read(read).eligible); // Counts are compared to current 3x2, not saved 2x2.
  properties = encode_analysis_field(current);
  properties.erase("MVUtensilsAnalysisPel");
  properties["MVUtensilsAnalysisSAD"][0] = -1;
  CHECK(!plan.read(read).eligible); // Invalid metadata precedes array validation.

  current.grid.width = 2;
  rejects([&] { plan.observe(current); });
  current = field(metadata(3, 2));
  current.grid.values.pop_back();
  rejects([&] { plan.observe(current); });
}

void metadata_only_creation() {
  const auto m = metadata();
  const auto properties = encode_analysis_field({m, FieldState::metadata_only, {0, 0, {}}});
  int array_reads = 0;
  const auto read = [&](const std::string& name) -> IntegerPropertyView {
    if (name == "MVUtensilsAnalysisVectors" || name == "MVUtensilsAnalysisSAD") {
      ++array_reads;
      return {false, 4, nullptr}; // Present arrays of the wrong native type.
    }
    const auto found = properties.find(name);
    return found == properties.end() ? IntegerPropertyView{}
                                     : IntegerPropertyView{true, found->second.size(), found->second.data()};
  };
  const auto plan = AnalysisInputPlan::from_properties(read, 48, 48, 3, 3);
  CHECK(array_reads == 0);
  rejects([&] { plan.read(read); });
  CHECK(array_reads == 2);
  const auto absent = [&](const std::string& name) -> IntegerPropertyView {
    const auto found = properties.find(name);
    return found == properties.end() ? IntegerPropertyView{}
                                     : IntegerPropertyView{true, found->second.size(), found->second.data()};
  };
  CHECK(!plan.read(absent).eligible);
}

void exact_centers() {
  const std::uint64_t midpoint = (std::uint64_t{1} << 54) + (std::uint64_t{1} << 30);
  CHECK(analysis_detail::integer32(midpoint) == 0x1p54f);
  CHECK(analysis_detail::integer32(midpoint + 1) == 0x1.000002p54f);
  CHECK(analysis_detail::integer32(midpoint - 1) == 0x1p54f);
  CHECK(analysis_detail::square32((std::uint64_t{1} << 32) + 128) == 0x1.000002p64f);
  CHECK(analysis_detail::square32(INT64_MAX) == 0x1p126f);
  auto m = metadata(3, 1);
  m.width = m.pad_x = m.block_width = INT32_MAX;
  m.real_width = 8;
  m.height = m.real_height = m.block_height = 2;
  m.pad_y = m.pel = 1;
  auto current = field(m);
  current.grid.values[1].vector.x = -1;
  current.grid.values[2].vector.x = INT32_MIN;
  const auto values = AnalysisInputPlan(m, 8, 2, 1, 1, 0, 100).observe(current);
  CHECK(values.eligible && values.values.back().x == 5368709117LL);
  CHECK(values.values.back().base == 1); // Public geometry need not fit clip or the render block table.
}

void weight_rules() {
  const Transform identity;
  auto grid = observations(9, 9, false);
  auto weights = select_weights(grid, identity, 10, 1, 1000);
  CHECK(std::count(weights.begin(), weights.end(), 1.0f) == 1);
  CHECK(weights[40] == 1);
  grid = observations(8, 8, false);
  weights = select_weights(grid, identity, 10, 1, 1000);
  CHECK(std::all_of(weights.begin(), weights.end(), [](float value) { return value == 0 && !std::signbit(value); }));
  for (auto& value : grid.values)
    value.dx = 2;
  FitParameters translation;
  translation.zoom = translation.rotation = false;
  const auto border_fit = fit(grid, translation);
  CHECK(border_fit.map.tx > 0.5f && border_fit.map.tx < 0.7f); // Initial update used every base weight.
  CHECK(border_fit.error == 1 && border_fit.iteration == 10);

  grid = observations(1, 1);
  grid.values[0].base = 100;
  grid.values[0].sad = 400;
  CHECK(select_weights(grid, identity, -1, 0.05f, 0)[0] == 5);
  grid.values[0].sad = 401;
  CHECK(select_weights(grid, identity, -1, 0.05f, 0)[0] == 0);
  grid = observations(3, 3);
  grid.values[4].dx = 2;
  CHECK(select_weights(grid, identity, 2, 1, 1000)[4] == 1);
  CHECK(select_weights(grid, identity, 1.99f, 1, 1000)[4] == 0);
  grid = observations(3, 3);
  grid.values[0].dx = 0x1p24f;
  grid.values[1].dx = 1;
  grid.values[2].dx = -0x1p24f;
  CHECK(select_weights(grid, identity, 0.1f, 1, 1000)[4] == 1); // Ordered neighbor sum is zero.

  grid = observations(1, 1);
  Transform dangerous;
  dangerous.tx = 3;
  dangerous.h = std::numeric_limits<float>::max();
  CHECK(select_weights(grid, dangerous, 10, 1, 2)[0] == 0); // Horizontal rejection skips overflowing vertical math.
  dangerous.tx = 0;
  rejects([&] { select_weights(grid, dangerous, 10, 1, 2); });
  grid.values[0].sad = 401;
  CHECK(select_weights(grid, dangerous, 10, 1, 2)[0] == 0); // SAD rejects before either residual.
  grid.values[0].sad = 0;
  CHECK(select_weights(grid, identity, 10, 1, 2)[0] == 1); // Rejection is not permanent.
}

void fitting() {
  auto grid = observations(1, 1);
  grid.values[0].dx = 2;
  const auto step = fit_update(grid, {1}, Transform{}, 1, 0.3f, false, false);
  CHECK(step.map.tx == 0.54545456f);
  CHECK(step.error == 1.9306146f);
  CHECK(step.map.ty == 0 && step.map.u == 1 && step.map.h == 1);
  grid = observations(2, 2);
  for (auto& value : grid.values)
    value.base = 0;
  auto result = fit(grid);
  CHECK(result.good && result.error == 1 && result.iteration == 10);
  CHECK(result.map.tx == 0 && result.map.ty == 0 && result.map.u == 1 && result.map.h == 1);
  FitParameters parameters;
  parameters.error = 1;
  CHECK(!fit(grid, parameters).good);
  parameters.error = -1;
  CHECK(!fit(grid, parameters).good);
  const Observations ineligible;
  result = fit(ineligible, parameters);
  CHECK(result.good && result.error == -2 && result.iteration == 0);
  parameters.error = 0;
  CHECK(!fit(ineligible, parameters).good);
  parameters.error = 15;
  CHECK(!fit(ineligible, parameters).good && fit(ineligible, parameters).error == 30);
  parameters.error = std::numeric_limits<float>::max();
  rejects([&] { fit(ineligible, parameters); });
  parameters = {};
  parameters.aspect = std::numeric_limits<float>::denorm_min();
  rejects([&] { fit(ineligible, parameters); });
  grid = observations(1, 1);
  rejects([&] { fit_update(grid, {-1}, Transform{}, 1, 0.3f, false, false); });    // Negative R/N.
  rejects([&] { fit_update(grid, {-0.1f}, Transform{}, 1, 0.3f, false, false); }); // Zero divisor even with flags off.
  parameters = {};
  parameters.zerow = -1;
  rejects([&] { fit(grid, parameters); }); // Negative zerow is accepted until its required arithmetic fails.
}

void complete_example() {
  const auto m = metadata();
  auto current = field(m);
  current.grid.values = {{{1, -3}, 0}, {{0, 0}, 0}, {{4, 2}, 0}, {{-2, 1}, 0}};
  std::vector<std::uint8_t> storage(48 * 48, 1);
  const auto mask = checked_plane(static_cast<const std::uint8_t*>(storage.data()), 48, 48, 48, storage.size());
  const AnalysisInputPlan plan(m, 48, 48, 3, 3, 400, 51, MaskDescription{48, 48, 3});
  FitParameters parameters;
  parameters.zoom = parameters.rotation = false;
  const auto result = fit(plan.observe(current, mask), parameters);
  CHECK(result.good && result.iteration == 10);
  CHECK(result.map.tx == 0.49161040782928467f && result.map.ty == 0);
  CHECK(result.map.u == 1 && result.map.h == 1 && result.map.v == 0 && result.map.w == 0);
  CHECK(result.error == 1.6047950983047485f);
  const auto output = motion(result.map, 1, 24, 24, true);
  CHECK(output.dx == 0.4916095733642578f && output.dy == 0);
  CHECK(output.rotation == 0 && std::signbit(output.rotation) && output.zoom == 1 && output.good);
}
} // namespace

int main() {
  try {
    input_and_mask();
    current_metadata();
    metadata_only_creation();
    exact_centers();
    for (std::uint64_t limit :
         {std::uint64_t{4096}, std::uint64_t{1} << 24, std::uint64_t{1} << 26, std::uint64_t{1} << 53})
      for (std::uint64_t value = limit - 8; value <= limit + 8; ++value) {
        CHECK(analysis_detail::integer32(value) == analysis_detail::Integer::product(value, 1).rounded());
        CHECK(analysis_detail::square32(value) == analysis_detail::Integer::product(value, value).rounded());
      }
    weight_rules();
    fitting();
    complete_example();
    std::cout << "Depan analysis checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
