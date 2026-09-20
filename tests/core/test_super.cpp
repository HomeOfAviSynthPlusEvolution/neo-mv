#include "core/super/pyramid.hpp"
#include "core/motion/analyse.hpp"
#include "core/motion/recalculate.hpp"
#include "core/motion/super_input.hpp"

#include <iostream>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Super assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const std::logic_error&) {
    caught = true;
  } catch (const std::overflow_error&) {
    caught = true;
  }
  CHECK(caught);
}
using namespace neo_mv;
template <class T>
struct Input {
  std::array<std::vector<T>, 3> data;
  std::array<span2d::Plane<const T>, 3> views{};
  Input(SuperGeometryParams p, T value, int scale = 1) {
    for (int k = 0; k < (p.chroma ? 3 : 1); ++k) {
      const int w = p.width * scale / (k == 0 ? 1 : p.ratio_x), h = p.height * scale / (k == 0 ? 1 : p.ratio_y);
      data[k].assign(std::size_t(w + 1) * h, value);
      views[k] = checked_plane<const T>(data[k].data(), w, h, (w + 1) * sizeof(T), data[k].size() * sizeof(T));
    }
  }
};
template <class T>
void constants() {
  const int bits = std::is_same_v<T, float> ? 32 : sizeof(T) == 1 ? 8 : 16;
  const T value = std::is_same_v<T, float> ? T(-0.125) : std::numeric_limits<T>::max();
  for (int pel : {1, 2, 4})
    for (int filter : {0, 1, 2})
      for (int sharp : {0, 1, 2}) {
        SuperGeometryParams p{32, 32, 4, 4, 0, 0, 4, 4, 2, 2, pel, true};
        Input<T> source(p, value);
        SuperPyramid<T> pyramid(SuperPlan<T>(p, bits, sharp, filter), source.views);
        const auto& g = pyramid.plan().geometry();
        CHECK(g.planes[0].levels.size() == 3);
        for (int k = 0; k < 3; ++k)
          for (std::size_t l = 0; l < g.planes[k].levels.size(); ++l) {
            const int phases = l == 0 ? pel : 1;
            for (int ay = 0; ay < phases; ++ay)
              for (int ax = 0; ax < phases; ++ax) {
                const auto view = pyramid.phase(k, static_cast<int>(l), ax, ay);
                CHECK(view.width() == g.planes[k].levels[l].padded_width - (phases == 4 && ax == 3));
                CHECK(view.height() == g.planes[k].levels[l].padded_height - (phases == 4 && ay == 3));
                for (int y = 0; y < view.height(); ++y)
                  for (int x = 0; x < view.width(); ++x)
                    CHECK(view.row(y)[x] == value);
              }
          }
      }
}

void borders_reduction_and_lifetime() {
  SuperGeometryParams p{18, 10, 8, 8, 4, 4, 2, 2, 1, 1, 1, false, true};
  Input<std::uint8_t> source(p, 7);
  source.data[0][9 * 19 + 17] = 99;
  SuperPyramid<std::uint8_t> pyramid(SuperPlan<std::uint8_t>(p, 8), source.views);
  CHECK(pyramid.phase(0, 0).width() == 24 && pyramid.phase(0, 0).height() == 16);
  CHECK(pyramid.phase(0, 0).row(0)[0] == 7 && pyramid.phase(0, 0).row(2)[2] == 7);
  CHECK(pyramid.phase(0, 0).row(15)[23] == 99);
  const auto copy = pyramid;
  CHECK(copy.phase(0, 0).data() != pyramid.phase(0, 0).data());
  auto moved = std::move(pyramid);
  std::fill(source.data[0].begin(), source.data[0].end(), 0);
  CHECK(moved.phase(0, 0).row(15)[23] == 99 && copy.phase(0, 0).row(15)[23] == 99);
  rejects([&] { moved.phase(0, 0, 1); });
  rejects([&] { moved.phase(1, 0); });
  p = {32, 32, 4, 4, 0, 0, 4, 4, 1, 1, 1};
  Input<float> impulse(p, 0);
  impulse.data[0][4 * 33 + 4] = 10;
  const float expected[] = {2.5f, 1.40625f, 0.9765625f};
  for (int filter = 0; filter < 3; ++filter) {
    SuperPyramid<float> out(SuperPlan<float>(p, 32, 2, filter), impulse.views);
    CHECK(out.phase(0, 1).row(6)[6] == expected[filter]);
    CHECK(out.phase(0, 1).row(0)[0] == 0);
    CHECK(impulse.data[0][4 * 33 + 4] == 10);
  }
  // Odd padding makes chroma's coarse width differ from luma width / ratio.
  p = {72, 72, 8, 8, 0, 0, 3, 3, 2, 2, 1, true};
  Input<std::uint16_t> chroma(p, 123);
  SuperPyramid<std::uint16_t> out(SuperPlan<std::uint16_t>(p, 10), chroma.views);
  CHECK(out.phase(0, 2).width() == 24 && out.phase(1, 2).width() == 10);
}

void external_and_errors() {
  SuperGeometryParams p{16, 16, 4, 4, 0, 0, 4, 4, 1, 1, 4};
  Input<float> source(p, 7), external(p, 0, 4);
  for (int y = 0; y < 64; ++y)
    for (int x = 0; x < 64; ++x)
      external.data[0][y * 65 + x] =
          x % 4 == 0 && y % 4 == 0 ? std::numeric_limits<float>::quiet_NaN() : float(10 + (y % 4) * 4 + x % 4);
  SuperPyramid<float> out(SuperPlan<float>(p, 32, 2, 2, true), source.views, external.views);
  CHECK(out.phase(0, 0).row(4)[4] == 7 && out.phase(0, 1).row(4)[4] == 7);
  for (int ay = 0; ay < 4; ++ay)
    for (int ax = 0; ax < 4; ++ax)
      if (ax || ay) {
        auto view = out.phase(0, 0, ax, ay);
        CHECK(view.width() == 24 && view.height() == 24);
        CHECK(view.row(0)[0] == 10 + ay * 4 + ax && view.row(23)[23] == 10 + ay * 4 + ax);
      }
  rejects([&] { SuperPyramid<float>(SuperPlan<float>(p, 32, 2, 1, true), source.views); });
  p.pel = 1;
  SuperPyramid<float> unused(SuperPlan<float>(p, 32, 2, 1, true), source.views);
  CHECK(unused.phase(0, 0).row(4)[4] == 7);
  source.data[0][0] = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { SuperPyramid<float>(SuperPlan<float>(p, 32), source.views); });
  rejects([&] { SuperPlan<float>(p, 16); });
  rejects([&] { SuperPlan<float>(p, 32, 3); });
  rejects([&] { SuperPlan<float>(p, 32, 2, -1); });
  rejects([&] { super_detail::sample_count<float>(INT32_MAX, INT32_MAX); });
  p = {4, 4, 4, 4, 0, 0, 1, 1, 2, 2, 2, true, true};
  rejects([&] { SuperPlan<std::uint8_t>(p, 8, 2); });
  SuperPlan<std::uint8_t> external_small(p, 8, 2, 1, true);
  Input<std::uint8_t> small(p, 1), doubled(p, 2, 2);
  SuperPyramid<std::uint8_t> small_out(external_small, small.views, doubled.views);
  CHECK(small_out.phase(1, 0, 1, 1).row(0)[0] == 2);
  SuperPlan<std::uint8_t> replacement(p, 8, 0, 0, true);
  p.pel = 1;
  auto assignment_plan = SuperPlan<std::uint8_t>(p, 8);
  assignment_plan = replacement;
  CHECK(assignment_plan.params().pel == 2 && assignment_plan.filter() == 0 && assignment_plan.external());
  SuperPyramid<std::uint8_t> assigned(SuperPlan<std::uint8_t>(p, 8), small.views);
  assigned = small_out;
  CHECK(assigned.phase(1, 0, 1, 1).row(0)[0] == 2);
  CHECK(assigned.phase(1, 0, 1, 1).data() != small_out.phase(1, 0, 1, 1).data());
  assigned = std::move(small_out);
  CHECK(assigned.phase(1, 0, 1, 1).row(0)[0] == 2);
}

void analysis_integration() {
  // Consume only public logical views, including restricted built-in quarters.
  SuperGeometryParams p{32, 32, 4, 4, 0, 0, 4, 4, 2, 2, 4, true};
  Input<std::uint8_t> input(p, 42);
  SuperPyramid<std::uint8_t> current(SuperPlan<std::uint8_t>(p, 8), input.views);
  const auto reference = current;
  const auto geometries = super_sampling_geometry(current.plan());
  const auto frames = borrow_super_frames(current, reference);
  auto m = super_analysis_metadata(current.plan(), 1);
  const auto vectors = analyse_vectors<std::uint8_t>(m, geometries, frames);
  CHECK(vectors.values.size() == 64);
  for (const auto& v : vectors.values)
    CHECK(v.vector.x == 0 && v.vector.y == 0 && v.error == 0);
  AnalysisField field{m, FieldState::complete, vectors};
  m.levels = 1;
  const auto refined = recalculate_vectors(field, m, geometries[0], frames[0]);
  for (const auto& v : refined.values)
    CHECK(v.vector.x == 0 && v.vector.y == 0 && v.error == 0);
  const auto luma_geometry = super_sampling_geometry(current.plan(), false);
  const auto luma_frames = borrow_super_frames(current, reference, false);
  const auto luma_metadata = super_analysis_metadata(current.plan(), -1, false);
  CHECK(!luma_metadata.chroma && luma_metadata.ratio_x == 2 && luma_metadata.delta == -1);
  const auto luma = analyse_vectors<std::uint8_t>(luma_metadata, luma_geometry, luma_frames);
  CHECK(luma.values.size() == 64 && luma.values.back().error == 0);
  CHECK(luma_frames[0].reference[1][0].data() == nullptr);
  rejects([&] { validate_super_pair(current.plan(), SuperPlan<std::uint8_t>(p, 8, 2, 1, true)); });
  auto changed = p;
  changed.pel = 2;
  rejects([&] { validate_super_pair(current.plan(), SuperPlan<std::uint8_t>(changed, 8)); });
  changed = p;
  changed.one_level = true;
  rejects([&] { validate_super_pair(current.plan(), SuperPlan<std::uint8_t>(changed, 8)); });
  changed = p;
  changed.pad_x = 5;
  rejects([&] { validate_super_pair(current.plan(), SuperPlan<std::uint8_t>(changed, 8)); });
  changed = p;
  changed.ratio_y = 1;
  rejects([&] { validate_super_pair(current.plan(), SuperPlan<std::uint8_t>(changed, 8)); });
  validate_super_pair(current.plan(), SuperPlan<std::uint8_t>(p, 8, 0, 0));
  SuperPlan<std::uint16_t> depth10(p, 10), depth12(p, 12);
  rejects([&] { validate_super_pair(depth10, depth12); });

  // Distinct images detect accidentally binding both roles to one payload.
  p = {4, 4, 4, 4, 0, 0, 4, 4, 1, 1, 1, false, true};
  Input<std::uint8_t> left(p, 0), right(p, 0);
  const std::uint8_t a[] = {10, 20, 30, 30}, b[] = {0, 10, 20, 30};
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x) {
      left.data[0][y * 5 + x] = a[x];
      right.data[0][y * 5 + x] = b[x];
    }
  const SuperPlan<std::uint8_t> plan(p, 8);
  const SuperPyramid<std::uint8_t> moving(plan, left.views), shifted(plan, right.views);
  const auto moving_geometry = super_sampling_geometry(plan);
  const auto moving_frames = borrow_super_frames(moving, shifted);
  CHECK(block_error(moving_geometry[0], {0, 0, 4, 4}, moving_frames[0], {0, 0}, BlockMetric::sad).raw == 120);
  AnalyseControls controls;
  controls.search = 4;
  controls.pnew = controls.pzero = 0;
  const auto motion =
      analyse_vectors<std::uint8_t>(super_analysis_metadata(plan, 1), moving_geometry, moving_frames, controls);
  CHECK(motion.values[0].vector.x == 1 && motion.values[0].vector.y == 0 && motion.values[0].error == 0);
}
} // namespace
int main() {
  try {
    constants<std::uint8_t>();
    constants<std::uint16_t>();
    constants<float>();
    borders_reduction_and_lifetime();
    external_and_errors();
    analysis_integration();
    std::cout << "Scalar Super composition checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
