#include "core/super/pyramid.hpp"
#include "core/motion/analyse.hpp"

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
}

void analysis_integration() {
  // Consume only public logical views, including restricted built-in quarters.
  SuperGeometryParams p{32, 32, 4, 4, 0, 0, 4, 4, 2, 2, 4, true};
  Input<std::uint8_t> input(p, 42);
  SuperPyramid<std::uint8_t> current(SuperPlan<std::uint8_t>(p, 8), input.views);
  const auto reference = current;
  std::vector<SamplingGeometry> geometries;
  std::vector<SamplingFrames<std::uint8_t>> frames;
  const auto& g = current.plan().geometry();
  for (std::size_t l = 0; l < g.planes[0].levels.size(); ++l) {
    SamplingGeometry sg;
    sg.pel = l == 0 ? p.pel : 1;
    sg.ratio_x = sg.ratio_y = 2;
    sg.chroma = true;
    SamplingFrames<std::uint8_t> sf;
    for (int k = 0; k < 3; ++k) {
      sf.current[k] = current.phase(k, static_cast<int>(l));
      sg.planes[k].pad_x = g.planes[k].pad_x;
      sg.planes[k].pad_y = g.planes[k].pad_y;
      sg.planes[k].current = {sf.current[k].width(), sf.current[k].height()};
      for (int ay = 0; ay < sg.pel; ++ay)
        for (int ax = 0; ax < sg.pel; ++ax) {
          auto v = reference.phase(k, static_cast<int>(l), ax, ay);
          sf.reference[k][ay * sg.pel + ax] = v;
          sg.planes[k].reference[ay * sg.pel + ax] = {v.width(), v.height()};
        }
    }
    geometries.push_back(sg);
    frames.push_back(sf);
  }
  AnalysisMetadata m;
  m.width = m.height = m.real_width = m.real_height = 32;
  m.pad_x = m.pad_y = 4;
  m.block_width = m.block_height = 4;
  m.pel = 4;
  m.delta = 1;
  m.bits = 8;
  m.chroma = true;
  m.ratio_x = m.ratio_y = 2;
  const auto vectors = analyse_vectors<std::uint8_t>(m, geometries, frames);
  CHECK(vectors.values.size() == 64);
  for (const auto& v : vectors.values)
    CHECK(v.vector.x == 0 && v.vector.y == 0 && v.error == 0);
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
