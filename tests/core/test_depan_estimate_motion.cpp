#include "core/depan/estimate_motion.hpp"

#include <array>
#include <iostream>

namespace {
using namespace neo_mv;
using namespace neo_mv::depan;
using namespace neo_mv::depan::estimate;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Depan estimate motion assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F> void rejects(F&& f) {
  bool caught = false;
  try { f(); }
  catch (const std::invalid_argument&) { caught = true; }
  catch (const std::overflow_error&) { caught = true; }
  CHECK(caught);
}
struct Surface {
  // Padding must never participate, including in finite-surface validation.
  std::array<float, 24> values{};
  Surface() {
    for (int y = 0; y < 4; ++y)
      values[y * 6 + 4] = values[y * 6 + 5] = std::numeric_limits<float>::quiet_NaN();
  }
  float& at(int x, int y) { return values[y * 6 + x]; }
  span2d::Plane<const float> plane() const {
    return checked_plane(values.data(), 4, 4, 6 * sizeof(float), sizeof(values));
  }
  void fill(float value) {
    for (int y = 0; y < 4; ++y)
      for (int x = 0; x < 4; ++x)
        at(x,y) = value;
  }
};
void peaks() {
  Surface s;
  s.at(1,0) = 160;
  auto p = find_peak(s.plane(), 1, 1, 0, 4);
  CHECK(p.ix == 1 && p.iy == 0 && p.dx == 1 && p.dy == 0 && p.good);
  CHECK(p.confidence == 88.00879669189453f);
  CHECK(find_peak(s.plane(), 1, 1, 1, 4).confidence == 58.67253112792969f);
  CHECK(find_peak(s.plane(), 1, 1, 0, p.confidence).good);
  CHECK(!find_peak(s.plane(), 1, 1, 0, std::nextafter(p.confidence, 100.0f)).good);
  s.at(3,0) = 160;
  s.at(2,0) = 1000;
  CHECK(find_peak(s.plane(), 1, 1, 0, 0).ix == 1);
  s.at(1,0) = 0;
  p = find_peak(s.plane(), 1, 1, 0, 0);
  CHECK(p.ix == 3 && p.dx == -1);
  s.fill(1024);
  CHECK(find_peak(s.plane(), 1, 1, 0, 0).confidence == 0);
  s.fill(0.1f);
  CHECK(find_peak(s.plane(), 1, 1, 0, 0).confidence < 0);
  s.fill(0);
  CHECK(find_peak(s.plane(), 0, 0, 0, 0).good);
  CHECK(!find_peak(s.plane(), 0, 0, 0, 4).good);
  s.at(0,0) = -1.6f;
  rejects([&] { find_peak(s.plane(), 0, 0, 0, 0); });
  s.fill(0);
  s.at(1,0) = 160;
  rejects([&] { find_peak(s.plane(), 1, 1, -2, 100); });
  CHECK(find_peak(s.plane(), 1, 1, -1, 100).confidence > 100);
  s.at(2,2) = std::numeric_limits<float>::infinity();
  rejects([&] { find_peak(s.plane(), 1, 1, 0, 100); });
  s.fill(std::numeric_limits<float>::max());
  rejects([&] { find_peak(s.plane(), 1, 1, 0, 100); });
  rejects([&] { find_peak(s.plane(), 2, 1, 0, 0); });
  rejects([&] { find_peak(s.plane(), 0, -1, 0, 0); });
}
void scalar_grouping() {
  Surface s;
  s.at(1,1) = 160;
  const auto peak = find_peak(s.plane(), 1, 1, 0.3f, 0);
  // Preserve the sequential multiply/divide grouping, with an explicit rounded
  // reference for each scalar operation rather than a relaxed algebraic form.
  const float p = neo_mv::depan::div(160,16), mean = neo_mv::depan::div(neo_mv::depan::div(160,9),16);
  const float t = neo_mv::depan::div(mul(sub(p,mean),100),add(p,0.1f));
  const float d = add(2,mul(0.3f,1));
  CHECK(peak.confidence == mul(t,neo_mv::depan::div(mul(neo_mv::depan::div(2,d),2),d)));
  rejects([&] { find_peak(s.plane(),1,1,0,-1); });
  rejects([&] { find_peak(s.plane(),1,1,std::numeric_limits<float>::quiet_NaN(),0); });
  const std::array<float,4> tiny{10,6,8,0};
  const auto plane = checked_plane(tiny.data(),2,2,2*sizeof(float),sizeof(tiny));
  const auto tiny_peak = find_peak(plane,0,0,0,0);
  const auto tiny_motion = refine_motion(plane,tiny_peak,0,0,1,false,false);
  CHECK(tiny_motion.good && tiny_motion.dx == 0 && tiny_motion.dy == 0);
}
void refinements() {
  Surface s;
  s.at(0,0) = 10;
  s.at(3,0) = 6;
  s.at(1,0) = 8;
  const Peak origin{0,0,0,0,70,true};
  auto w = refine_motion(s.plane(), origin, 1, 1, 1, false, false);
  CHECK(w.dx == 0.1666666716337204f && w.dy == 0 && w.good && w.confidence == 70);
  CHECK(refine_motion(s.plane(), origin, 1, 1, 2, true, true).dy == 0.5f);
  CHECK(refine_motion(s.plane(), origin, 1, 1, 2, true, false).dy == -0.5f);
  s.fill(0);
  s.at(1,0) = 10;
  s.at(0,0) = 6;
  s.at(2,0) = 8;
  const Peak boundary{1,0,1,0,70,true};
  CHECK(refine_motion(s.plane(), boundary, 1, 1, 1, false, false).dx == 1);
  s.at(0,0) = 8;
  s.at(2,0) = 6;
  CHECK(refine_motion(s.plane(), boundary, 1, 1, 1, false, false).dx == 0.8333333134651184f);
  // Rejected windows skip interpolation even if its finite inputs would overflow.
  s.fill(std::numeric_limits<float>::max());
  auto bad = origin;
  bad.good = false;
  w = refine_motion(s.plane(), bad, 1, 1, 1, false, false);
  CHECK(w.dx == 0 && w.dy == 0 && !w.good && w.confidence == 70);
  rejects([&] { refine_motion(s.plane(), origin, 1, 1, 1, false, false); });
  s.fill(0);
  CHECK(refine_motion(s.plane(), origin, 1, 1, 1, false, false).dx == 0);
  rejects([&] { refine_motion(s.plane(), origin, 1, 1, 0, false, false); });
}
void combination() {
  const WindowMotion a{-1,0,80,true}, b{1,0,60,true};
  auto m = combine(a,b,1.02f,100,1);
  CHECK(!m.good && m.dx == 0 && m.dy == 0 && m.zoom == 1 && m.confidence == 60);
  m = combine(a,b,1.1f,100,1);
  CHECK(m.good && m.dx == 0 && m.dy == 0 && m.zoom == 1.0199999809265137f && m.confidence == 60);
  CHECK(!combine(a,b,0.9f,100,1).good);
  CHECK(combine(a,std::nullopt,1,0,1).good);
  m = combine(a,b,1.1f,100,0);
  CHECK(!m.good && m.confidence == 0 && !std::signbit(m.confidence));
  auto invalid = b;
  invalid.good = false;
  CHECK(!combine(a,invalid,1.1f,100,1).good);
  auto negative_zero = a, positive_zero = b;
  negative_zero.confidence = -0.0f;
  positive_zero.confidence = 0.0f;
  CHECK(std::signbit(combine(negative_zero,positive_zero,1.1f,100,1).confidence));
  rejects([&] { combine(a,std::nullopt,1.1f,100,1); });
  rejects([&] { combine(a,b,1.1f,0,1); });
  auto huge1 = a, huge2 = b;
  huge1.dx = -std::numeric_limits<float>::max();
  huge2.dx = std::numeric_limits<float>::max();
  huge1.good = huge2.good = false;
  rejects([&] { combine(huge1,huge2,1.1f,100,0); });
}
void temporal() {
  BasicMotion m{1,0,1,5,true};
  auto out = temporal_motion(m,20,std::nullopt,4);
  CHECK(!out.good && out.dx == 0 && out.dy == 0 && out.rotation == 0 && out.zoom == 1);
  CHECK(!std::signbit(out.dx) && !std::signbit(out.dy) && !std::signbit(out.rotation));
  CHECK(temporal_motion(m,10,std::nullopt,4).good);
  CHECK(temporal_motion(m,0,std::nullopt,4).good);
  CHECK(!temporal_motion(m,0,20,4).good);
  m.confidence = 8;
  CHECK(temporal_motion(m,100,100,4).good);
  CHECK(m.confidence == 8);
  CHECK(required_basic_indices(2,5) == std::vector<int>({1,2,3}));
  CHECK(required_source_indices(2,5) == std::vector<int>({0,1,2,3}));
  CHECK(required_basic_indices(0,1) == std::vector<int>({0}));
  CHECK(required_source_indices(0,1) == std::vector<int>({0}));
  CHECK(required_basic_indices(4,5) == std::vector<int>({3,4}));
  CHECK(required_source_indices(4,5) == std::vector<int>({2,3,4}));
  CHECK(required_basic_indices(INT32_MAX-1,INT32_MAX) == std::vector<int>({INT32_MAX-2,INT32_MAX-1}));
  rejects([&] { required_basic_indices(0,0); });
  rejects([&] { required_source_indices(5,5); });
  m.good = false;
  rejects([&] { temporal_motion(m,std::numeric_limits<float>::infinity(),std::nullopt,4); });
}
} // namespace
int main() {
  try {
    peaks();
    scalar_grouping();
    refinements();
    combination();
    temporal();
    std::cout << "Depan estimate motion checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
