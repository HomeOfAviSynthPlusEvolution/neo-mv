#include "core/motion/search.hpp"

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("motion search assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class Exception, class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const Exception&) {
    caught = true;
  }
  CHECK(caught);
}
using namespace neo_mv;
using Point = std::pair<int, int>;
Point point(MotionVector v) {
  return {v.x, v.y};
}
struct Errors {
  std::map<Point, std::int64_t> values;
  std::vector<Point> visits;
  std::int64_t fallback = 9;
  BlockError operator()(MotionVector v) {
    visits.push_back(point(v));
    const auto found = values.find(point(v));
    const auto e = found == values.end() ? fallback : found->second;
    return {e, 0, e};
  }
};
SearchParams params(int type, int range) {
  return {{0, 0}, 0, 0, {-100, -100, 101, 101}, type, range, {}};
}
void result(SearchResult r, int x, int y, std::int64_t cost, std::int64_t raw) {
  CHECK(r.vector.x == x && r.vector.y == y && r.cost == cost && r.raw == raw);
}

void specification_examples() {
  const SearchResult initial{{0, 0}, 9, 9};
  Errors axial{{{{-1, 0}, 4}, {{1, 0}, 4}, {{0, -1}, 4}, {{0, 1}, 4}}, {}};
  result(refine_motion(initial, params(1, 1), axial), 0, -1, 4, 4);
  result(refine_motion({{0, 0}, 4, 9}, params(1, 1), axial), 0, 0, 4, 9);
  axial.values[{0, -1}] = 0;
  result(refine_motion(initial, params(4, 1), axial), -1, 0, 4, 4);
  result(refine_motion(initial, params(5, 1), axial), 0, -1, 0, 0);
  Errors hex{{{{2, 0}, 3}, {{2, -1}, 1}}, {}};
  result(refine_motion(initial, params(2, 2), hex), 2, -1, 1, 1);
  auto weighted = params(4, 2);
  weighted.lambda = 256;
  Errors distance{{{{1, 0}, 5}, {{2, 0}, 5}}, {}};
  result(refine_motion({{0, 0}, 8, 9}, weighted, distance), 1, 0, 6, 5);
  Errors log{{{{1, 0}, 4}, {{1, 1}, 2}}, {}};
  result(refine_motion(initial, params(0, 1), log), 1, 1, 2, 2);
  Errors multi{{{{-4, 2}, 0}}, {}};
  result(refine_motion(initial, params(3, 1), multi), -4, 2, 0, 0);
}

void ordered_lists() {
  const std::vector<Point> q1{{0, -1}, {0, 1}, {-1, 0}, {1, 0}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  Errors flat;
  refine_motion({{0, 0}, 9, 9}, params(2, 1), flat);
  CHECK(flat.visits == q1); // no second ring for hex range 1
  flat.visits.clear();
  refine_motion({{0, 0}, 9, 9}, params(1, 2), flat);
  auto rings = q1;
  const std::vector<Point> q2{{-1, -2}, {-1, 2}, {0, -2}, {0, 2}, {1, -2},  {1, 2},  {-2, -1}, {2, -1},
                              {-2, 0},  {2, 0},  {-2, 1}, {2, 1}, {-2, -2}, {-2, 2}, {2, -2},  {2, 2}};
  rings.insert(rings.end(), q2.begin(), q2.end());
  CHECK(flat.visits == rings);
  flat.visits.clear();
  flat.values[{0, -1}] = 1;
  refine_motion({{0, 0}, 9, 9}, params(1, 2), flat);
  CHECK(flat.visits == rings); // improvements do not move the ring origin

  Errors group;
  auto multiscale = params(3, 1);
  multiscale.cross_center = MotionVector{10, 10};
  refine_motion({{0, 0}, 9, 9}, multiscale, group);
  const std::vector<Point> offsets{{-4, 2}, {-4, 1}, {-4, 0}, {-4, -1}, {-4, -2}, {4, -2},  {4, -1}, {4, 0},
                                   {4, 1},  {4, 2},  {2, 3},  {0, 4},   {-2, 3},  {-2, -3}, {0, -4}, {2, -3}};
  std::vector<Point> expected;
  for (auto v : offsets)
    expected.emplace_back(10 + v.first, 10 + v.second);
  expected.insert(expected.end(), q1.begin(), q1.end());
  CHECK(group.visits == expected); // hex begins at selected result, not cross centre
  Errors cross;
  refine_motion({{0, 0}, 9, 9}, params(3, 4), cross);
  const std::vector<Point> first{{-1, 0}, {1, 0}, {-3, 0}, {3, 0}, {0, -1}, {0, 1}, {0, -3}, {0, 3}};
  CHECK(std::equal(first.begin(), first.end(), cross.visits.begin()));
}

void recurrence_and_domains() {
  Errors hex{{{{2, 0}, 8}, {{4, 0}, 7}, {{6, 0}, 6}, {{6, 1}, 5}}, {}};
  result(refine_motion({{0, 0}, 9, 9}, params(2, 6), hex), 6, 1, 5, 5);
  CHECK(hex.visits.size() == 20); // six initial, two triples, eight final
  CHECK(hex.visits[6] == Point(3, 2) && hex.visits[7] == Point(4, 0) && hex.visits[8] == Point(3, -2));
  CHECK(hex.visits[9] == Point(5, 2) && hex.visits[10] == Point(6, 0) && hex.visits[11] == Point(5, -2));
  std::vector<Point> log_visits;
  auto parabola = [&](MotionVector v) {
    log_visits.push_back(point(v));
    const std::int64_t e = (v.x - 2) * (v.x - 2) + (v.y - 2) * (v.y - 2);
    return BlockError{e, 0, e};
  };
  result(refine_motion({{0, 0}, 8, 8}, params(0, 1), parabola), 2, 2, 0, 0);
  CHECK(log_visits.size() == 24);
  CHECK(log_visits[0] == Point(1, 0) && log_visits[1] == Point(-1, 0));
  CHECK(log_visits[8] == Point(2, 1) && log_visits[16] == Point(3, 2));

  auto restricted = params(1, 1);
  restricted.domain = {0, 0, 2, 2};
  Errors eligible;
  result(refine_motion({{-1, 0}, 0, 7}, restricted, eligible), -1, 0, 0, 7); // safe admitted seed may be outside Omega
  CHECK(eligible.visits == std::vector<Point>({{0, 0}, {0, 1}}));
  auto edge = params(4, 1);
  edge.domain = {INT32_MAX - 1, 0, std::int64_t(INT32_MAX) + 1, 1};
  Errors boundary;
  refine_motion({{INT32_MAX, 0}, 9, 9}, edge, boundary);
  CHECK(boundary.visits == std::vector<Point>({{INT32_MAX - 1, 0}}));
}

void exact_costs_and_errors() {
  CHECK(candidate_cost({1, 0}, {0, 0}, 256, 0, {5, 0, 5}) == 6);
  CHECK(candidate_cost({0, 0}, {0, 0}, 0, 1, {128, 128, 256}) == 256); // separate floors
  CHECK(candidate_cost({0, 0}, {0, 0}, 0, 256, {5, 3, 8}) == 16);
  CHECK(candidate_cost({1, 0}, {0, 0}, INT64_MAX, 0, {0, 0, 0}) == INT64_MAX / 256);
  constexpr std::uint64_t d = 4294967295ULL;
  CHECK(candidate_cost({INT32_MAX, INT32_MAX}, {INT32_MIN, INT32_MIN}, 64, 0, {0, 0, 0}) ==
        static_cast<std::int64_t>((d * d) / 2));
  CHECK(candidate_cost({INT32_MAX, INT32_MAX}, {INT32_MIN, INT32_MIN}, 0, 0, {INT64_MAX, 0, INT64_MAX}) == INT64_MAX);
  rejects<std::overflow_error>(
      [] { candidate_cost({INT32_MAX, INT32_MAX}, {INT32_MIN, INT32_MIN}, 65, 0, {0, 0, 0}); });
  rejects<std::overflow_error>([] { candidate_cost({0, 0}, {0, 0}, 0, 1, {INT64_MAX, 0, INT64_MAX}); });
  rejects<std::invalid_argument>([] { candidate_cost({0, 0}, {0, 0}, -1, 0, {0, 0, 0}); });
  rejects<std::invalid_argument>([] { candidate_cost({0, 0}, {0, 0}, 0, 257, {0, 0, 0}); });
  rejects<std::invalid_argument>([] { candidate_cost({0, 0}, {0, 0}, 0, 0, {-1, 0, -1}); });
  rejects<std::invalid_argument>([] { candidate_cost({0, 0}, {0, 0}, 0, 0, {1, 2, 4}); });
  Errors untouched;
  rejects<std::invalid_argument>([&] { refine_motion({{0, 0}, 9, 9}, params(6, 1), untouched); });
  rejects<std::invalid_argument>([&] { refine_motion({{0, 0}, 9, 9}, params(1, 0), untouched); });
  CHECK(untouched.visits.empty());
  rejects<std::runtime_error>([] {
    refine_motion({{0, 0}, 9, 9}, params(1, 1),
                  [](MotionVector) -> BlockError { throw std::runtime_error("sampling failed"); });
  });
}
} // namespace

int main() {
  try {
    specification_examples();
    ordered_lists();
    recurrence_and_domains();
    exact_costs_and_errors();
    std::cout << "Scalar motion refinement checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
