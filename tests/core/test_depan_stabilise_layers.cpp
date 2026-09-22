#include "core/depan/stabilise_layers.hpp"
#include "core/depan/sampling.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/depan.hpp"
#endif
#include <iostream>
#include <string>
namespace {
using namespace neo_mv;
using namespace neo_mv::depan;
using namespace neo_mv::depan::stabilise;
void check(bool v, int line) {
  if (!v)
    throw std::runtime_error("stabilise layers assertion " + std::to_string(line));
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
void selection() {
  Parameters p;
  p.prev = 2;
  p.next = 3;
  auto c = normalize(p, 48, 48, 24, 1);
  std::vector<int> reads;
  auto l = select_layers({{}, 0}, 2, 8, p, c, [&](int k) {
    reads.push_back(k);
    return Motion{1, 0, 0, 1, k != 5};
  });
  CHECK(l.previous->frame == 1 && l.previous->map.tx == 2);
  CHECK(l.next->frame == 4 && l.next->map.tx == -2);
  CHECK(reads == std::vector<int>({2, 1, 3, 4, 5}));
  rejects([&] {
    select_layers({{}, 0}, 2, 8, p, c, [](int k) {
      if (k == 5)
        throw std::invalid_argument("malformed");
      return Motion{1, 0, 0, 1, k != 3};
    });
  });
  l = select_layers({{}, 2}, 2, 8, p, c, [](int) { return Motion{0, 0, 0, 1, true}; });
  CHECK(l.previous->frame == 2 && l.previous->map.tx == 0);
  l = select_layers({{}, 7}, 7, 8, p, c, [](int) -> Motion { throw std::runtime_error("unexpected read"); });
  CHECK(l.previous->frame == 7 && l.next->frame == 7);
  CHECK(diagnostic(4, {{2, 0, 1, 0, 0, 1}, 0}, c) == "frame=4 base =0 dx=2.00 dy=0.00 rot=-0.000 zoom=1.00000");
}
template <class T, class Plan>
void layers(int bits) {
  constexpr int w = 4, h = 4, stride = 7;
  std::vector<T> input(stride * h, 10), output(stride * h, 77);
  auto src = checked_plane<const T>(input.data(), w, h, stride * sizeof(T), input.size() * sizeof(T));
  auto dst = checked_plane<T>(output.data(), w, h, stride * sizeof(T), output.size() * sizeof(T));
  Plan(w, h, bits, 0, 0, 0, 0, {1, 0, 1, 0, 0, 1}).render(src, dst);
  for (int y = 0; y < h; ++y)
    CHECK(dst.row(y)[3] == 0 && dst.row(y)[0] == 10);
  std::fill(input.begin(), input.end(), 20);
  Plan(w, h, bits, 0, 0, 0, 0, {-1, 0, 1, 0, 0, 1}).render(src, dst, true);
  CHECK(dst.row(1)[0] == 10 && dst.row(1)[1] == 20);
  std::fill(input.begin(), input.end(), 30);
  Plan(w, h, bits, 0, 0, 0, 0, {1, 0, 1, 0, 0, 1}).render(src, dst, true);
  CHECK(dst.row(1)[0] == 30 && dst.row(1)[3] == 20);
  std::fill(input.begin(), input.end(), 0);
  Plan(w, h, bits, 1, 0, 0, 0, {}).render(src, dst, true);
  CHECK(dst.row(1)[0] == 0 && dst.row(1)[3] == 20 && dst.row(3)[3] == 0);
  for (int y = 0; y < h; ++y)
    for (int x = w; x < stride; ++x)
      CHECK(output[y * stride + x] == 77);
  // Preserve must exactly mark symbolic border branches for all sampling
  // classes/modes, including real border-colored source pixels.
  for (int mode = 0; mode < 3; ++mode)
    for (Transform map :
         {Transform{-.5f, -.5f, 1, 0, 0, 1}, Transform{2, 1, .9f, 0, 0, .9f}, Transform{1, 1, 1, .2f, -.2f, 1}}) {
      Plan plan(w, h, bits, mode, 0, 0, 0, map);
      SamplingPlan oracle(w, h, bits, mode, 0, 0, 0, map);
      std::fill(output.begin(), output.end(), 77);
      plan.render(src, dst, true);
      for (int y = 0; y < h; ++y)
        oracle.row_coordinates(y, [&](int x, SamplingCoordinates q) {
          CHECK(dst.row(y)[x] == (oracle.evaluate_result(src, q) ? 0 : 77));
        });
    }
}
} // namespace
int main() {
  try {
    selection();
    layers<std::uint8_t, SamplingPlan>(8);
    layers<std::uint16_t, SamplingPlan>(12);
#if NEO_MV_TEST_HIGHWAY
    layers<std::uint8_t, HighwaySamplingPlan>(8);
    layers<std::uint16_t, HighwaySamplingPlan>(12);
#endif
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
