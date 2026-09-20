#include "core/interpolation/rate.hpp"

#include <iostream>
#include <string>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("frame-rate assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
void examples() {
  const FrameRatePlan doubled(10, 24, 1, 0, 1);
  CHECK(doubled.output_num() == 48 && doubled.output_den() == 1 && doubled.output_frames() == 20);
  CHECK(doubled.ratio_num() == 1 && doubled.ratio_den() == 2);
  const auto a = doubled.position(3), endpoint = doubled.position(2);
  CHECK(a.left == 1 && a.right == 2 && a.time == 128 && !a.endpoint);
  CHECK(endpoint.left == 1 && endpoint.time == 0 && endpoint.endpoint == 1);
  const auto b = FrameRatePlan(10, 24, 1, 0, 1, 2).position(3);
  CHECK(b.left == 1 && b.right == 3 && b.time == 64 && !b.endpoint);
  const FrameRatePlan defaults(10, 24, 1);
  CHECK(defaults.output_num() == 25 && defaults.output_den() == 1 && defaults.output_frames() == 10);
  CHECK(FrameRatePlan(10, 24, 1, 60, 0).output_num() == 48);
  const FrameRatePlan rate30(10, 24, 1, 30, 1);
  CHECK(rate30.ratio_num() == 4 && rate30.ratio_den() == 5 && rate30.output_frames() == 12);
  const auto c = rate30.position(1);
  CHECK(c.left == 0 && c.right == 1 && c.time == 205);
  CHECK(FrameRatePlan(10, 24, 1, 30, 1, 2).position(1).time == 102);
  const FrameRatePlan thousand(2, 1, 1, 1000, 1);
  CHECK(thousand.output_frames() == 2000);
  const auto d = thousand.position(999), e = thousand.position(1999);
  CHECK(d.left == 0 && d.right == 1 && d.time == 256 && d.endpoint == 1);
  CHECK(e.left == 1 && e.right == 2 && e.time == 256 && e.endpoint == 1);
  const auto distant = FrameRatePlan(2, 1, 1, 1000, 1, INT32_MAX).position(1999);
  CHECK(distant.right == std::int64_t(INT32_MAX) + 1 && distant.time == 0 && distant.endpoint == 1);
}
void reduction_and_exact_positions() {
  const FrameRatePlan reduced(3, INT64_MAX, INT64_MAX, 0, 1);
  CHECK(reduced.output_num() == 2 && reduced.output_den() == 1 && reduced.output_frames() == 6);
  const FrameRatePlan cancellation(7, INT64_MAX, INT64_MAX - 1, INT64_MAX, INT64_MAX - 1);
  CHECK(cancellation.ratio_num() == 1 && cancellation.ratio_den() == 1 && cancellation.output_frames() == 7);
  const FrameRatePlan cancel_two(4, INT64_MAX, 2, 0, 1);
  CHECK(cancel_two.output_num() == INT64_MAX && cancel_two.output_den() == 1);
  // The two rates convert to the same double. Exact floor still selects the
  // preceding left frame, while the prescribed floating time reaches 256.
  const FrameRatePlan almost_one(2, INT64_MAX - 1, INT64_MAX, 1, 1);
  const auto a = almost_one.position(1);
  CHECK(a.left == 0 && a.right == 1 && a.time == 256 && a.endpoint == 1);
  CHECK(FrameRatePlan(2, INT64_MAX - 1, INT64_MAX, 1, 1, 2).position(1).time == 128);
  const FrameRatePlan huge(INT32_MAX, INT64_MAX - 1, INT64_MAX, 1, 1);
  CHECK(huge.output_frames() == INT32_MAX);
  CHECK(huge.position(INT32_MAX - 1).left == INT32_MAX - 2);
  // Exhaust small rates against a separate direct integer formula. Products
  // here fit int64, unlike the plan's admitted general domain.
  for (int frames : {1, 2, 7, 31})
    for (int i = 1; i <= 9; ++i)
      for (int j = 1; j <= 9; ++j)
        for (int p = 1; p <= 7; ++p)
          for (int q = 1; q <= 7; ++q) {
            const auto count = std::int64_t(frames) * j * p / (i * q);
            if (!count)
              continue;
            const FrameRatePlan plan(frames, i, j, p, q);
            CHECK(plan.output_frames() == count);
            for (std::int64_t n = 0; n < count; ++n)
              CHECK(plan.position(n).left == n * i * q / (j * p));
          }
}
void failures() {
  rejects([] { FrameRatePlan(0, 24, 1); });
  rejects([] { FrameRatePlan(std::int64_t(INT32_MAX) + 1, 24, 1); });
  rejects([] { FrameRatePlan(2, 0, 1); });
  rejects([] { FrameRatePlan(2, 1, -1); });
  rejects([] { FrameRatePlan(2, 1, 1, -1, 0); });
  rejects([] { FrameRatePlan(2, 1, 1, 0, -1); });
  rejects([] { FrameRatePlan(2, 1, 1, 1, 1, 0); });
  rejects([] { FrameRatePlan(1, 24, 1, 1, 1); });
  rejects([] { FrameRatePlan(INT32_MAX, 1, 1, 2, 1); });
  rejects([] { FrameRatePlan(2, INT64_MAX, 1, 0, 1); });
  rejects([] { FrameRatePlan(2, INT64_MAX, 1, 1, INT64_MAX); });
  rejects([] { FrameRatePlan(2, 1, INT64_MAX, INT64_MAX, 1); });
  const FrameRatePlan plan(2, 1, 1, 1, 1);
  rejects([&] { plan.position(-1); });
  rejects([&] { plan.position(2); });
}
} // namespace
int main() {
  try {
    examples();
    reduction_and_exact_positions();
    failures();
    std::cout << "Frame-rate mapping specifications passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
