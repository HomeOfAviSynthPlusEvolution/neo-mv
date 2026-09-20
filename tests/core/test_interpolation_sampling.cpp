#include "core/interpolation/sampling.hpp"
#include "core/super/pyramid.hpp"

#include <iostream>
#include <limits>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("interpolation sampling assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& call, const std::string& message = {}) {
  bool caught = false;
  try {
    call();
  } catch (const std::invalid_argument& error) {
    caught = true;
    CHECK(message.empty() || std::string(error.what()).find(message) != std::string::npos);
  }
  CHECK(caught);
}
template <class T>
struct Image {
  RenderPhaseGeometry geometry;
  std::vector<super_detail::PlaneBuffer<T>> storage;
  Image(int width, int height, int pad, int pel, int base = 10, bool quarter = false)
      : geometry{pel, 1, 1, pad, pad, {}} {
    for (int phase = 0; phase < pel * pel; ++phase) {
      const int w = width + 2 * pad - (quarter && phase % pel == 3);
      const int h = height + 2 * pad - (quarter && phase / pel == 3);
      geometry.phases[phase] = {w, h};
      storage.emplace_back(w, h);
      auto view = storage.back().view();
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
          view.row(y)[x] = static_cast<T>(base + phase * 100 + y * 10 + x);
    }
  }
  SubpixelPhases<T> view() const {
    SubpixelPhases<T> result;
    result.pel = geometry.pel;
    for (std::size_t i = 0; i < storage.size(); ++i)
      result.planes[i] = storage[i].view();
    return result;
  }
  T at(int phase, int x, int y) const { return storage[phase].view().row(y)[x]; }
};
DenseFlowField field(int width, int height) {
  return {width, height, std::vector<std::int16_t>(std::size_t(width) * height),
          std::vector<std::int16_t>(std::size_t(width) * height)};
}
void direction_and_extra() {
  Image<std::uint16_t> left(2, 2, 2, 2), right(2, 2, 2, 2, 1000);
  auto B = field(2, 2), F = B, BB = B, FF = B;
  F.x[0] = -1;
  B.y[0] = -1;
  InterpolationSamplingPlan plan(left.geometry, right.geometry, 2, 2, 128, 16);
  const auto basic = plan.sample(left.view(), right.view(), B, F);
  CHECK(basic.size() == 4);
  CHECK(basic[0].A == left.at(1, 1, 2));
  CHECK(basic[0].C == right.at(2, 2, 1));
  CHECK(basic[0].A0 == left.at(0, 2, 2));
  CHECK(basic[0].C0 == right.at(0, 2, 2));
  CHECK(basic[3].A == left.at(0, 3, 3) && basic[3].C == right.at(0, 3, 3));
  FF.y[0] = -1;
  BB.x[0] = 3;
  const auto extra = plan.sample(left.view(), right.view(), B, F, &BB, &FF);
  CHECK(extra[0].A == basic[0].A && extra[0].C == basic[0].C);
  CHECK(extra[0].E == left.at(2, 2, 1));
  CHECK(extra[0].K == right.at(1, 2, 2));
  // Coordinates are already in this plane's units. Ratios are not applied again.
  auto chroma = left.geometry;
  chroma.ratio_x = chroma.ratio_y = 2;
  const auto mapped =
      InterpolationSamplingPlan(chroma, right.geometry, 2, 2, 128, 16).sample(left.view(), right.view(), B, F);
  CHECK(mapped[0].A == basic[0].A && mapped[0].C == basic[0].C);
}
void displacement_boundaries() {
  const std::int16_t values[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
  const int half[] = {-2, -2, -1, -1, 0, 0, 1, 1, 2};
  for (const int pel : {1, 2, 4}) {
    Image<std::uint16_t> image(1, 1, 5, pel);
    auto B = field(1, 1), F = B;
    for (const int time : {0, 128, 256})
      for (int i = 0; i < 9; ++i)
        for (const bool vertical : {false, true}) {
          F.x[0] = vertical ? 0 : values[i];
          F.y[0] = vertical ? values[i] : 0;
          const int displacement = time == 0 ? 0 : (time == 128 ? half[i] : values[i]);
          int integer = 0;
          while (integer * pel > displacement)
            --integer;
          while ((integer + 1) * pel <= displacement)
            ++integer;
          const int fraction = displacement - integer * pel;
          const auto sampled = InterpolationSamplingPlan(image.geometry, image.geometry, 1, 1, time, 16)
                                   .sample(image.view(), image.view(), B, F);
          CHECK(sampled[0].A == image.at(vertical ? fraction * pel : fraction, 5 + (vertical ? 0 : integer),
                                         5 + (vertical ? integer : 0)));
        }
  }
  Image<std::uint16_t> small(1, 1, 1, 2), enough(1, 1, 2, 2);
  auto B = field(1, 1), F = B;
  F.x[0] = -3;
  rejects([&] { InterpolationSamplingPlan(small.geometry, small.geometry, 1, 1, 256, 16).preflight(B, F); });
  const auto valid = InterpolationSamplingPlan(enough.geometry, enough.geometry, 1, 1, 256, 16)
                         .sample(enough.view(), enough.view(), B, F);
  CHECK(valid[0].A == enough.at(1, 0, 2));
}
void required_positions() {
  Image<std::uint16_t> image(2, 1, 1, 2);
  auto B = field(2, 1), F = B, BB = B, FF = B;
  InterpolationSamplingPlan plan(image.geometry, image.geometry, 2, 1, 0, 16);
  F.x[0] = FF.x[0] = INT16_MIN; // Zero-time left coordinates are still valid.
  B.x[1] = INT16_MAX;
  rejects([&] { plan.preflight(B, F); });
  B.x[1] = 0;
  BB.x[1] = INT16_MAX;
  plan.preflight(B, F);
  rejects([&] { plan.preflight(B, F, &BB, &FF); });
  rejects([&] { plan.preflight(B, F, &BB); });
  rejects([&] { plan.preflight(B, F, nullptr, &FF); });
  auto malformed = F;
  malformed.y.clear();
  rejects([&] { plan.preflight(B, malformed); });
  auto bad = image.view();
  bad.planes[3] = {};
  rejects([&] { plan.sample(bad, image.view(), B, F); });
  rejects([&] { InterpolationSamplingPlan(image.geometry, image.geometry, 2, 1, -1, 16); });
  rejects([&] { InterpolationSamplingPlan(image.geometry, image.geometry, 2, 1, 257, 16); });

  Image<std::uint16_t> quarter(1, 1, 1, 4, 10, true), external(1, 1, 1, 4);
  B = field(1, 1);
  F = B;
  F.x[0] = 7;
  rejects([&] { InterpolationSamplingPlan(quarter.geometry, quarter.geometry, 1, 1, 256, 16).preflight(B, F); });
  const auto valid = InterpolationSamplingPlan(external.geometry, external.geometry, 1, 1, 256, 16)
                         .sample(external.view(), external.view(), B, F);
  CHECK(valid[0].A == external.at(3, 2, 1));
}
void numeric_inputs_and_preflight() {
  Image<float> left(2, 1, 1, 1), right(2, 1, 1, 1, 1000);
  auto B = field(2, 1), F = B;
  InterpolationSamplingPlan plan(left.geometry, right.geometry, 2, 1, 0, 32);
  left.storage[0].view().row(1)[1] = std::numeric_limits<float>::quiet_NaN();
  B.x[1] = INT16_MAX;
  // The later coordinate must fail before reading the earlier NaN sample.
  rejects([&] { plan.sample(left.view(), right.view(), B, F); }, "logical phase domain");
  B.x[1] = 0;
  rejects([&] { plan.sample(left.view(), right.view(), B, F); }, "non-finite");
  left.storage[0].view().row(1)[1] = -10.0f;
  left.storage[0].view().row(0)[0] = std::numeric_limits<float>::infinity(); // Unused storage is not arithmetic input.
  CHECK(plan.sample(left.view(), right.view(), B, F)[0].A == -10.0f);
  Image<std::uint16_t> integer(1, 1, 1, 1);
  auto zero = field(1, 1);
  integer.storage[0].view().row(1)[1] = 1024;
  rejects([&] {
    InterpolationSamplingPlan(integer.geometry, integer.geometry, 1, 1, 0, 10)
        .sample(integer.view(), integer.view(), zero, zero);
  });
}
} // namespace
int main() {
  try {
    direction_and_extra();
    displacement_boundaries();
    required_positions();
    numeric_inputs_and_preflight();
    std::cout << "Interpolation sampling specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
