#include "core/interpolation/input.hpp"

#include <iostream>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("interpolation input assertion at " + std::to_string(line));
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
struct Fixture {
  SuperPlan<std::uint8_t> super{{8, 8, 8, 8, 0, 0, 4, 4, 1, 1, 1}, 8};
  RenderVideo video{8, 8, 8, false, 1, 1, 5};
  std::array<AnalysisMetadata, 2> metadata{super_analysis_metadata(super, 1), super_analysis_metadata(super, -1)};
  InterpolationInputPlan<std::uint8_t> plan() const { return {video, super, 5, metadata, {5, 5}}; }
  AnalysisField field(int direction) const {
    const auto m = metadata[direction];
    return {m,
            FieldState::complete,
            {m.blocks_x, m.blocks_y, std::vector<MotionTriple>(std::size_t(m.blocks_x) * m.blocks_y, {{0, 0}, 0})}};
  }
};

void creation() {
  Fixture f;
  CHECK(f.plan().distance() == 1);
  CHECK(f.plan().plane_count() == 1);
  f.metadata[1].levels = 7;
  CHECK(f.plan().metadata(1).levels == 7);
  f.metadata[0].delta = 2;
  f.metadata[1].delta = -2;
  CHECK(f.plan().distance() == 2);
  f.metadata[1].delta = -1;
  rejects([&] { f.plan(); });
  f = Fixture{};
  std::swap(f.metadata[0], f.metadata[1]);
  rejects([&] { f.plan(); });
  f = Fixture{};
  f.metadata[0].delta = f.metadata[1].delta = 0;
  rejects([&] { f.plan(); });
  f = Fixture{};
  f.metadata[1].bits = 10;
  rejects([&] { f.plan(); });
  f = Fixture{};
  f.metadata[1].levels = 0;
  rejects([&] { f.plan(); });
  f = Fixture{};
  rejects([&] { InterpolationInputPlan<std::uint8_t>(f.video, f.super, 4, f.metadata, {5, 5}); });
  rejects([&] { InterpolationInputPlan<std::uint8_t>(f.video, f.super, 5, f.metadata, {5, 4}); });
  for (double threshold : {-1.0, 101.0, std::numeric_limits<double>::infinity()})
    rejects([&] { InterpolationInputPlan<std::uint8_t>(f.video, f.super, 5, f.metadata, {5, 5}, 400, threshold); });
  rejects([&] { InterpolationInputPlan<std::uint8_t>(f.video, f.super, 5, f.metadata, {5, 5}, 16321); });
  f.video.frames = std::int64_t(INT32_MAX) + 1;
  rejects([&] {
    InterpolationInputPlan<std::uint8_t>(f.video, f.super, f.video.frames, f.metadata,
                                         {f.video.frames, f.video.frames});
  });
  f = Fixture{};
  f.metadata[0].delta = INT32_MAX;
  f.metadata[1].delta = -INT32_MAX;
  CHECK(f.plan().distance() == INT32_MAX); // A short clip does not invalidate the descriptor pair.
}

void fields() {
  Fixture f;
  const auto plan = f.plan();
  auto b = f.field(0), forward = f.field(1);
  CHECK(plan.main_eligible(b, forward));
  b.metadata.levels = 9;
  CHECK(plan.eligible(b, 0));
  b.state = FieldState::metadata_only;
  b.grid = {0, 0, {}};
  CHECK(!plan.eligible(b, 0));
  b.metadata.delta = 2;
  rejects([&] { plan.eligible(b, 0); }); // Metadata changes cannot hide behind missing arrays.
  b = f.field(0);
  b.state = FieldState::invalid_metadata;
  CHECK(!plan.eligible(b, 0));
  forward.grid.values.back().error = -1;
  rejects([&] { plan.main_eligible(b, forward); }); // Both directions must be validated.
  forward = f.field(1);
  forward.grid.values.back().vector.x = INT32_MAX;
  rejects([&] { plan.main_eligible(b, forward); });
  forward = f.field(1);
  forward.grid.values.clear();
  rejects([&] { plan.main_eligible(b, forward); });
  b = f.field(0);
  b.grid.values[0].error = 401;
  CHECK(!plan.eligible(b, 0));
  f.metadata[0].bits = f.metadata[1].bits = 16;
  b = f.field(0);
  b.grid.values[0].error = 401;
  CHECK(f.plan().eligible(b, 0)); // Analysis depth, not the GRAY8 render depth, scales scene SAD.
}

void selection() {
  Fixture f;
  const auto plan = f.plan();
  std::vector<std::pair<int, std::int64_t>> reads;
  const auto good = [&](int direction, std::int64_t n) {
    reads.emplace_back(direction, n);
    return f.field(direction);
  };
  CHECK(plan.select(4, 5, good).mode == PairMode::fallback);
  CHECK(plan.select(-1, 0, good).mode == PairMode::fallback);
  CHECK(reads.empty());
  CHECK(plan.select(3, 4, good).mode == PairMode::extra);
  const std::vector<std::pair<int, std::int64_t>> expected{{0, 3}, {1, 4}, {0, 4}, {1, 3}};
  CHECK(reads == expected); // bw[4]'s nominal target 5 is irrelevant to extra eligibility.
  reads.clear();
  CHECK(plan.select(2, 3, good, false).mode == PairMode::basic);
  CHECK(reads.size() == 2);
  const auto unavailable = [&](int direction, std::int64_t n) {
    reads.emplace_back(direction, n);
    auto field = f.field(direction);
    if (direction == 0 && n == 2)
      field.state = FieldState::metadata_only;
    if (direction == 0 && n == 3)
      field.grid.values[0].error = -1; // An unused extra must not be read.
    return field;
  };
  reads.clear();
  CHECK(plan.select(2, 3, unavailable).mode == PairMode::fallback);
  CHECK(reads.size() == 2);
  const auto bad_second = [&](int direction, std::int64_t) {
    auto field = f.field(direction);
    if (direction == 0)
      field.state = FieldState::metadata_only;
    else
      field.grid.values[0].error = -1;
    return field;
  };
  rejects([&] { plan.select(2, 3, bad_second); });
  const auto extra = [&](int direction, std::int64_t n) {
    auto field = f.field(direction);
    if (direction == 0 && n == 3)
      field.state = FieldState::metadata_only;
    if (direction == 1 && n == 2)
      field.grid.values[0].error = -1;
    return field;
  };
  rejects([&] { plan.select(2, 3, extra); });
  CHECK(plan.select(2, 3, extra, false).mode == PairMode::basic);
  rejects([&] { plan.select(1, 3, good); });
  rejects([&] { plan.validate_image({}); });
}
} // namespace

int main() {
  try {
    creation();
    fields();
    selection();
    std::cout << "Interpolation input checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
