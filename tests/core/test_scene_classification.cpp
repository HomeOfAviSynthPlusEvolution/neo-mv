#include "core/motion/scene_classification.hpp"

#include <iostream>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("scene assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E, class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const E&) {
    caught = true;
  }
  CHECK(caught);
}
using namespace neo_mv;
AnalysisField field() {
  AnalysisMetadata m;
  m.width = m.height = m.real_width = m.real_height = 16;
  m.pad_x = m.pad_y = 4;
  m.pel = 2;
  m.levels = 1;
  m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = 8;
  m.blocks_x = m.blocks_y = 2;
  m.bits = 8;
  return {m, FieldState::complete, {2, 2, {{{0, 0}, 400}, {{0, 0}, 401}, {{0, 0}, 900}, {{0, 0}, 0}}}};
}
void strict_thresholds() {
  auto f = field();
  const SceneClassifier classify(scene_descriptor(f.metadata), 400, 50);
  CHECK(classify.thresholds.error == 400 && classify.thresholds.count == 2);
  CHECK(classify(f) == 0);
  f.grid.values[0].error = 401;
  CHECK(classify(f) == 1);
  auto fractional = scene_thresholds(scene_descriptor(f.metadata), 400, 51);
  CHECK(fractional.count == static_cast<float>(2.04));
  auto d = scene_descriptor(f.metadata);
  d.block_width = 16;
  d.chroma = true;
  d.ratio_x = d.ratio_y = 2;
  d.bits = 10;
  CHECK(scene_thresholds(d, 400, 50).error == 4814);
  d = scene_descriptor(f.metadata);
  d.bits = 32;
  CHECK(scene_thresholds(d, 400, 50).error == 102800);
  const SceneClassifier zero(scene_descriptor(f.metadata), 400, 0), all(scene_descriptor(f.metadata), 400, 100);
  CHECK(zero(f) == 1 && all(f) == 0);
  for (auto& v : f.grid.values)
    v.error = 400;
  CHECK(zero(f) == 0);
  // Conversion to binary32 precedes percentage range validation.
  CHECK(scene_thresholds(d, 400, std::nextafter(100.0, 101.0)).count == 4.0f);
  CHECK(scene_thresholds(d, 400, -std::numeric_limits<double>::denorm_min()).count == 0.0f);
  d.blocks_x = 16777217;
  d.blocks_y = 1;
  CHECK(scene_thresholds(d, 400, 100).count == 16777216.0f); // count is not stored as an integer
}
void availability_and_descriptors() {
  auto f = field();
  const SceneClassifier classify(scene_descriptor(f.metadata), 400, 100);
  f.state = FieldState::metadata_only;
  f.grid.values.clear();
  CHECK(classify(f) == 1);
  f.metadata.bits = 10;
  rejects<std::invalid_argument>([&] { classify(f); });
  f.state = FieldState::invalid_metadata;
  CHECK(classify(f) == 1); // neither descriptor matching nor entries are read
  f = field();
  f.grid.values[0].error = -1;
  rejects<std::invalid_argument>([&] { classify(f); });
  f = field();
  f.grid.values.pop_back();
  rejects<std::invalid_argument>([&] { classify(f); });
  f = field();
  f.grid.values[0].vector = {INT32_MAX, 0};
  rejects<std::invalid_argument>([&] { classify(f); });
  f = field();
  f.metadata.delta = INT32_MIN;
  f.metadata.levels = 5;
  CHECK(classify(f) == 0); // unrelated descriptor scalars do not cause mismatch
}
void invalid_parameters() {
  const auto d = scene_descriptor(field().metadata);
  rejects<std::invalid_argument>([&] { scene_thresholds(d, -1, 50); });
  rejects<std::invalid_argument>([&] { scene_thresholds(d, 16321, 50); });
  rejects<std::invalid_argument>([&] { scene_thresholds(d, 400, -0.01); });
  rejects<std::invalid_argument>([&] { scene_thresholds(d, 400, 100.01); });
  rejects<std::invalid_argument>([&] { scene_thresholds(d, 400, std::numeric_limits<double>::infinity()); });
  rejects<std::invalid_argument>([&] { scene_thresholds(d, 400, std::numeric_limits<double>::quiet_NaN()); });
  rejects<std::invalid_argument>([&] { scene_thresholds(d, 400, std::numeric_limits<double>::max()); });
  auto huge = d;
  huge.block_width = huge.block_height = INT32_MAX;
  huge.bits = 32;
  rejects<std::overflow_error>([&] { scene_thresholds(huge, 16320, 50); });
}
} // namespace

int main() {
  try {
    strict_thresholds();
    availability_and_descriptors();
    invalid_parameters();
    std::cout << "Scalar scene classification checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
