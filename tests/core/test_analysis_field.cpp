#include "core/motion/analysis_field.hpp"

#include <iostream>

namespace {
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("analysis field assertion failed at line " + std::to_string(line));
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
AnalysisMetadata metadata() {
  AnalysisMetadata m;
  m.width = m.height = m.real_width = m.real_height = 16;
  m.pad_x = m.pad_y = 4;
  m.pel = 2;
  m.levels = 1;
  m.chroma = true;
  m.ratio_x = m.ratio_y = 2;
  m.block_width = m.block_height = 8;
  m.blocks_x = m.blocks_y = 2;
  m.delta = 1;
  m.bits = 8;
  return m;
}
AnalysisField field() {
  return {metadata(), FieldState::complete, {2, 2, std::vector<MotionTriple>(4)}};
}
struct Properties {
  AnalysisProperties integers;
  std::map<std::string, std::uint64_t> other_types;
  std::vector<std::string> reads;
  IntegerPropertyView operator()(const std::string& key) {
    reads.push_back(key);
    const auto other = other_types.find(key);
    if (other != other_types.end())
      return {false, other->second, nullptr};
    const auto integer = integers.find(key);
    if (integer == integers.end())
      return {};
    return {true, integer->second.size(), integer->second.data()};
  }
  bool read_arrays() const {
    return std::any_of(reads.begin(), reads.end(), [](const auto& name) {
      return name.find("AnalysisVectors") != std::string::npos || name.find("AnalysisSAD") != std::string::npos;
    });
  }
};

void transport_and_ownership() {
  CHECK(pack_vector({-3, 2}) == 12884901885LL);
  CHECK(pack_vector({3, -2}) == -8589934589LL);
  for (int x : {INT32_MIN, -1, 0, 1, INT32_MAX})
    for (int y : {INT32_MIN, -1, 0, 1, INT32_MAX}) {
      const auto v = unpack_vector(pack_vector({x, y}));
      CHECK(v.x == x && v.y == y);
    }
  auto source = field();
  source.grid.values[0] = {{-3, 2}, 12884901885LL};
  Properties properties{encode_analysis_field(source, "custom_"), {}, {}};
  auto decoded = read_analysis_field(properties, true, "custom_");
  CHECK(decoded.state == FieldState::complete && decoded.grid.values.size() == 4);
  CHECK(decoded.grid.values[0].vector.x == -3 && decoded.grid.values[0].vector.y == 2);
  CHECK(decoded.grid.values[0].error == 12884901885LL);
  properties.integers.clear();
  CHECK(decoded.grid.values[0].error == 12884901885LL); // owns decoded values
  source.state = FieldState::metadata_only;
  const auto absent = encode_analysis_field(source);
  CHECK(absent.count("MVUtensilsAnalysisVectors") == 0 && absent.count("MVUtensilsAnalysisSAD") == 0);
}

void scalar_decoding() {
  Properties p{encode_analysis_field(field()), {}, {}};
  p.integers["MVUtensilsAnalysisWidth"] = {INT64_MAX, -1};
  p.integers["MVUtensilsAnalysisDeltaFrame"] = {INT64_MIN};
  p.integers["MVUtensilsAnalysisChroma"] = {INT64_MIN};
  auto decoded = read_analysis_field(p, false);
  CHECK(decoded.state == FieldState::metadata_only && !p.read_arrays());
  CHECK(decoded.metadata.width == INT32_MAX && decoded.metadata.delta == INT32_MIN && decoded.metadata.chroma);
  p.integers.erase("MVUtensilsAnalysisChroma");
  p.integers.erase("MVUtensilsAnalysisDeltaFrame");
  decoded = read_analysis_field(p, false);
  CHECK(!decoded.metadata.chroma && decoded.metadata.delta == 0 && decoded.state == FieldState::metadata_only);
  p.integers["MVUtensilsAnalysisPel"].clear();
  CHECK(read_analysis_field(p).state == FieldState::invalid_metadata && !p.read_arrays());
  p.integers["MVUtensilsAnalysisPel"] = {2};
  p.other_types["MVUtensilsAnalysisPel"] = 1;
  CHECK(read_analysis_field(p).state == FieldState::invalid_metadata && !p.read_arrays());
  auto generic = metadata();
  generic.block_width = 6;
  generic.block_height = 3;
  CHECK(valid_analysis_metadata(generic)); // public metadata is broader than Super's block-pair table
  generic.bits = 17;
  CHECK(!valid_analysis_metadata(generic));
}

void availability_and_malformed_arrays() {
  Properties p{encode_analysis_field(field()), {}, {}};
  CHECK(read_analysis_field(p).state == FieldState::complete); // all zero is measured data
  p.integers["MVUtensilsAnalysisSAD"].resize(3);
  p.other_types["MVUtensilsAnalysisVectors"] = 4;
  CHECK(read_analysis_field(p).state == FieldState::metadata_only); // count mismatch wins over wrong type
  p.integers["MVUtensilsAnalysisSAD"].resize(4);
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  CHECK(read_analysis_field(p, false).state == FieldState::metadata_only);
  p.other_types.clear();
  p.integers["MVUtensilsAnalysisVectors"][0] = pack_vector({23, 0});
  CHECK(read_analysis_field(p).state == FieldState::complete);
  p.integers["MVUtensilsAnalysisVectors"][0] = pack_vector({24, 0});
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  p.integers["MVUtensilsAnalysisVectors"][0] = pack_vector({-8, 0});
  CHECK(read_analysis_field(p).state == FieldState::complete);
  p.integers["MVUtensilsAnalysisVectors"][0] = pack_vector({-9, 0});
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  p.integers["MVUtensilsAnalysisVectors"][0] = 0;
  p.integers["MVUtensilsAnalysisSAD"][1] = -1;
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  // Decode must reject a malformed tail even when earlier SADs already imply a scene cut.
  p.integers["MVUtensilsAnalysisSAD"].assign(4, INT64_MAX);
  p.integers["MVUtensilsAnalysisSAD"].back() = -1;
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  p.integers["MVUtensilsAnalysisSAD"].back() = 0;
  p.integers["MVUtensilsAnalysisVectors"].back() = pack_vector({INT32_MAX, 0});
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  p.integers.erase("MVUtensilsAnalysisVectors");
  CHECK(read_analysis_field(p).state == FieldState::metadata_only);

  auto empty_bounds = field();
  empty_bounds.metadata.width = empty_bounds.metadata.real_width = 2;
  empty_bounds.metadata.pad_x = 0;
  empty_bounds.state = FieldState::metadata_only;
  p = {encode_analysis_field(empty_bounds), {}, {}};
  CHECK(read_analysis_field(p).state == FieldState::metadata_only);
  p.integers["MVUtensilsAnalysisVectors"] = std::vector<std::int64_t>(4);
  p.integers["MVUtensilsAnalysisSAD"] = std::vector<std::int64_t>(4);
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
}

void external_geometry_and_overflow() {
  auto scaled = field();
  auto& m = scaled.metadata;
  m.width = m.real_width = 64;
  m.height = m.real_height = 32;
  m.block_width = m.block_height = 16;
  m.overlap_x = m.overlap_y = 8;
  m.pad_x = m.pad_y = 8;
  m.blocks_x = 7;
  m.blocks_y = 3;
  m.levels = 5;
  scaled.grid = {7, 3, std::vector<MotionTriple>(21, {{-6, 4}, 28})};
  for (int by = 0; by < m.blocks_y; ++by)
    for (int bx = 0; bx < m.blocks_x; ++bx) {
      const auto domain = field_detail::bounds(m, bx, by);
      const auto index = std::size_t(by) * m.blocks_x + bx;
      scaled.grid.values[index].vector = {static_cast<std::int32_t>(index % 2 ? domain.left : domain.right - 1),
                                          static_cast<std::int32_t>(index % 2 ? domain.bottom - 1 : domain.top)};
    }
  Properties p{encode_analysis_field(scaled), {}, {}};
  const auto decoded = read_analysis_field(p);
  CHECK(decoded.state == FieldState::complete && decoded.grid.values.size() == 21 && decoded.metadata.width == 64);
  for (std::size_t i = 0; i < scaled.grid.values.size(); ++i)
    CHECK(decoded.grid.values[i].vector.x == scaled.grid.values[i].vector.x &&
          decoded.grid.values[i].vector.y == scaled.grid.values[i].vector.y);
  const auto outside = field_detail::bounds(m, 3, 1).right;
  p.integers["MVUtensilsAnalysisVectors"][10] = pack_vector({static_cast<std::int32_t>(outside), 0});
  rejects<std::invalid_argument>([&] { read_analysis_field(p); });
  scaled.grid.values[10].vector.x = static_cast<std::int32_t>(outside);
  rejects<std::invalid_argument>([&] { encode_analysis_field(scaled); });
  auto huge = metadata();
  huge.blocks_x = huge.block_width = INT32_MAX;
  huge.pel = 4;
  rejects<std::overflow_error>([&] { valid_analysis_metadata(huge); });
  auto inconsistent = field();
  inconsistent.grid.values.pop_back();
  rejects<std::invalid_argument>([&] { encode_analysis_field(inconsistent); });
}
} // namespace

int main() {
  try {
    transport_and_ownership();
    scalar_decoding();
    availability_and_malformed_arrays();
    external_geometry_and_overflow();
    std::cout << "Analysis field transport and validation checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
