#include "core/render/compensation.hpp"
#include "core/motion/composition.hpp"

#include <iostream>

namespace {
using namespace neo_mv;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("compensation assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class F>
bool rejects(F call) {
  try {
    call();
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}
void controls() {
  const SuperPlan<std::uint8_t> super({16, 16, 8, 8, 0, 0, 4, 4, 2, 2, 2, true}, 8);
  auto m = super_analysis_metadata(super, -1, false);
  const CompensationRule full(m, 100), half(m, 100, 50), zero(m, 100, 0);
  CHECK(CompensationRule(m, 2147483648LL).threshold() == 2147483648LL);
  CHECK(full.time_coefficient() == 256 && half.time_coefficient() == 128 && zero.time_coefficient() == 0);
  CHECK(full.select({-3, 2}, 99, 0).reference);
  CHECK(!full.select({-3, 2}, 100, 0).reference);
  CHECK(half.select({-3, 2}, 99, 0).displacement.x == -1);
  CHECK(zero.select({-3, 2}, 99, 0).reference && zero.select({-3, 2}, 99, 0).displacement.x == 0);
  CHECK(!CompensationRule(m, 0).select({0, 0}, 0, 0).reference);
  CHECK(CompensationRule(m, 100, std::nextafter(100.0, 0.0)).time_coefficient() == 255);
  CHECK(rejects([&] { CompensationRule bad(m, -1); }));
  for (double time : {-0.01, 100.01, double(INFINITY), double(NAN)})
    CHECK(rejects([&] { CompensationRule bad(m, 100, time); }));
  CHECK(rejects([&] { full.select({0, 0}, -1, 0); }));
  CHECK(rejects([&] { full.select({0, 0}, 0, 1); }));
  const CompensationRule parity(m, 100, 50, true);
  CHECK(parity.field_shift(0, true, false) == 1);
  CHECK(parity.field_shift(0, false, true) == -1);
  CHECK(parity.field_shift(0, false, false) == 0);
  CHECK(parity.reference_displacement({0, -3}, 1).y == 0); // shift after truncation
  CHECK(rejects([&] { parity.field_shift(0); }));
  CHECK(CompensationRule(m, 100, 50, true, false).field_shift(0) == -1);
  CHECK(CompensationRule(m, 100, 50, true, false).field_shift(1) == 1);
  CHECK(CompensationRule(m, 100, 50, true, true).field_shift(0) == 1);
  CHECK(full.field_shift(0) == 0);
  m.delta = 2;
  CHECK(CompensationRule(m, 100, 50, true).field_shift(0) == 0);
  m.pel = 1;
  CHECK(rejects([&] { CompensationRule bad(m, 100, 100, true); }));
  m.bits = 10;
  CHECK(CompensationRule(m, 400).threshold() == 1605);
}
void samples_and_admission() {
  for (int pad : {3, 4}) {
    const SuperPlan<std::uint8_t> super({16, 16, 8, 8, 0, 0, pad, pad, 2, 2, 2, true}, 8);
    const auto m = super_analysis_metadata(super, 1, false);
    const auto g = super_sampling_geometry(super)[0];
    const CompensationRule full(m, 100, 100, true), zero(m, 100, 0);
    for (int by = 0; by < 2; ++by)
      for (int bx = 0; bx < 2; ++bx)
        for (int plane = 0; plane < 3; ++plane) {
          const auto pg = render_phase_geometry(g, plane);
          const auto b = analysis_block(m, bx, by);
          CHECK(rejects([&] { full.admit(pg, b, analysis_domain(m, b)); }) == (pad == 3 && plane != 0));
          zero.admit(pg, b, analysis_domain(m, b));
        }
  }
  const SuperPlan<std::uint8_t> super({8, 8, 8, 8, 0, 0, 4, 4}, 8);
  const auto m = super_analysis_metadata(super, 1);
  const auto g = render_phase_geometry(super_sampling_geometry(super)[0], 0);
  const CompensationRule full(m, 100), zero(m, 100, 0), fields(m, 100, 100, true);
  std::array<super_detail::PlaneBuffer<std::uint8_t>, 2> input{super_detail::PlaneBuffer<std::uint8_t>(8, 8),
                                                               super_detail::PlaneBuffer<std::uint8_t>(8, 8)};
  for (int k = 0; k < 2; ++k)
    for (int y = 0; y < 8; ++y)
      for (int x = 0; x < 8; ++x)
        input[k].view().row(y)[x] = k == 0 ? 10 : 80;
  const SuperPyramid<std::uint8_t> current(super, {input[0].view(), {}, {}}),
      reference(super, {input[1].view(), {}, {}});
  SubpixelPhases<std::uint8_t> c{2}, r{2};
  for (int ay = 0; ay < 2; ++ay)
    for (int ax = 0; ax < 2; ++ax) {
      c.planes[ay * 2 + ax] = current.phase(0, 0, ax, ay);
      r.planes[ay * 2 + ax] = reference.phase(0, 0, ax, ay);
    }
  super_detail::PlaneBuffer<std::uint8_t> output(8, 8);
  for (const auto* rule : {&full, &zero})
    for (int sad : {99, 100}) {
      sample_compensated_block(*rule, g, {0, 0, 8, 8}, {{0, 0}, sad}, 0, c, r, output.view(), 8);
      for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
          CHECK(output.view().row(y)[x] == (sad < 100 ? 80 : 10));
    }
  CHECK(
      rejects([&] { sample_compensated_block(fields, g, {0, 0, 8, 8}, {{0, -8}, 100}, -1, c, r, output.view(), 8); }));
  CHECK(output.view().row(0)[0] == 10); // unsafe reference fails even when current would be selected
  auto bad_reference = r;
  bad_reference.planes[3] = {};
  CHECK(rejects(
      [&] { sample_compensated_block(full, g, {0, 0, 8, 8}, {{0, 0}, 100}, 0, c, bad_reference, output.view(), 8); }));
}
} // namespace
int main() {
  try {
    controls();
    samples_and_admission();
    std::cout << "Scalar compensation selection and sampling checks passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
