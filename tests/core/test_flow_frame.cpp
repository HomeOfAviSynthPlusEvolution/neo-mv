#include "core/flow/frame.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/flow.hpp"
#endif

#include <iostream>

namespace {
using namespace neo_mv;
template <class T>
using TestKernels =
#if NEO_MV_TEST_HIGHWAY
    HighwayFlowKernels<T>;
#else
    ScalarFlowKernels<T>;
#endif
template <class T>
using Plan = FlowFramePlan<T, TestKernels<T>>;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("Flow frame assertion at " + std::to_string(line));
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
template <class T>
struct Image {
  SuperPlan<T> plan;
  std::vector<super_detail::PlaneBuffer<T>> storage;
  Image(SuperPlan<T> p, T base) : plan(std::move(p)) {
    const auto g = super_sampling_geometry(plan)[0];
    for (int k = 0; k < plan.geometry().plane_count; ++k)
      for (int a = 0; a < g.pel * g.pel; ++a) {
        const auto e = g.planes[k].reference[a];
        storage.emplace_back(e.width, e.height);
        auto v = storage.back().view();
        for (int y = 0; y < v.height(); ++y)
          for (int x = 0; x < v.width(); ++x)
            v.row(y)[x] = T(base + T(k * 16 + a));
      }
  }
  RenderImage<T> view() const {
    RenderImage<T> out{&plan, {}};
    std::size_t i = 0;
    for (int k = 0; k < plan.geometry().plane_count; ++k) {
      out.planes[k].pel = plan.params().pel;
      for (int a = 0; a < plan.params().pel * plan.params().pel; ++a)
        out.planes[k].planes[a] = storage[i++].view();
    }
    return out;
  }
  RenderPixels<T> pixels() const {
    RenderPixels<T> out{};
    const auto v = view();
    for (int k = 0; k < plan.geometry().plane_count; ++k) {
      const auto& p = plan.geometry().planes[k];
      out[k] = v.planes[k].planes[0].subplane(p.pad_x, p.pad_y, p.actual_width, p.actual_height);
    }
    return out;
  }
};
AnalysisField field(AnalysisMetadata m) {
  return {m,
          FieldState::complete,
          {m.blocks_x, m.blocks_y, std::vector<MotionTriple>(std::size_t(m.blocks_x) * m.blocks_y, {{0, 0}, 0})}};
}
template <class T>
void constant(const RenderOutput<T>& output, int k, T expected) {
  const auto v = output.at(k).view();
  for (int y = 0; y < v.height(); ++y)
    for (int x = 0; x < v.width(); ++x)
      CHECK(v.row(y)[x] == expected);
}
template <class T>
void simple(int bits) {
  SuperPlan<T> super({8, 8, 8, 8, 0, 0, 4, 4, 1, 1, 1}, bits);
  RenderVideo video{8, 8, bits, false, 1, 1, 2};
  auto m = super_analysis_metadata(super, 1);
  m.bits = 8; // Render precision and analysis precision are independent.
  Image<T> clip(super, T{5}), reference_image(super, T{80});
  auto ref = reference_image.view();
  auto f = field(m);
  Plan<T> plan(video, super, 2, m, 2);
  constant(plan.render(clip.pixels(), f, 0, &ref), 0, T{80});
  constant(plan.render(clip.pixels(), f, 1), 0, T{5});
  FlowParameters p;
  p.time = 0;
  constant(Plan<T>(video, super, 2, m, 2, p).render(clip.pixels(), f, 0, &ref), 0, T{80});
  f.state = FieldState::metadata_only;
  f.grid.values.clear();
  constant(plan.render(clip.pixels(), f, 0), 0, T{5});
  f = field(m);
  f.grid.values[0].error = INT64_MAX;
  constant(plan.render(clip.pixels(), f, 0), 0, T{5});
  f.grid.values[0].error = -1;
  rejects([&] { plan.render(clip.pixels(), f, 1); }); // Decode before temporal fallback.
  f = field(m);
  rejects([&] { plan.render(clip.pixels(), f, 0); });
  auto changed = f;
  changed.metadata.delta = -1;
  changed.state = FieldState::metadata_only;
  rejects([&] { plan.reference(changed, 0); });
  f.metadata.levels = 17;
  constant(plan.render(clip.pixels(), f, 0, &ref), 0, T{80});
  m.delta = 0;
  f = field(m);
  f.grid.values[0].vector.x = 1;
  const Plan<T> same(video, super, 2, m, 2);
  CHECK(same.reference(f, 1) == 1);
  constant(same.render(clip.pixels(), f, 1, &ref), 0, T{80});
  p.fields = true; // pel=1 explicitly accepted and inactive.
  const Plan<T> inactive(video, super, 2, m, 2, p);
  CHECK(!inactive.field_correction_active());
  constant(inactive.render(clip.pixels(), f, 1, &ref), 0, T{80});
}
struct ObservedKernels : TestKernels<std::uint8_t> {
  static inline int copies = 0;
  static void sample_preflighted(const TestKernels<std::uint8_t>::Sampling& plan, const DenseFlowField& field,
                                 const SubpixelPhases<std::uint8_t>& ref, span2d::Plane<std::uint8_t> out) {
    ++copies;
    TestKernels<std::uint8_t>::sample_preflighted(plan, field, ref, out);
  }
};
void admission_and_parity() {
  SuperGeometryParams g{16, 16, 8, 8, 0, 0, 3, 3, 2, 2, 2, true};
  SuperPlan<std::uint8_t> super(g, 8);
  RenderVideo video{16, 16, 8, true, 2, 2, 2};
  auto m = super_analysis_metadata(super, 1, false);
  Image<std::uint8_t> clip(super, 5), reference_image(super, 80);
  auto ref = reference_image.view();
  auto f = field(m);
  const FlowFramePlan<std::uint8_t, ObservedKernels> plan(video, super, 2, m, 2);
  constant(plan.render(clip.pixels(), f, 0, &ref), 2, std::uint8_t{112});
  f.grid.values[0].vector.x = -6; // Publicly valid but first chroma coordinate is -1.
  ObservedKernels::copies = 0;
  rejects([&] { plan.render(clip.pixels(), f, 0, &ref); });
  CHECK(ObservedKernels::copies == 0); // Y must not be copied before chroma preflight.
  constant(plan.render(clip.pixels(), f, 1), 0, std::uint8_t{5});
  FlowParameters p;
  p.time = 0;
  constant(Plan<std::uint8_t>(video, super, 2, m, 2, p).render(clip.pixels(), f, 0, &ref), 1, std::uint8_t{96});
  g.pad_x = g.pad_y = 4;
  SuperPlan<std::uint8_t> padded(g, 8);
  auto pm = super_analysis_metadata(padded, 1, false);
  Image<std::uint8_t> pc(padded, 5), pr(padded, 80);
  auto rv = pr.view();
  auto pf = field(pm);
  pf.grid.values[0].vector.x = -6;
  Plan<std::uint8_t>(video, padded, 2, pm, 2).render(pc.pixels(), pf, 0, &rv);
  pf = field(pm);
  p.fields = true;
  const Plan<std::uint8_t> parity(video, padded, 2, pm, 2, p);
  CHECK(parity.field_correction_active());
  rejects([&] { parity.render(pc.pixels(), pf, 0, &rv); }); // Required even at time=0.
  constant(parity.render(pc.pixels(), pf, 0, &rv, true, false), 0, std::uint8_t{80});
  constant(parity.render(pc.pixels(), pf, 1), 0, std::uint8_t{5});
  p.time = 50;
  p.tff = true;
  const Plan<std::uint8_t> override_parity(video, padded, 2, pm, 2, p);
  constant(override_parity.render(pc.pixels(), pf, 0, &rv, false, false), 0, std::uint8_t{82});
  constant(override_parity.render(pc.pixels(), pf, 0, &rv), 1, std::uint8_t{96});
  p.tff = false;
  constant(Plan<std::uint8_t>(video, padded, 2, pm, 2, p).render(pc.pixels(), pf, 0, &rv), 0, std::uint8_t{80});
  auto invalid = rv;
  invalid.planes[2].planes[3] = {};
  rejects([&] { override_parity.render(pc.pixels(), pf, 0, &invalid); });
  constant(override_parity.render(pc.pixels(), pf, 1, &invalid), 0, std::uint8_t{5});
  // Only consumed level-zero geometry matters.
  g.one_level = true;
  Image<std::uint8_t> shallow(SuperPlan<std::uint8_t>(g, 8), 80);
  override_parity.validate_reference(shallow.view());
  p.time = -1;
  rejects([&] { Plan<std::uint8_t>(video, padded, 2, pm, 2, p); });
  rejects([&] { Plan<std::uint8_t>(video, padded, 1, pm, 2); });
  rejects([&] { Plan<std::uint8_t>(video, padded, 2, pm, 1); });
}
void float_bits() {
  SuperPlan<float> super({8, 8, 8, 8, 0, 0, 4, 4, 1, 1, 1}, 32);
  Image<float> clip(super, 0), ref_image(super, 0);
  const auto m = super_analysis_metadata(super, 1);
  const Plan<float> plan({8, 8, 32, false, 1, 1, 2}, super, 2, m, 2);
  const std::uint32_t patterns[] = {0x7fa12345, 0x7fc54321, 0x80000000, 0, 0x7f800000, 0xff800000, 1, 0xbf800000};
  for (auto* image : {&clip, &ref_image}) {
    auto view = image->storage[0].view();
    for (int y = 0; y < 8; ++y)
      std::memcpy(view.row(y + 4).data() + 4, patterns, sizeof(patterns));
  }
  const auto ref = ref_image.view();
  for (int n : {0, 1}) {
    const auto output = plan.render(clip.pixels(), field(m), n, n == 0 ? &ref : nullptr);
    for (int y = 0; y < 8; ++y)
      CHECK(std::memcmp(output[0].view().row(y).data(), patterns, sizeof(patterns)) == 0);
  }
}
} // namespace
int main() {
  try {
    simple<std::uint8_t>(8);
    simple<std::uint16_t>(10);
    simple<std::uint16_t>(16);
    simple<float>(32);
    admission_and_parity();
    float_bits();
    std::cout << "Flow frame specifications passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
