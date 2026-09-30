#include "render_destination.hpp"
#include "core/render/frame.hpp"
#if NEO_MV_TEST_HIGHWAY
#include "highway/render_ops.hpp"
#endif
#include <iostream>

namespace {
using namespace neo_mv;
template <class T>
using TestKernels =
#if NEO_MV_TEST_HIGHWAY
    HighwayRenderKernels<T>;
#else
    ScalarRenderKernels<T>;
#endif
template <class T>
using TestCompensate = CompensateFramePlan<T, TestKernels<T>>;
template <class T>
using TestDegrain = DegrainFramePlan<T, TestKernels<T>>;
void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("render frame assertion at " + std::to_string(line));
}
#define CHECK(c) check((c), __LINE__)
template <class F>
void rejects(F&& f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}
template <class T>
struct Image {
  SuperPlan<T> plan;
  std::vector<super_detail::PlaneBuffer<T>> storage;
  Image(SuperPlan<T> p, T value) : plan(std::move(p)) {
    const auto g = super_sampling_geometry(plan)[0];
    for (int k = 0; k < plan.geometry().plane_count; ++k)
      for (int a = 0; a < g.pel * g.pel; ++a) {
        const auto e = g.planes[k].reference[a];
        storage.emplace_back(e.width, e.height);
        auto v = storage.back().view();
        for (int y = 0; y < v.height(); ++y)
          for (int x = 0; x < v.width(); ++x)
            v.row(y)[x] = value;
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
    auto v = view();
    for (int k = 0; k < plan.geometry().plane_count; ++k) {
      const auto& p = plan.geometry().planes[k];
      out[k] = v.planes[k].planes[0].subplane(p.pad_x, p.pad_y, p.actual_width, p.actual_height);
    }
    return out;
  }
};
AnalysisField field(AnalysisMetadata m, int sad = 0) {
  return {m,
          FieldState::complete,
          {m.blocks_x, m.blocks_y, std::vector<MotionTriple>(std::size_t(m.blocks_x) * m.blocks_y, {{0, 0}, sad})}};
}
template <class T>
void constant(const RenderOutput<T>& out, int plane, T value) {
  auto v = out.at(plane).view();
  for (int y = 0; y < v.height(); ++y)
    for (int x = 0; x < v.width(); ++x)
      CHECK(v.row(y)[x] == value);
}

template <class T>
void compensation(int bits) {
  const SuperPlan<T> super({8, 8, 8, 8, 0, 0, 4, 4}, bits);
  const RenderVideo video{8, 8, bits, false, 1, 1, 3};
  Image<T> clip(super, T(5)), current(super, T(10)), ref(super, T(80));
  auto m = super_analysis_metadata(super, 1);
  CompensateParameters p;
  p.thsad = 100;
  auto eight = m;
  eight.bits = 8; // Analysis precision does not follow render depth.
  TestCompensate<T> plan(video, super, 3, eight, 3, p);
  auto r = ref.view();
  constant(plan.render(0, field(eight, 99), clip.pixels(), current.view(), &r), 0, T(80));
  constant(plan.render(0, field(eight, 100), clip.pixels(), current.view(), &r), 0, T(10));
  constant(plan.render(2, field(eight), clip.pixels(), current.view(), nullptr), 0, T(5));
  for (int n : {0, 2})
    test::verify_destination<T>([&](const RenderDestination<T>* dst) {
      return plan.render(n, field(eight, 99), clip.pixels(), current.view(), n ? nullptr : &r, {}, {}, dst);
    });
  auto absent = field(eight);
  absent.state = FieldState::metadata_only;
  absent.grid = {0, 0, {}};
  constant(plan.render(0, absent, clip.pixels(), current.view(), nullptr), 0, T(5));
  rejects([&] { plan.render(0, field(eight), clip.pixels(), current.view(), nullptr); });
  auto bad = current.view();
  bad.planes[0].planes[0] = {};
  rejects([&] { plan.render(0, absent, clip.pixels(), bad, nullptr); });
  auto changed = field(eight);
  ++changed.metadata.delta;
  rejects([&] { plan.reference(changed, 0); });
  changed = field(eight);
  changed.metadata.levels = 100;
  constant(plan.render(0, changed, clip.pixels(), current.view(), &r), 0, T(80));
  auto corrupt = field(eight);
  corrupt.grid.values[0].error = -1;
  rejects([&] { plan.render(2, corrupt, clip.pixels(), current.view(), nullptr); });
  m.delta = 0;
  m.bits = 8;
  TestCompensate<T> same(video, super, 3, m, 3);
  CHECK(same.reference(field(m), 2) == 2);
  auto c = current.view();
  constant(same.render(2, field(m), clip.pixels(), c, &c), 0, T(10));
}

void admission_and_fields() {
  SuperGeometryParams geometry{16, 16, 8, 8, 0, 0, 3, 3, 2, 2, 2, true};
  SuperPlan<std::uint8_t> bad(geometry, 8);
  RenderVideo video{16, 16, 8, true, 2, 2, 3};
  auto m = super_analysis_metadata(bad, 1, false);
  rejects([&] { TestCompensate<std::uint8_t>(video, bad, 3, m, 3); });
  CompensateParameters zero;
  zero.time = 0;
  TestCompensate<std::uint8_t>(video, bad, 3, m, 3, zero);
  geometry.pad_x = geometry.pad_y = 4;
  SuperPlan<std::uint8_t> super(geometry, 8);
  m = super_analysis_metadata(super, 1, false);
  CompensateParameters p;
  p.fields = true;
  p.thsad = 0;
  TestCompensate<std::uint8_t> plan(video, super, 3, m, 3, p);
  Image<std::uint8_t> clip(super, 5), current(super, 10), ref(super, 80);
  auto r = ref.view();
  auto f = field(m);
  rejects([&] { plan.render(0, f, clip.pixels(), current.view(), &r); });
  constant(plan.render(0, f, clip.pixels(), current.view(), &r, true, false), 2, std::uint8_t(10));
  f.grid.values[0].vector.y = -8; // Publicly valid, unsafe after the actual -1 field shift.
  rejects([&] { plan.render(0, f, clip.pixels(), current.view(), &r, false, true); });
  // The same unavailable field needs neither reference nor field parity.
  constant(plan.render(2, f, clip.pixels(), current.view(), nullptr), 0, std::uint8_t(5));
  auto changed = geometry;
  changed.one_level = true;
  Image<std::uint8_t> shallow(SuperPlan<std::uint8_t>(changed, 8), 10);
  plan.validate_current(shallow.view());

  SuperPlan<float> fs(geometry, 32);
  auto fm = super_analysis_metadata(fs, 1, false);
  fm.bits = 8;
  TestCompensate<float> fp({16, 16, 32, true, 2, 2, 3}, fs, 3, fm, 3, p);
  Image<float> fc(fs, 5), cur(fs, std::numeric_limits<float>::quiet_NaN()), fr(fs, 80);
  auto ff = field(fm);
  ff.grid.values[1].vector.y = -8; // Second block must fail before first-block NaN sampling.
  auto rv = fr.view();
  try {
    fp.render(0, ff, fc.pixels(), cur.view(), &rv, false, true);
    CHECK(false);
  } catch (const std::invalid_argument& error) {
    CHECK(std::string(error.what()).find("logical phase domain") != std::string::npos);
  }
}

void degrain() {
  SuperPlan<std::uint8_t> super({8, 8, 8, 8, 0, 0, 4, 4}, 8);
  RenderVideo video{8, 8, 8, false, 1, 1, 3};
  auto a = super_analysis_metadata(super, -1), b = super_analysis_metadata(super, 1);
  Image<std::uint8_t> clip(super, 5), centre(super, 100), past(super, 80), future(super, 140);
  TestDegrain<std::uint8_t> plan(video, super, 3, {a, b}, {3, 3});
  std::vector<AnalysisField> fields{field(a), field(b)};
  test::verify_destination<std::uint8_t>([&](const RenderDestination<std::uint8_t>* dst) {
    return plan.render(1, fields, clip.pixels(), centre.view(), {past.view(), future.view()}, dst);
  });
  constant(plan.render(1, fields, clip.pixels(), centre.view(), {past.view(), future.view()}), 0, std::uint8_t(107));
  constant(plan.render(2, fields, clip.pixels(), centre.view(), {past.view(), {}}), 0, std::uint8_t(90));
  auto absent = fields;
  for (auto& f : absent) {
    f.state = FieldState::metadata_only;
    f.grid = {0, 0, {}};
  }
  constant(plan.render(1, absent, clip.pixels(), centre.view(), {{}, {}}), 0, std::uint8_t(100));
  DegrainParameters limited;
  limited.limit = {2.2, 2.2};
  TestDegrain<std::uint8_t> limiter(video, super, 3, {a, b}, {3, 3}, limited);
  constant(limiter.render(1, fields, clip.pixels(), centre.view(), {past.view(), future.view()}), 0, std::uint8_t(102));
  DegrainParameters none;
  none.planes = {false, false, false};
  none.weights = {0, 0, 0};
  TestDegrain<std::uint8_t> disabled(video, super, 3, {a, b}, {3, 3}, none);
  constant(disabled.render(1, fields, clip.pixels(), centre.view(), {past.view(), future.view()}), 0, std::uint8_t(5));
  rejects([&] { disabled.render(1, fields, clip.pixels(), centre.view(), {{}, {}}); });
  auto corrupt = fields;
  corrupt[1].grid.values[0].error = -1;
  rejects([&] { disabled.references(corrupt, 2); });
  rejects([&] { TestDegrain<std::uint8_t>(video, super, 3, {a, b}, {3, 2}); });
  // Cropped overlapping grid, centre-only output still uses the composition path.
  SuperPlan<std::uint8_t> wide({18, 16, 8, 8, 0, 0, 4, 4}, 8);
  a = super_analysis_metadata(wide, -1);
  a.overlap_x = a.overlap_y = 4;
  a.blocks_x = 4;
  a.blocks_y = 3;
  b = a;
  b.delta = 1;
  Image<std::uint8_t> wc(wide, 100), wp(wide, 5);
  TestDegrain<std::uint8_t> overlap({18, 16, 8, false, 1, 1, 3}, wide, 3, {a, b}, {3, 3});
  auto fa = field(a), fb = field(b);
  fa.state = fb.state = FieldState::metadata_only;
  constant(overlap.render(1, {fa, fb}, wp.pixels(), wc.view(), {{}, {}}), 0, std::uint8_t(100));
}

template <class T>
void chroma_and_precision(int bits) {
  SuperPlan<T> super({16, 16, 8, 8, 0, 0, 4, 4, 2, 2, 2, true}, bits);
  RenderVideo video{16, 16, bits, true, 2, 2, 3};
  auto a = super_analysis_metadata(super, -1, false), b = super_analysis_metadata(super, 1, false);
  a.bits = b.bits = 8;
  Image<T> clip(super, T(5)), centre(super, T(100)), past(super, T(80)), future(super, T(140));
  DegrainParameters p;
  p.planes = {false, true, false};
  TestDegrain<T> plan(video, super, 3, {a, b}, {3, 3}, p);
  auto result = plan.render(1, {field(a), field(b)}, clip.pixels(), centre.view(), {past.view(), future.view()});
  constant(result, 0, T(5));
  constant(result, 1, std::is_same_v<T, float> ? T(106.640625) : T(107));
  constant(result, 2, T(5));
  p.limit[1] = 2.2;
  TestDegrain<T> limited(video, super, 3, {a, b}, {3, 3}, p);
  auto capped = limited.render(1, {field(a), field(b)}, clip.pixels(), centre.view(), {past.view(), future.view()});
  constant(capped, 0, T(5));
  constant(capped, 1, std::is_same_v<T, float> ? T(102.2f) : T(102));
  constant(capped, 2, T(5));
  for (std::size_t i = 0; i < future.storage.size(); ++i) {
    auto v = future.storage[i].view();
    for (int y = 0; y < v.height(); ++y)
      for (int x = 0; x < v.width(); ++x)
        v.row(y)[x] = T(40 + i % 4);
  }
  auto vectors = field(b);
  for (auto& v : vectors.grid.values)
    v.vector.x = -1;
  TestCompensate<T> compensated(video, super, 3, b, 3);
  auto r = future.view();
  auto shifted = compensated.render(0, vectors, clip.pixels(), centre.view(), &r);
  for (int k = 0; k < 3; ++k)
    constant(shifted, k, T(41)); // Negative odd chroma vectors use floor, not truncation.
}
template <class T>
void wiener_chroma(int bits) {
  for (int pel : {1, 2, 4})
    for (int ry : {1, 2}) {
      SuperPlan<T> super({16, 16, 8, 8, 0, 0, 16, 16, 2, ry, pel, true}, bits);
      RenderVideo video{16, 16, bits, true, 2, ry, 3};
      auto a = super_analysis_metadata(super, -1, false), b = super_analysis_metadata(super, 1, false);
      a.bits = b.bits = 8;
      Image<T> centre(super, T(20)), reference(super, T(200));
      for (std::size_t n = 0; n < reference.storage.size(); n += pel * pel) {
        auto v = reference.storage[n].view();
        for (int y = 0; y < v.height(); ++y)
          for (int x = 0; x < v.width(); ++x)
            v.row(y)[x] = T(8 + 2 * x + 2 * y);
      }
      for (int delta : {-3, -1, 1, 3, 8}) {
        auto vectors = field(b);
        for (auto& v : vectors.grid.values)
          v.vector = {delta, delta};
        CompensateParameters cp;
        cp.chroma_wiener = true;
        TestCompensate<T> plan(video, super, 3, b, 3, cp);
        auto ref = reference.view();
        auto result = plan.render(0, vectors, centre.pixels(), centre.view(), &ref);
        const double exact = 8 + 2 * (8 + 16 / ry) + 2.0 * delta / pel * (0.5 + 1.0 / ry);
        const T expected = std::is_floating_point_v<T> ? T(exact) : T(std::floor(exact + 0.5));
        CHECK(result[1].view().row(0)[0] == expected);
        CHECK(result[2].view().row(0)[0] == expected);
        DegrainParameters dp;
        dp.chroma_wiener = true;
        dp.weights = {0, 0, 1};
        TestDegrain<T> degrain(video, super, 3, {a, b}, {3, 3}, dp);
        auto denoised = degrain.render(1, {field(a), vectors}, centre.pixels(), centre.view(), {ref, ref});
        const T blended = std::is_floating_point_v<T> ? T((20 + 255.0 * expected) / 256)
                                                       : T((20 + 255 * int(expected) + 128) / 256);
        CHECK(denoised[1].view().row(0)[0] == blended);
        CHECK(denoised[2].view().row(0)[0] == blended);
        dp.planes = {false, true, false};
        TestDegrain<T> u_only(video, super, 3, {a, b}, {3, 3}, dp);
        auto u = u_only.render(1, {field(a), vectors}, centre.pixels(), centre.view(), {ref, ref});
        CHECK(u[1].view().row(0)[0] == blended);
        constant(u, 2, T(20));
      }
    }
}

template <class T>
void wiener_kernel(int bits) {
  RenderPhaseGeometry g{4, 2, 2, 2, 2};
  for (auto& e : g.phases)
    e = {6, 6};
  std::array<T, 36> samples{};
  SubpixelPhases<T> image{4, {}};
  image.planes[0] = checked_plane<const T>(samples.data(), 6, 6, 6 * sizeof(T), sizeof(samples));
  samples[14] = T(128);
  const auto sample = [&](int x, int y) {
    SampledRenderBlock<T> block{samples.data() + 14, 6, nullptr};
    std::vector<std::vector<T>> owned;
    sample_chroma_wiener(block, owned, g, {0, 0, 2, 2}, {x, y}, image, bits);
    return block.data[0];
  };
  // Impulse response at integer, eighth, quarter, and half positions.
  const int expected[] = {128, 116, 104, 92, 80, 60, 40, 20};
  for (int phase = 0; phase < 8; ++phase) {
    CHECK(sample(phase, 0) == T(expected[phase]));
    CHECK(sample(0, phase) == T(expected[phase]));
    for (int other = 0; other < 8; ++other) {
      const double exact = double(expected[phase]) * expected[other] / 128;
      const T diagonal = std::is_floating_point_v<T> ? T(exact) : T(std::floor(exact + 0.5));
      const T actual = sample(phase, other);
      if (actual != diagonal)
        throw std::runtime_error("Wiener impulse bits=" + std::to_string(bits) + " phase=" +
                                 std::to_string(phase) + "," + std::to_string(other) + " actual=" +
                                 std::to_string(actual) + " expected=" + std::to_string(diagonal));
    }
  }
  CHECK(sample(-1, 0) == T(116));
  samples[14] = T(1);
  CHECK(sample(4, 4) == (std::is_floating_point_v<T> ? T(0.390625) : T(0)));
  // Rounding after each axis would give 1 here for integer samples.
  // Negative lobes survive internally; clip integer output to its bit depth.
  const T maximum = std::is_floating_point_v<T> ? T(1) : T((1 << bits) - 1);
  samples.fill(T(0));
  for (int x : {0, 2, 3, 5})
    samples[12 + x] = maximum;
  CHECK(sample(4, 0) == (std::is_floating_point_v<T> ? T(1.3125) : maximum));
  samples.fill(T(0));
  samples[13] = samples[16] = maximum;
  CHECK(sample(4, 0) == (std::is_floating_point_v<T> ? T(-0.3125) : T(0)));
  // Every tap clamps at a one-pixel logical edge; no padding is assumed.
  T edge = T(37);
  image.planes[0] = checked_plane<const T>(&edge, 1, 1, sizeof(T), sizeof(T));
  g.pad_x = g.pad_y = 0;
  for (auto& e : g.phases)
    e = {1, 1};
  SampledRenderBlock<T> block{&edge, 1, nullptr};
  std::vector<std::vector<T>> owned;
  sample_chroma_wiener(block, owned, g, {0, 0, 2, 2}, {7, 7}, image, bits);
  CHECK(block.data[0] == edge);
  block.data = &edge;
  sample_chroma_wiener(block, owned, g, {0, 0, 2, 2}, {0, 0}, image, bits);
  CHECK(block.data == &edge && owned.size() == 1);
}

struct CountingKernels : TestKernels<std::uint8_t> {
  inline static int calls = 0;
  static std::int64_t scene_count(const AnalysisMetadata& m, const MotionGrid& grid, std::int64_t threshold) {
    ++calls;
    return TestKernels<std::uint8_t>::scene_count(m, grid, threshold);
  }
};
void scene_dispatch() {
  const SuperPlan<std::uint8_t> super({8, 8, 8, 8, 0, 0, 4, 4}, 8);
  const RenderVideo video{8, 8, 8, false, 1, 1, 3};
  const auto a = super_analysis_metadata(super, -1), b = super_analysis_metadata(super, 1);
  const CompensateFramePlan<std::uint8_t, CountingKernels> compensate(video, super, 3, a, 3);
  CountingKernels::calls = 0;
  CHECK(!compensate.reference(field(a), 0)); // Validate before temporal fallback.
  CHECK(CountingKernels::calls == 1);
  auto missing = field(a);
  missing.state = FieldState::metadata_only;
  CHECK(!compensate.reference(missing, 0));
  CHECK(CountingKernels::calls == 1);
  DegrainParameters p;
  p.weights = {0, 0, 0};
  const DegrainFramePlan<std::uint8_t, CountingKernels> degrain(video, super, 3, {a, b}, {3, 3}, p);
  CHECK(degrain.references({field(a), field(b)}, 1).size() == 2);
  CHECK(CountingKernels::calls == 3); // Zero weights do not bypass validation.
  auto corrupt = field(b);
  corrupt.grid.values.back().error = -1;
  rejects([&] { degrain.references({field(a), corrupt}, 2); });
  CHECK(CountingKernels::calls == 5);
}
} // namespace
int main() {
  try {
    compensation<std::uint8_t>(8);
    compensation<std::uint16_t>(16);
    compensation<float>(32);
    admission_and_fields();
    degrain();
    scene_dispatch();
    wiener_chroma<std::uint8_t>(8);
    wiener_chroma<std::uint16_t>(16);
    wiener_chroma<float>(32);
    wiener_kernel<std::uint8_t>(8);
    wiener_kernel<std::uint16_t>(10);
    wiener_kernel<float>(32);
    chroma_and_precision<std::uint8_t>(8);
    chroma_and_precision<std::uint16_t>(16);
    chroma_and_precision<float>(32);
    std::cout << "render frame tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
