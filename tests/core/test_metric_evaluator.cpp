#include "core/motion/analyse.hpp"
#include "core/motion/recalculate.hpp"
#include "motion_fixture.hpp"
#include <iostream>
#if NEO_MV_TEST_HIGHWAY
#include "highway/kernels.hpp"
#include "hwy/targets.h"
#endif

using namespace neo_mv;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
void same(const MotionGrid& a, const MotionGrid& b) {
  check(a.values.size() == b.values.size(), "grid size");
  for (std::size_t i = 0; i < a.values.size(); ++i)
    check(a.values[i].vector.x == b.values[i].vector.x && a.values[i].vector.y == b.values[i].vector.y &&
          a.values[i].error == b.values[i].error, "full grid mismatch");
}
template <class T, class K>
void candidates() {
  for (int bits : {8, 10, 12, 16}) {
    if (bits > int(sizeof(T) * 8)) continue;
    for (auto shape : {std::pair{4,4}, {6,6}, {8,4}, {8,8}, {12,12}, {16,2}, {16,8}, {16,16}, {24,24},
                       {32,16}, {32,32}, {48,48}, {64,32}, {64,64}, {128,64}, {128,128}})
      for (int pel : {1, 2, 4}) {
        MotionFixture<T> f(shape.first, shape.second, shape.first, shape.second, 4, pel, true);
        f.metadata.bits = bits;
        // Include 4:4:4 and 4:2:0; separate host tests cover 4:2:2.
        const int max = (1 << bits) - 1;
        for (int k = 0; k < 3; ++k) {
          for (std::size_t i = 0; i < f.source[k].size(); ++i) f.source[k][i] = T((i*13 + k*17) % (max+1));
          for (int phase = 0; phase < pel*pel; ++phase)
            for (std::size_t i = 0; i < f.reference[k][phase].size(); ++i)
              f.reference[k][phase][i] = T((i*17 + phase*19 + max/4) % (max+1));
        }
        const auto b = analysis_block(f.metadata, 0, 0);
        const auto src = source_block(f.geometry, b, f.frames, 0);
        check(K::block_luma_sum(src) == block_luma_sum(src), "SIMD source sum");
        const auto prepared = K::prepare_frames(f.geometry, f.frames);
        for (const auto& descriptor : motion_metrics) {
          if (descriptor.transform == MetricTransform::satd && (b.width%4 || b.height%4)) continue;
          auto config = make_metric_config(descriptor.name);
          if (descriptor.mix == MetricMix::local) config = make_metric_config(descriptor.name, 0.6, 0.04);
          const auto plan = metric_layer_plan(config, {9});
          MetricScratch scratch;
          with_motion_evaluator<T, K>(plan, f.geometry, b, f.frames, prepared, scratch, bits, [&](auto& eval) {
            for (MotionVector v : {MotionVector{0,0}, {-1,-1}, {pel+1,-pel}}) {
              const auto ref = reference_block(f.geometry, b, f.frames, 0, v);
              const auto pair = K::sad_and_reference_sum(src, ref);
              const auto s = block_metric(src, ref, BlockMetric::sad);
              check(pair.sad == s && pair.reference_sum == block_luma_sum(ref), "fused statistics");
              std::int64_t x = s;
              if (descriptor.transform == MetricTransform::satd) x = block_metric(src, ref, BlockMetric::satd);
              if (descriptor.transform == MetricTransform::dct) {
                DctWorkspace dct(b.width, b.height, bits);
                dct.set_source(src);
                const auto parts = dct.compare_parts(ref);
                const auto scale = std::int64_t(std::sqrt(b.width*b.height) + 0.5);
                x = (parts.coefficient_l1 + (descriptor.dc_weight-1)*parts.dc_abs) * scale / 2;
              }
              auto expected = x;
              if (descriptor.mix == MetricMix::global) {
                const int w = 9 / descriptor.global_divisor;
                expected = (s*(16-w) + x*w)/16;
              }
              if (descriptor.mix == MetricMix::local) {
                const auto ls = block_luma_sum(src), lr = block_luma_sum(ref);
                expected = std::abs(ls-lr)*65536 > (ls+lr)*2621 ? (s*26214 + x*39322)/65536 : s;
              }
              const auto plain = block_error<T, true>(f.geometry, b, f.frames, v, BlockMetric::sad);
              const auto actual = eval(v);
              check(actual.luma == expected && actual.chroma == plain.chroma &&
                    actual.raw == expected+plain.chroma, "candidate oracle mismatch");
            }
            return 0;
          });
        }
      }
  }
}
template <class K>
void contexts_and_search() {
  MotionFixture<std::uint8_t> f(16, 8, 8, 8);
  f.metadata.bits = 8;
  std::fill(f.source[0].begin(), f.source[0].end(), 50);
  std::fill(f.reference[0][0].begin(), f.reference[0][0].end(), 60);
  check(metric_frame_context<std::uint8_t, K>(f.metadata, f.geometry, f.frames).base_weight == 10, "global weight");
  // Equal positive/negative changes cancel before absolute value.
  for (int y = 0; y < 8; ++y)
    for (int x = 8; x < 16; ++x)
      f.reference[0][0][(y+4)*25+x+4] = 40;
  check(metric_frame_context<std::uint8_t, K>(f.metadata, f.geometry, f.frames).base_weight == 0, "signed cancellation");
  // A single untriggered local DCT block never allocates/executes a transform.
  MetricScratch scratch;
  const auto plan = metric_layer_plan(make_metric_config("sad_dct_local", 0.5, 0.9), {});
  PreparedMetricEvaluator<std::uint8_t,K> eval(plan, f.geometry, {0,0,8,8}, f.frames, scratch, 8);
  eval({0,0});
  check(!scratch.dct, "untriggered DCT allocation");
  const auto local = metric_layer_plan(make_metric_config("sad_dct_local", 1., 0.), {});
  PreparedMetricEvaluator<std::uint8_t,K> triggered(local, f.geometry, {0,0,8,8}, f.frames, scratch, 8);
  check(triggered({0,0}).luma == 20, "local DC once");
  check(bool(scratch.dct), "triggered workspace");
  const auto* workspace = scratch.dct.get();
  for (int y = 0; y < 8; ++y)
    for (int x = 8; x < 16; ++x) f.source[0][(y+4)*25+x+4] = 70;
  PreparedMetricEvaluator<std::uint8_t,K> next(local, f.geometry, {8,0,8,8}, f.frames, scratch, 8);
  check(next({0,0}).luma == 60 && workspace == scratch.dct.get(), "reuse and source reset");
  for (auto mode : {MotionMetric::sad_dct_global, MotionMetric::sad_satd_global, MotionMetric::sad_satd_global_half}) {
    AnalyseControls a; a.badrange = 0;
    const auto sad = analyse_vectors<std::uint8_t,K>(f.metadata, {f.geometry}, {f.frames}, a);
    a.metric = mode;
    same(sad, analyse_vectors<std::uint8_t,K>(f.metadata, {f.geometry}, {f.frames}, a));
    RecalculateControls r; r.metric = mode; r.thsad = INT32_MAX;
    const auto refined = recalculate_vectors<std::uint8_t,K>(f.old_field(), f.metadata, f.geometry, f.frames, r);
    const int weight = mode == MotionMetric::sad_satd_global_half ? 4 : 8;
    const int transform = mode == MotionMetric::sad_dct_global ? 80 : 320;
    check(refined.values[0].error == (640*(16-weight)+transform*weight)/16, "recalculate fixed weight");
  }
  for (const auto& d : motion_metrics) {
    AnalyseControls a; a.metric = make_metric_config(d.name); a.badrange=0; a.trymany=2;
    same(analyse_vectors<std::uint8_t,K>(f.metadata,{f.geometry},{f.frames},a),
         analyse_vectors<std::uint8_t>(f.metadata,{f.geometry},{f.frames},a));
  }
  f.metadata.overlap_x = 4;
  f.metadata.blocks_x = 3;
  std::fill(f.source[0].begin(), f.source[0].end(), 0);
  std::fill(f.reference[0][0].begin(), f.reference[0][0].end(), 0);
  for (int y = 0; y < 8; ++y)
    for (int x = 4; x < 12; ++x) f.reference[0][0][(y+4)*25+x+4] = 16;
  // Three overlapped blocks have changes 512,1024,512: trunc(2048/3)/64=10.
  // A whole-frame pixel average would incorrectly produce 8.
  check(metric_frame_context<std::uint8_t,K>(f.metadata,f.geometry,f.frames).base_weight == 10, "overlap grid");
  f.metadata.bits = 10;
  check(metric_frame_context<std::uint8_t,K>(f.metadata,f.geometry,f.frames).base_weight == 2, "bit normalization");
  std::fill(f.reference[0][0].begin(), f.reference[0][0].end(), 255);
  check(metric_frame_context<std::uint8_t,K>(f.metadata,f.geometry,f.frames).base_weight == 16, "saturation");
}
int main() {
  try {
    candidates<std::uint8_t, ScalarKernels<std::uint8_t>>();
    candidates<std::uint16_t, ScalarKernels<std::uint16_t>>();
    contexts_and_search<ScalarKernels<std::uint8_t>>();
#if NEO_MV_TEST_HIGHWAY
    for (auto target : hwy::SupportedAndGeneratedTargets()) {
      hwy::SetSupportedTargetsForTest(target);
      candidates<std::uint8_t, HighwayKernels<std::uint8_t>>();
      candidates<std::uint16_t, HighwayKernels<std::uint16_t>>();
      contexts_and_search<HighwayKernels<std::uint8_t>>();
      std::cout << hwy::TargetName(target) << " mixed metrics passed\n";
    }
    hwy::SetSupportedTargetsForTest(0);
#endif
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
