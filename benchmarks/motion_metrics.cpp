// Optional manual benchmark; no timing assertions. Setup is outside timers.
// Usage: neo_mv_motion_metrics_benchmark [block=16] [iterations=1000]
#include "../tests/core/motion_fixture.hpp"
#include "core/motion/analyse.hpp"
#include "highway/kernels.hpp"
#include <chrono>
#include <iostream>

using namespace neo_mv;
namespace {
volatile std::int64_t sink = 0;
template <class F> double timed(int count, F&& run) {
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < count; ++i) run(i);
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count()/count;
}
template <class K> void measure(int block, int iterations, const char* backend) {
  for (int scene = 0; scene < 4; ++scene) {
    MotionFixture<std::uint8_t> f(block*2,block*2,block,block,8,1,false);
    const int stride = f.geometry.planes[0].current.width+1;
    for (std::size_t i = 0; i < f.source[0].size(); ++i) {
      const int y = int(i)/stride, x = int(i)%stride;
      const int s = (x*13+y*7)%160+16;
      f.source[0][i] = std::uint8_t(s);
      f.reference[0][0][i] = std::uint8_t(scene==0 ? s : scene==1 ? s+32 :
          scene==2 ? s+(x%16<8 ? 32:0) : (x*37+y*19)%256);
    }
    const auto region = analysis_block(f.metadata,0,0);
    const auto source = source_block(f.geometry,region,f.frames,0);
    const auto prepared = K::prepare_frames(f.geometry,f.frames);
    for (const auto& d : motion_metrics) {
      if (d.transform == MetricTransform::satd && block%4) continue;
      const auto config = make_metric_config(d.name);
      MetricFrameContext context;
      const auto stats_us = timed(100,[&](int) {
        context = metric_frame_context<std::uint8_t,K>(f.metadata,f.geometry,f.frames);
        sink = context.base_weight;
      });
      const auto plan = metric_layer_plan(config,context);
      MetricScratch scratch;
      int triggered = 0;
      const auto ls = block_luma_sum(source);
      for (int i = 0; i < 25; ++i)
        triggered += local_metric_active(ls,block_luma_sum(reference_block(f.geometry,region,f.frames,0,
            {i%5-2,i/5-2})),config.threshold);
      const auto candidate_us = with_motion_evaluator<std::uint8_t,K>(plan,f.geometry,region,f.frames,prepared,
          scratch,8,[&](auto& eval) {
            sink = eval({0,0}).raw; // warm up lazy source transform
            return timed(iterations,[&](int i) { sink = eval({i%5-2,i/5%5-2}).raw; });
          });
      AnalyseControls controls; controls.metric=config; controls.badrange=0;
      const auto analysis_us = timed(10,[&](int) {
        const auto grid = analyse_vectors<std::uint8_t,K>(f.metadata,{f.geometry},{f.frames},controls);
        sink = grid.values.front().error;
      });
      // This intentionally measures one-level Analyse (global modes -> SAD).
      // candidate_us separately measures the nonzero frame-pair policy.
      std::cout << backend << ',' << block << ",8,0,1," << scene << ',' << d.name << ',' << context.base_weight
                << ',' << triggered/25.0 << ',' << stats_us << ',' << candidate_us << ',' << analysis_us << '\n';
    }
  }
}
}
int main(int argc,char** argv) {
  try {
    const int block=argc>1 ? std::stoi(argv[1]):16;
    const int iterations=argc>2 ? std::stoi(argv[2]):1000;
    if (!geometry_detail::block_pair(block,block) || iterations<=0) throw std::invalid_argument("invalid benchmark arguments");
    std::cout << "backend,block,bits,chroma,levels,scene,metric,base_weight,local_trigger_fraction,stats_us,candidate_us,analysis_us\n";
    measure<ScalarKernels<std::uint8_t>>(block,iterations,"scalar");
    measure<HighwayKernels<std::uint8_t>>(block,iterations,"highway");
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
