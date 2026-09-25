#include "core/motion/metric_context.hpp"
#include <cfenv>
#include <iostream>
#include <limits>

using namespace neo_mv;
void check(bool ok) { if (!ok) throw std::runtime_error("metric policy assertion failed"); }
template <class F> void rejects(F f) {
  try { f(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("expected metric rejection");
}
int main() {
  try {
    for (std::size_t i = 0; i < motion_metrics.size(); ++i) {
      const auto mode = parse_motion_metric(motion_metrics[i].name);
      check(std::size_t(mode) == i);
      auto config = make_metric_config(motion_metrics[i].name);
      for (int bits : {8, 10, 12, 16}) validate_motion_metric(config, 12, 12, bits);
      if (motion_metrics[i].integer_only()) rejects([&] { validate_motion_metric(config, 8, 8, 32); });
      else validate_motion_metric(config, 8, 8, 32);
      if (motion_metrics[i].transform == MetricTransform::satd) {
        rejects([&] { validate_motion_metric(config, 6, 6, 8); });
        rejects([&] { validate_motion_metric(config, 16, 2, 8); });
      }
      if (motion_metrics[i].mix != MetricMix::local) {
        rejects([&] { make_metric_config(motion_metrics[i].name, 0.5); });
        rejects([&] { make_metric_config(motion_metrics[i].name, {}, 0.03125); });
      }
    }
    for (auto name : {"", "2", "SAD", "sad_dct_local_50"}) rejects([&] { parse_motion_metric(name); });
    for (double value : {-1., 1.001, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()})
      rejects([&] { make_metric_config("sad_satd_local", value); });
    const int saved = std::fegetround();
    for (int mode : {FE_TONEAREST, FE_TOWARDZERO, FE_UPWARD, FE_DOWNWARD}) {
      check(std::fesetround(mode) == 0);
      check(quantize_metric_parameter(0) == 0 && quantize_metric_parameter(1) == 65536);
      check(quantize_metric_parameter(0.6) == 39322 && quantize_metric_parameter(0.04) == 2621);
      for (int n : {0, 1, 32767, 65535}) {
        const double half = (double(n) + 0.5) / 65536.;
        check(quantize_metric_parameter(half) == n + 1);
        check(quantize_metric_parameter(std::nextafter(half, 0.)) == n);
      }
      check(quantize_metric_parameter(std::numeric_limits<double>::denorm_min()) == 0);
    }
    check(std::fesetround(saved) == 0);
    check(mix_metric(3, 3, 32768) == 3); // separate half truncations would give 2
    check(mix_metric(100, 0, 32768) == 50); // SAD cannot be used as a lower bound
    check(mix_metric(101, 13, 39322) == 48);
    check(!local_metric_active(0, 0, 0));
    check(!local_metric_active(31, 33, 2048));
    check(local_metric_active(30, 34, 2048));
    check(!local_metric_active(1073725440, 0, 65536));
    check(local_metric_active(1073725440, 0, 65535));
    for (int w : {0, 1, 8, 15, 16}) {
      auto plan = metric_layer_plan(MotionMetric::sad_satd_global, {w});
      check(plan.weight == w);
      check(mix_metric(32, 16, w, 16) == 32 - w);
      check(metric_layer_plan(MotionMetric::sad_satd_global_half, {w}).weight == w / 2);
    }
    auto zero = make_metric_config("sad_satd_local", 0);
    check(metric_layer_plan(zero, {}).descriptor.transform == MetricTransform::sad);
    rejects([&] { validate_motion_metric(zero, 6, 6, 8); });
    std::cout << "Metric policy checks passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
