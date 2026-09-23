#include "core/mask/input.hpp"

#include <iostream>
#include <random>
#include <cfenv>

namespace {
using namespace neo_mv;

void check(bool condition, int line) {
  if (!condition)
    throw std::runtime_error("mask input assertion failed at line " + std::to_string(line));
}
#define CHECK(condition) check((condition), __LINE__)
template <class E = std::invalid_argument, class F>
void rejects(F call) {
  bool caught = false;
  try {
    call();
  } catch (const E&) {
    caught = true;
  }
  CHECK(caught);
}

AnalysisField field(int bits = 8, int delta = 1) {
  AnalysisMetadata m;
  m.width = m.height = m.real_width = m.real_height = 16;
  m.pad_x = m.pad_y = 4;
  m.pel = 2;
  m.levels = 1;
  m.ratio_x = m.ratio_y = 1;
  m.block_width = m.block_height = 8;
  m.blocks_x = m.blocks_y = 2;
  m.bits = bits;
  m.delta = delta;
  return {m, FieldState::complete, {2, 2, {{{0, 0}, 400}, {{0, 0}, 401}, {{0, 0}, 900}, {{0, 0}, 0}}}};
}

void creation_and_fallback() {
  const auto m = field().metadata;
  MaskParameters p;
  p.scval = 12.5;
  p.time = 50;
  MaskInputPlan<std::uint8_t> plan(m, 1, p);
  CHECK(plan.fallback() == 13 && plan.time256() == 128 && plan.bits() == 8);
  CHECK(plan.gamma() == 1.0f && plan.f() == 1.0f / 100.0f);
  p.scval = -1;
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).fallback() == 0);
  p.scval = -1.5;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p.scval = 255.49;
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).fallback() == 255);
  p.scval = 255.5;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p.scval = 1022.5;
  CHECK(MaskInputPlan<std::uint16_t>(field(10).metadata, 1, p).fallback() == 1023);
  p.scval = 1.25;
  CHECK(MaskInputPlan<float>(field(32).metadata, 1, p).fallback() == 1.25f);
  p.scval = -2;
  CHECK(MaskInputPlan<float>(field(32).metadata, 1, p).fallback() == -2.0f);
  for (double value : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    p.scval = value;
    rejects([&] { MaskInputPlan<float> bad(field(32).metadata, 1, p); });
  }
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 0); });
  rejects([&] { MaskInputPlan<std::uint8_t> bad({}, 1); });
  rejects([&] { MaskInputPlan<std::uint16_t> bad(m, 1); });
  rejects([&] { MaskInputPlan<float> bad(m, 1); });
  rejects([&] { MaskInputPlan<std::uint8_t> bad(field(10).metadata, 1); });

  // Non-render block sizes and impossible Super sampling do not prohibit mask
  // creation. Masks only need the public metadata and grid coverage.
  auto unusual = m;
  unusual.width = unusual.height = unusual.real_width = unusual.real_height = 6;
  unusual.block_width = unusual.block_height = 3;
  unusual.pad_x = unusual.pad_y = 0;
  MaskInputPlan<std::uint8_t> accepted(unusual, 1);
  CHECK(accepted.metadata().block_width == 3);
  auto cropped = m;
  cropped.real_width = 14;
  CHECK(MaskInputPlan<std::uint8_t>(cropped, 1).metadata().real_width == 14);
  cropped.blocks_x = 1;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(cropped, 1); });
  cropped.blocks_x = 3;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(cropped, 1); });
  cropped = m;
  cropped.blocks_x = INT32_MAX;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(cropped, 1); });
}

void conversion_differential() {
  // Independent arithmetic oracle: scale, round to an even integer, rescale.
  auto reference = [](double value) {
    const double magnitude = std::abs(value);
    if (magnitude == 0)
      return static_cast<float>(value);
    int exponent;
    std::frexp(magnitude, &exponent);
    const int shift = (std::max)(exponent - 24, -149);
    const double scaled = std::ldexp(magnitude, -shift);
    double rounded = std::floor(scaled);
    const auto remainder = scaled - rounded;
    if (remainder > 0.5 || (remainder == 0.5 && std::fmod(rounded, 2.0) != 0))
      ++rounded;
    const float result = static_cast<float>(std::ldexp(rounded, shift));
    return std::signbit(value) ? -result : result;
  };
  std::mt19937_64 random(20260922);
  std::vector<std::pair<double, float>> samples;
  for (int i = 0; i < 30000; ++i) {
    const auto bits = random();
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    if (std::isfinite(value) && std::abs(value) < 0x1.ffffffp127)
      samples.emplace_back(value, reference(value));
  }
  for (int exponent = -149; exponent <= 127; ++exponent) {
    const double midpoint = std::ldexp(1.0, exponent) + std::ldexp(1.0, (std::max)(exponent - 24, -150));
    for (double value : {std::nextafter(midpoint, 0.0), midpoint, std::nextafter(midpoint, INFINITY)})
      for (double sign : {-1.0, 1.0})
        samples.emplace_back(sign * value, reference(sign * value));
  }
  const int saved = std::fegetround();
  bool equal = true;
  for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    if (std::fesetround(mode) != 0)
      continue;
    for (const auto& sample : samples) {
      const float actual = mask_detail::binary32(sample.first);
      equal = equal && std::memcmp(&actual, &sample.second, sizeof(actual)) == 0;
    }
  }
  std::fesetround(saved);
  CHECK(equal);
#if defined(__SSE2__) || defined(_M_X64)
  const auto saved_csr = _mm_getcsr();
  _mm_setcsr((saved_csr & ~0x6000u) | 0x8040u); // Nearest, with FTZ and DAZ enabled.
  for (const auto& sample : samples) {
    const float actual = mask_detail::binary32(sample.first);
    equal = equal && std::memcmp(&actual, &sample.second, sizeof(actual)) == 0;
  }
  _mm_setcsr(saved_csr);
  CHECK(equal);
#endif
}

void parameter_rounding() {
  using mask_detail::binary32;
  const float smallest = std::numeric_limits<float>::denorm_min();
  CHECK(binary32(0x1p-149) == smallest);
  CHECK(binary32(0x1p-150) == 0.0f);
  CHECK(binary32(0x1.8p-149) == 2 * smallest);
  CHECK(std::signbit(binary32(-0x1p-150)));
  CHECK(binary32(1.0 + 0x1p-24) == 1.0f);
  CHECK(binary32(1.0 + 3 * 0x1p-24) == 1.0f + 0x1p-22f);
  CHECK(binary32(std::nextafter(0x1.ffffffp127, 0.0)) == std::numeric_limits<float>::max());
  rejects([&] { binary32(0x1.ffffffp127); });
  rejects([&] { binary32(-0x1.ffffffp127); });
  rejects([&] { binary32(std::numeric_limits<double>::max()); });

  const auto m = field().metadata;
  MaskParameters p;
  p.ml = std::numeric_limits<float>::max();
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).f() > 0);
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).f() < std::numeric_limits<float>::min());
  p.ml = 0x1p-128;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p.ml = 0x1p-150;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p = {};
  p.gamma = -0x1p-150;
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).gamma() == 0);
  p.gamma = -0x1p-149;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p = {};
  p.thscd2 = std::nextafter(100.0, 101.0);
  MaskInputPlan<std::uint8_t> rounded_percentage(m, 1, p);
  p.thscd2 = -0x1p-150;
  MaskInputPlan<std::uint8_t> negative_zero_percentage(m, 1, p);
  p.thscd2 = 100.001;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p = {};
  p.thscd1 = 16321;
  rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  p = {};
  for (double time : {-1.0, 100.001, std::numeric_limits<double>::infinity()}) {
    p.time = time;
    rejects([&] { MaskInputPlan<std::uint8_t> bad(m, 1, p); });
  }
  p.time = 0;
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).time256() == 0);
  p.time = 100;
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).time256() == 256);
  p.time = std::nextafter(100.0, 0.0);
  CHECK(MaskInputPlan<std::uint8_t>(m, 1, p).time256() == 255);
}

void eligibility() {
  auto f = field();
  MaskParameters p;
  p.thscd2 = 50;
  const MaskInputPlan<std::uint8_t> plan(f.metadata, 1, p);
  CHECK(plan.eligible(f)); // delta=1 is beyond this one-frame clip; no reference needed.
  f.grid.values[0].error = 401;
  CHECK(!plan.eligible(f));
  f.grid.values[3].error = -1;
  rejects([&] { plan.eligible(f); }); // complete validation continues after scene cut.
  f = field();
  f.grid.values[3].vector.x = INT32_MAX;
  rejects([&] { plan.eligible(f); });
  f = field();
  f.grid.values.pop_back();
  rejects([&] { plan.eligible(f); });
  f = field();
  f.grid.height = 1;
  rejects([&] { plan.eligible(f); });
  f = field();
  f.metadata.levels = 9;
  CHECK(plan.eligible(f));
  f.state = FieldState::metadata_only;
  f.grid = {0, 0, {}};
  CHECK(!plan.eligible(f));
  f.metadata.delta = 0;
  rejects([&] { plan.eligible(f); });
  for (auto scalar : field_detail::scalars) {
    if (scalar.member == &AnalysisMetadata::levels)
      continue;
    auto changed = field();
    ++(changed.metadata.*(scalar.member));
    changed.state = FieldState::metadata_only;
    rejects([&] { plan.eligible(changed); });
  }
  f = field();
  f.metadata.chroma = true;
  f.state = FieldState::metadata_only;
  rejects([&] { plan.eligible(f); });
  f.state = FieldState::invalid_metadata;
  f.metadata = {};
  CHECK(!plan.eligible(f));
  for (int delta : {INT32_MIN, INT32_MAX}) {
    f = field(8, delta);
    CHECK(MaskInputPlan<std::uint8_t>(f.metadata, 1, p).eligible(f));
  }
}

void scores_and_power() {
  using mask_detail::power;
  using mask_detail::quantize;
  CHECK(quantize<std::uint8_t>(127.5, 8) == 127);
  CHECK(quantize<std::uint16_t>(1023.75, 10) == 1023);
  CHECK(quantize<std::uint8_t>(1e100, 8) == 255);
  CHECK(quantize<float>(0.5, 32) == 0.5f);
  CHECK(quantize<float>(2.0, 32) == 1.0f);
  CHECK(quantize<float>(0x1p-149, 32) == std::numeric_limits<float>::denorm_min());
  rejects([&] { quantize<std::uint8_t>(std::numeric_limits<double>::infinity(), 8); });
  rejects([&] { quantize<float>(std::numeric_limits<float>::quiet_NaN(), 32); });
  rejects([&] { quantize<std::uint8_t>(-0.1, 8); });
  CHECK(power(0.0f, 0.0f) == 1.0f);
  CHECK(power(0.0, 2.0) == 0.0);
  CHECK(power(123.0, 0.0) == 1.0);
  CHECK(power(0.25, 1.0) == 0.25);
  CHECK(power(0.5f, 149.0f) == std::numeric_limits<float>::denorm_min());
  CHECK(power(0.5, 1074.0) == std::numeric_limits<double>::denorm_min());
  CHECK(power(0.25, 0.5) >= std::nextafter(0.5, 0.0));
  CHECK(power(0.25, 0.5) <= std::nextafter(0.5, 1.0));
  rejects([&] { power(std::numeric_limits<float>::infinity(), 0.0f); });
  rejects([&] { power(-1.0, 0.0); });
  rejects([&] { power(1.0, -1.0); });
  rejects<std::overflow_error>([&] { power(2.0, 1024.0); });
}
} // namespace

int main() {
  try {
    creation_and_fallback();
    parameter_rounding();
    conversion_differential();
    eligibility();
    scores_and_power();
    std::cout << "Scalar mask input and numeric checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
