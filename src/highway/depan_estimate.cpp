#include "highway/depan_estimate.hpp"
#include <stdexcept>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/depan_estimate.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::estimate {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
void SamplesFinite(const float* values, std::size_t count) {
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  for (; count - i >= lanes; i += lanes)
    if (!hn::AllTrue(d, hn::IsFinite(hn::LoadU(d, values + i))))
      throw std::invalid_argument("non-finite DepanEstimate sample");
  for (; i < count; ++i)
    depan::finite(values[i]);
}
std::size_t ScanRow(const float* values, std::size_t count, float& sum, float& maximum) {
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0, index = count;
  for (; count - i >= lanes; i += lanes) {
    const auto v = hn::LoadU(d, values + i);
    // The confidence formula specifies this exact sequential sum. The peak
    // search is independent and may reduce, but must retain the first tie.
    for (std::size_t lane = 0; lane < lanes; ++lane)
      sum = depan::add(sum, values[i + lane]);
    const float candidate = hn::GetLane(hn::MaxOfLanes(d, v));
    if (candidate > maximum) {
      const auto lane = hn::FindFirstTrue(d, hn::Eq(v, hn::Set(d, candidate)));
      index = i + static_cast<std::size_t>(lane);
      // Load the original value so equal signed zeros retain the first sign.
      maximum = values[index];
    }
  }
  for (; i < count; ++i) {
    sum = depan::add(sum, values[i]);
    if (values[i] > maximum) {
      maximum = values[i];
      index = i;
    }
  }
  return index;
}
template <class D, class V>
V Check(D d, V value) {
  if (!hn::AllTrue(d, hn::IsFinite(value)))
    throw std::invalid_argument("non-finite DepanEstimate complex product");
  return value;
}
template <class D>
void Chunk(D d, float* current, const float* previous) {
  hn::Vec<D> ar, ai, br, bi;
  hn::LoadInterleaved2(d, current, ar, ai);
  hn::LoadInterleaved2(d, previous, br, bi);
  Check(d, ar);
  Check(d, ai);
  Check(d, br);
  Check(d, bi);
  // Separate product overflow checks are preserved even when the sum can cancel them.
  const auto rr = Check(d, hn::Mul(ar, br)), ii = Check(d, hn::Mul(ai, bi));
  const auto ri = Check(d, hn::Mul(ar, bi)), ir = Check(d, hn::Mul(ai, br));
#if HWY_NATIVE_FMA
  (void)rr;
  (void)ir;
  const auto real = Check(d, hn::MulAdd(ar, br, ii));
  const auto imag = Check(d, hn::NegMulAdd(ai, br, ri));
#else
  const auto real = Check(d, hn::Add(rr, ii)), imag = Check(d, hn::Sub(ri, ir));
#endif
  hn::StoreInterleaved2(real, imag, d, current);
}
bool NativeFma() { return HWY_NATIVE_FMA != 0; }
void Product(std::complex<float>* current, const std::complex<float>* previous, std::size_t count) {
  const hn::ScalableTag<float> d;
  const auto lanes = hn::Lanes(d);
  std::size_t i = 0;
  // std::complex<float> explicitly permits interleaved float array access.
  for (; count - i >= lanes; i += lanes)
    Chunk(d, reinterpret_cast<float*>(current + i), reinterpret_cast<const float*>(previous + i));
  const hn::CappedTag<float, 1> one;
  for (; i < count; ++i)
    Chunk(one, reinterpret_cast<float*>(current + i), reinterpret_cast<const float*>(previous + i));
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::estimate
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::estimate {
HWY_EXPORT(NativeFma);
bool native_fma() {
  return HWY_DYNAMIC_DISPATCH(NativeFma)();
}
HWY_EXPORT(SamplesFinite);
HWY_EXPORT(ScanRow);
void samples_finite(const float* values, std::size_t count) {
  HWY_DYNAMIC_DISPATCH(SamplesFinite)(values, count);
}
std::size_t scan_row(const float* values, std::size_t count, float& sum, float& maximum) {
  return HWY_DYNAMIC_DISPATCH(ScanRow)(values, count, sum, maximum);
}
HWY_EXPORT(Product);
void product(std::complex<float>* current, const std::complex<float>* previous, std::size_t count) {
  HWY_DYNAMIC_DISPATCH(Product)(current, previous, count);
}
} // namespace neo_mv::simd::estimate
#endif
