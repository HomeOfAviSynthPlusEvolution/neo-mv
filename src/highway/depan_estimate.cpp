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
  // Phase 6 requires each product and sum separately rounded, including
  // rejection of a non-finite product even if a later sum would cancel it.
  const auto rr = Check(d, hn::Mul(ar, br)), ii = Check(d, hn::Mul(ai, bi));
  const auto ri = Check(d, hn::Mul(ar, bi)), ir = Check(d, hn::Mul(ai, br));
  const auto real = Check(d, hn::Add(rr, ii)), imag = Check(d, hn::Sub(ri, ir));
  hn::StoreInterleaved2(real, imag, d, current);
}
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
HWY_EXPORT(Product);
void product(std::complex<float>* current, const std::complex<float>* previous, std::size_t count) {
  HWY_DYNAMIC_DISPATCH(Product)(current, previous, count);
}
} // namespace neo_mv::simd::estimate
#endif
