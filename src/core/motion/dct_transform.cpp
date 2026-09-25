#include "core/motion/dct_refine.hpp"
#include "core/motion/dct_fft.hpp"

#if NEO_MV_DCT_SIMD
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "core/motion/dct_transform.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace neo_mv::dct_detail {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
#if HWY_HAVE_FLOAT64
// Vector lanes are independent 1D transforms. Intermediate arrays store
// doubles rather than vector types, also supporting scalable-vector targets.
template <int N, int Full, class D>
void Fft(D d, const double* input, int step, double* re, double* im, const double* root) {
  const auto lanes = hn::Lanes(d);
  if constexpr (N == 1) {
    hn::StoreU(hn::LoadU(d, input), d, re);
    hn::StoreU(hn::Zero(d), d, im);
  } else if constexpr (N == 3) {
    const auto a = hn::LoadU(d, input), b = hn::LoadU(d, input + step), c = hn::LoadU(d, input + 2 * step);
    const auto sum = hn::Add(b, c), delta = hn::Sub(b, c);
    const auto real = hn::Sub(a, hn::Mul(hn::Set(d, 0.5), sum));
    const auto imag = hn::Mul(hn::Set(d, cosine_table(6)[2]), delta);
    hn::StoreU(hn::Add(a, sum), d, re);
    hn::StoreU(hn::Zero(d), d, im);
    hn::StoreU(real, d, re + lanes);
    hn::StoreU(hn::Neg(imag), d, im + lanes);
    hn::StoreU(real, d, re + 2 * lanes);
    hn::StoreU(imag, d, im + 2 * lanes);
  } else {
    Fft<N / 2, Full>(d, input, 2 * step, re, im, root);
    Fft<N / 2, Full>(d, input + step, 2 * step, re + (N / 2) * lanes, im + (N / 2) * lanes, root);
    for (int k = 0; k < N / 2; ++k) {
      const int index = 2 * k * (Full / N);
      const auto cr = hn::Set(d, root[index]), ci = hn::Set(d, root[Full / 2 + index]);
      const auto ar = hn::LoadU(d, re + k * lanes), ai = hn::LoadU(d, im + k * lanes);
      const auto br = hn::LoadU(d, re + (k + N / 2) * lanes), bi = hn::LoadU(d, im + (k + N / 2) * lanes);
      // Separate operations, matching the scalar graph and its error bound.
      const auto tr = hn::Sub(hn::Mul(br, cr), hn::Mul(bi, ci));
      const auto ti = hn::Add(hn::Mul(br, ci), hn::Mul(bi, cr));
      hn::StoreU(hn::Add(ar, tr), d, re + k * lanes);
      hn::StoreU(hn::Add(ai, ti), d, im + k * lanes);
      hn::StoreU(hn::Sub(ar, tr), d, re + (k + N / 2) * lanes);
      hn::StoreU(hn::Sub(ai, ti), d, im + (k + N / 2) * lanes);
    }
  }
}
template <int N, class D>
void Lines(D d, const double* input, int step, int lane_step, double* out, int out_step, int out_lane_step) {
  const auto lanes = hn::Lanes(d);
  HWY_ALIGN double reordered[N * 4], re[N * 4], im[N * 4], temporary[4];
  for (int i = 0; i < N / 2; ++i)
    for (std::size_t lane = 0; lane < lanes; ++lane) {
      reordered[i * lanes + lane] = input[2 * i * step + lane * lane_step];
      reordered[(N - 1 - i) * lanes + lane] = input[(2 * i + 1) * step + lane * lane_step];
    }
  const double* root = cosine_table(N);
  Fft<N, 2 * N>(d, reordered, int(lanes), re, im, root);
  for (int k = 0; k < N; ++k) {
    auto value = hn::Sub(hn::Mul(hn::LoadU(d, re + k * lanes), hn::Set(d, root[k])),
                         hn::Mul(hn::LoadU(d, im + k * lanes), hn::Set(d, root[N + k])));
    value = hn::Add(value, value);
    if (out_lane_step == 1)
      hn::StoreU(value, d, out + k * out_step);
    else {
      hn::StoreU(value, d, temporary);
      for (std::size_t lane = 0; lane < lanes; ++lane)
        out[k * out_step + lane * out_lane_step] = temporary[lane];
    }
  }
}
template <class D>
void Lines(D d, int n, const double* input, int step, int lane_step, double* out, int out_step, int out_lane_step) {
  switch (n) {
    case 2:
      return Lines<2>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 4:
      return Lines<4>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 6:
      return Lines<6>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 8:
      return Lines<8>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 12:
      return Lines<12>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 16:
      return Lines<16>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 24:
      return Lines<24>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 32:
      return Lines<32>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 48:
      return Lines<48>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 64:
      return Lines<64>(d, input, step, lane_step, out, out_step, out_lane_step);
    case 128:
      return Lines<128>(d, input, step, lane_step, out, out_step, out_lane_step);
    default:
      throw std::invalid_argument("unsupported SIMD DCT axis length");
  }
}
#endif
void Transform(int width, int height, const double* input, double* rows, double* output) {
  int y = 0, u = 0;
#if HWY_HAVE_FLOAT64
  const hn::CappedTag<double, 4> d;
  const int lanes = int(hn::Lanes(d));
  for (; y + lanes <= height; y += lanes)
    Lines(d, width, input + y * width, 1, width, rows + y * width, 1, width);
#endif
  for (; y < height; ++y)
    dct_line(width, input + y * width, 1, rows + y * width, 1);
#if HWY_HAVE_FLOAT64
  for (; u + lanes <= width; u += lanes)
    Lines(d, height, rows + u, width, 1, output + u, width, 1);
#endif
  for (; u < width; ++u)
    dct_line(height, rows + u, width, output + u, width);
}
void Quantize(const std::uint16_t* samples, int width, int height, const double* transformed, int* output, int maximum,
              double error) {
  const int size = width * height;
  const double normalization = 0x1.6a09e667f3bcdp-1 / double(size);
  int i = 1;
#if HWY_HAVE_FLOAT64
  const hn::CappedTag<double, 4> d;
  const hn::Rebind<std::int32_t, decltype(d)> di;
  const int lanes = int(hn::Lanes(d));
  const auto half = hn::Set(d, .5), one = hn::Set(d, 1.0);
  const auto scale = hn::Set(d, normalization), margin = hn::Set(d, 2 * error);
  const auto midpoint = hn::Set(d, double((maximum + 1) / 2)), upper = hn::Set(d, double(maximum));
  for (; i + lanes <= size; i += lanes) {
    const auto value = hn::Mul(hn::LoadU(d, transformed + i), scale);
    const auto low = hn::Floor(value);
    const auto ambiguous = hn::Le(hn::Abs(hn::Sub(value, hn::Add(low, half))), margin);
    if (hn::AllFalse(d, ambiguous)) {
      // No lane can be a half-integer tie: those always enter refinement.
      const auto q = hn::IfThenElse(hn::Gt(hn::Sub(value, low), half), hn::Add(low, one), low);
      const auto clamped = hn::Min(upper, hn::Max(hn::Zero(d), hn::Add(q, midpoint)));
      hn::StoreU(hn::DemoteTo(di, clamped), di, output + i);
    } else {
      HWY_ALIGN double values[4];
      hn::StoreU(value, d, values);
      for (int lane = 0; lane < lanes; ++lane)
        output[i + lane] = quantize_one(samples, width, height, i + lane, values[lane], error, maximum);
    }
  }
#endif
  for (; i < size; ++i)
    output[i] = quantize_one(samples, width, height, i, transformed[i] * normalization, error, maximum);
}
} // namespace HWY_NAMESPACE
} // namespace neo_mv::dct_detail
HWY_AFTER_NAMESPACE();
#endif

#if !NEO_MV_DCT_SIMD || HWY_ONCE
namespace neo_mv::dct_detail {
#if NEO_MV_DCT_SIMD
HWY_EXPORT(Transform);
HWY_EXPORT(Quantize);
#endif
void transform_block(int width, int height, const double* input, double* rows, double* output, bool simd) {
#if NEO_MV_DCT_SIMD
  if (simd) {
    HWY_DYNAMIC_DISPATCH(Transform)(width, height, input, rows, output);
    return;
  }
#else
  (void)simd;
#endif
  for (int y = 0; y < height; ++y)
    dct_line(width, input + y * width, 1, rows + y * width, 1);
  for (int u = 0; u < width; ++u)
    dct_line(height, rows + u, width, output + u, width);
}
void quantize_ac(const std::uint16_t* samples, int width, int height, const double* transformed, int* output,
                 int maximum, double error, bool simd) {
#if NEO_MV_DCT_SIMD
  if (simd) {
    HWY_DYNAMIC_DISPATCH(Quantize)(samples, width, height, transformed, output, maximum, error);
    return;
  }
#else
  (void)simd;
#endif
  const int size = width * height;
  const double normalization = 0x1.6a09e667f3bcdp-1 / double(size);
  for (int i = 1; i < size; ++i)
    output[i] = quantize_one(samples, width, height, i, transformed[i] * normalization, error, maximum);
}
} // namespace neo_mv::dct_detail
#endif
