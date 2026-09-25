#pragma once
#include "core/motion/dct_constants.hpp"
#include <array>
namespace neo_mv::dct_detail {
struct Complex {
  double re, im;
};
inline Complex add(Complex a, Complex b) {
  return {a.re + b.re, a.im + b.im};
}
inline Complex sub(Complex a, Complex b) {
  return {a.re - b.re, a.im - b.im};
}
inline Complex mul(Complex a, Complex b) {
  return {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
}
// FFT uses only radix 2 plus a radix-3 leaf. All rotations come from the
// generated, verified cosine tables; no runtime trigonometric functions.
template <int N, int Full>
void fft(const double* input, int stride, Complex* out, const double* root) {
  static_assert(N > 0 && Full % N == 0);
  if constexpr (N == 1) {
    out[0] = {*input, 0};
  } else if constexpr (N == 3) {
    double a = input[0], b = input[stride], c = input[2 * stride];
    double total = b + c, delta = b - c;
    double real = a - 0.5 * total, imag = cosine_table(6)[2] * delta;
    out[0] = {a + total, 0};
    out[1] = {real, -imag};
    out[2] = {real, imag};
  } else {
    static_assert(N % 2 == 0);
    fft<N / 2, Full>(input, stride * 2, out, root);
    fft<N / 2, Full>(input + stride, stride * 2, out + N / 2, root);
    for (int k = 0; k < N / 2; ++k) {
      int index = 2 * k * (Full / N);
      Complex twiddle{root[index], root[Full / 2 + index]};
      auto a = out[k], b = mul(out[k + N / 2], twiddle);
      out[k] = add(a, b);
      out[k + N / 2] = sub(a, b);
    }
  }
}
template <int N>
void dct_line(const double* input, int stride, double* out, int out_stride) {
  static_assert(N % 2 == 0);
  std::array<double, N> reordered;
  std::array<Complex, N> transformed;
  for (int i = 0; i < N / 2; ++i) {
    reordered[i] = input[2 * i * stride];
    reordered[N - 1 - i] = input[(2 * i + 1) * stride];
  }
  const double* root = cosine_table(N);
  fft<N, 2 * N>(reordered.data(), 1, transformed.data(), root);
  for (int k = 0; k < N; ++k)
    out[k * out_stride] = 2 * mul(transformed[k], {root[k], root[N + k]}).re;
}
// Input and output must address n samples at positive strides, with no overlap.
// Returns the unnormalized DCT-II: 2*sum(input[x]*cos(pi*(2x+1)*k/(2n))).
inline void dct_line(int n, const double* input, int stride, double* out, int out_stride) {
  switch (n) {
  case 2:
    dct_line<2>(input, stride, out, out_stride);
    break;
  case 4:
    dct_line<4>(input, stride, out, out_stride);
    break;
  case 6:
    dct_line<6>(input, stride, out, out_stride);
    break;
  case 8:
    dct_line<8>(input, stride, out, out_stride);
    break;
  case 12:
    dct_line<12>(input, stride, out, out_stride);
    break;
  case 16:
    dct_line<16>(input, stride, out, out_stride);
    break;
  case 24:
    dct_line<24>(input, stride, out, out_stride);
    break;
  case 32:
    dct_line<32>(input, stride, out, out_stride);
    break;
  case 48:
    dct_line<48>(input, stride, out, out_stride);
    break;
  case 64:
    dct_line<64>(input, stride, out, out_stride);
    break;
  case 128:
    dct_line<128>(input, stride, out, out_stride);
    break;
  default:
    throw std::invalid_argument("unsupported certified FFT length");
  }
}

// Absolute error bound for normalized AC after two separable passes and
// multiplication by (1/sqrt(2))/area. Requires IEEE binary64 operations,
// no reassociation, no contraction, and integer samples in [0, 2^bits-1].
//
// Write u=2^-52, ell=ceil(log2(2*n)). A radix-2 stage propagates the two
// input errors and contributes <=32*u*m*M local error for m-point inputs
// bounded by M. The radix-3 leaf contributes <=16*u*3*M. Verified twiddles
// have component error <u. Induction, followed by the final rotation and
// exact factor2, gives a 1D bound 2*n*M*q_n, q_n=64*u*(ell+1).
//
// Propagation through both axes gives <=4*area*M*(q_w+q_h+q_w*q_h).
// AC normalization gives <3*M*(q_w+q_h+q_w*q_h), plus <16*M*u for
// the normalization constant, division and final multiply. The bound below
// exceeds their sum. All arithmetic is far from overflow; any subnormal
// rounding/flush errors are also dominated by the spare margin in this bound.
// It applies to this operation graph, not to arbitrary FFT implementations.
inline double ac_error_bound(int width, int height, int bits) {
  cosine_table(width);
  cosine_table(height);
  if (bits < 8 || bits > 16)
    throw std::invalid_argument("DCT requires 8-16 bit integer samples");
  int levels = 4;
  for (int n = 2 * width; n > 1; n = (n + 1) / 2)
    ++levels;
  for (int n = 2 * height; n > 1; n = (n + 1) / 2)
    ++levels;
  // Both integer factors and the power-of-two scaling are exactly represented.
  return 0x1p-44 * double((1 << bits) - 1) * double(levels);
}
} // namespace neo_mv::dct_detail
