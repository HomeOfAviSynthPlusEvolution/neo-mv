// Included inside scalar and Highway target namespaces. No vector arrays:
// temporary storage also supports sizeless SVE/RVV vector types.
template <int N, int Full, class Op>
void Fft(const Op& op, const float* input, int step, float* re, float* im, const float* root) {
  const auto lanes = op.lanes();
  if constexpr (N == 1) {
    op.store(op.load(input), re);
    op.store(op.zero(), im);
  } else if constexpr (N == 2) {
    // Both inputs are real, and the only twiddle is exactly (1, 0).
    // Avoid the generic complex multiply and its temporary arrays.
    const auto a = op.load(input), b = op.load(input + step);
    op.store(op.add(a, b), re);
    op.store(op.sub(a, b), re + lanes);
    op.store(op.zero(), im);
    op.store(op.zero(), im + lanes);
  } else if constexpr (N == 3) {
    const auto a = op.load(input), b = op.load(input + step), c = op.load(input + 2 * step);
    const auto sum = op.add(b, c), delta = op.sub(b, c);
    const auto real = op.sub(a, op.mul(op.set(0.5f), sum));
    const auto imag = op.mul(op.set(cosine_table(6)[2]), delta);
    op.store(op.add(a, sum), re);
    op.store(op.zero(), im);
    op.store(real, re + lanes);
    op.store(op.neg(imag), im + lanes);
    op.store(real, re + 2 * lanes);
    op.store(imag, im + 2 * lanes);
  } else {
    Fft<N / 2, Full>(op, input, 2 * step, re, im, root);
    Fft<N / 2, Full>(op, input + step, 2 * step, re + (N / 2) * lanes, im + (N / 2) * lanes, root);
    for (int k = 0; k < N / 2; ++k) {
      const int index = 2 * k * (Full / N);
      const auto cr = op.set(root[index]), ci = op.set(root[Full / 2 + index]);
      const auto ar = op.load(re + k * lanes), ai = op.load(im + k * lanes);
      const auto br = op.load(re + (k + N / 2) * lanes), bi = op.load(im + (k + N / 2) * lanes);
      // Separate operations keep scalar and SIMD arithmetic comparable.
      const auto tr = op.sub(op.mul(br, cr), op.mul(bi, ci));
      const auto ti = op.add(op.mul(br, ci), op.mul(bi, cr));
      op.store(op.add(ar, tr), re + k * lanes);
      op.store(op.add(ai, ti), im + k * lanes);
      op.store(op.sub(ar, tr), re + (k + N / 2) * lanes);
      op.store(op.sub(ai, ti), im + (k + N / 2) * lanes);
    }
  }
}

template <int N, bool Normalize, class Op>
void Lines(const Op& op, const float* input, int stride, int count, float* output, float scale) {
  const int lanes = op.lanes();
  for (int j = 0; j < count; j += lanes) {
    alignas(64) float reordered[N * 8], re[N * 8], im[N * 8];
    for (int i = 0; i < N / 2; ++i) {
      op.store(op.load(input + 2 * i * stride + j), reordered + i * lanes);
      op.store(op.load(input + (2 * i + 1) * stride + j), reordered + (N - 1 - i) * lanes);
    }
    const float* root = cosine_table(N);
    Fft<N, 2 * N>(op, reordered, lanes, re, im, root);
    for (int k = 0; k < N; ++k) {
      auto value = op.sub(op.mul(op.load(re + k * lanes), op.set(root[k])),
                          op.mul(op.load(im + k * lanes), op.set(root[N + k])));
      if constexpr (Normalize)
        value = op.mul(value, op.set(scale));
      op.store(value, output + k * stride + j);
    }
  }
}
template <bool Normalize, class Op>
NEO_MV_DCT_INLINE void Matrix8(const Op& op, const float* input, float* output) {
  for (int j = 0; j < 8; j += op.lanes()) {
#if defined(__clang__)
#pragma clang loop unroll(full)
#elif defined(__GNUC__)
#pragma GCC unroll 8
#endif
    for (int k = 0; k < 8; ++k) {
      auto sum = op.zero();
#if defined(__clang__)
#pragma clang loop unroll(full)
#elif defined(__GNUC__)
#pragma GCC unroll 4
#endif
      for (int i = 0; i < 4; ++i) {
        const auto a = op.load(input + i * 8 + j), b = op.load(input + (7 - i) * 8 + j);
        const auto pair = (k & 1) ? op.sub(a, b) : op.add(a, b);
        sum = op.add(sum, op.mul(pair, op.set(Normalize ? matrix8_normalized[k][i] : matrix8[k][i])));
      }
      op.store(sum, output + k * 8 + j);
    }
  }
}
template <int N, class Op>
void Square(const Op& op, const float* input, float* rows, float* output) {
  constexpr int stride = (N + 7) & ~7;
  op.transpose(input, N, N, stride, rows, stride);
  if constexpr (N == 8)
    Matrix8<false>(op, rows, output);
  else
    Lines<N, false>(op, rows, stride, N, output, 1.0f);
  op.transpose(output, N, N, stride, rows, stride);
  if constexpr (N == 8)
    Matrix8<true>(op, rows, output);
  else
    Lines<N, true>(op, rows, stride, N, output, 0x1.6a09e6p+1f / float(N * N));
}
template <bool Normalize, class Op>
void Lines(const Op& op, int n, const float* input, int stride, int count, float* output, float scale) {
  switch (n) {
    case 2:
      return Lines<2, Normalize>(op, input, stride, count, output, scale);
    case 4:
      return Lines<4, Normalize>(op, input, stride, count, output, scale);
    case 6:
      return Lines<6, Normalize>(op, input, stride, count, output, scale);
    case 8:
      return Lines<8, Normalize>(op, input, stride, count, output, scale);
    case 12:
      return Lines<12, Normalize>(op, input, stride, count, output, scale);
    case 16:
      return Lines<16, Normalize>(op, input, stride, count, output, scale);
    case 24:
      return Lines<24, Normalize>(op, input, stride, count, output, scale);
    case 32:
      return Lines<32, Normalize>(op, input, stride, count, output, scale);
    case 48:
      return Lines<48, Normalize>(op, input, stride, count, output, scale);
    case 64:
      return Lines<64, Normalize>(op, input, stride, count, output, scale);
    case 128:
      return Lines<128, Normalize>(op, input, stride, count, output, scale);
    default:
      throw std::invalid_argument("unsupported DCT axis length");
  }
}
template <class Op>
void Transform(const Op& op, int w, int h, const float* input, float* rows, float* output) {
  if (w == h) {
    switch (w) {
      case 8:
        return Square<8>(op, input, rows, output);
      case 12:
        return Square<12>(op, input, rows, output);
      case 16:
        return Square<16>(op, input, rows, output);
      case 24:
        return Square<24>(op, input, rows, output);
      case 32:
        return Square<32>(op, input, rows, output);
      case 48:
        return Square<48>(op, input, rows, output);
      case 64:
        return Square<64>(op, input, rows, output);
      case 128:
        return Square<128>(op, input, rows, output);
    }
  }
  const int ws = padded_stride(w), hs = padded_stride(h);
  op.transpose(input, w, h, ws, rows, hs);
  Lines<false>(op, w, rows, hs, h, output, 1.0f);
  op.transpose(output, h, w, hs, rows, ws);
  Lines<true>(op, h, rows, ws, w, output, 0x1.6a09e6p+1f / float(w * h));
}
