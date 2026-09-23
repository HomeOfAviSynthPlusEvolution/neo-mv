#include "highway/render_rows.hpp"
#include <stdexcept>
#include <algorithm>
#include <type_traits>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "highway/render_rows.cpp"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"
HWY_BEFORE_NAMESPACE();
namespace neo_mv::simd::detail {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;
template <class T>
using Acc = std::conditional_t<std::is_same_v<T, float>, float, std::int32_t>;
template <class D, class V>
void CheckFinite(D d, V v) {
  if constexpr (std::is_same_v<hn::TFromD<D>, float>)
    if (!hn::AllTrue(d, hn::IsFinite(v)))
      throw std::overflow_error("non-finite render intermediate");
}
template <class D, class T>
auto LoadSample(D d, const T* p) {
  const hn::Rebind<T, D> narrow;
  if constexpr (std::is_same_v<T, float>)
    return hn::LoadU(d, p);
  else
    return hn::PromoteTo(d, hn::LoadU(narrow, p));
}
template <class D, class V, class T>
void StoreSample(D d, V v, T* p) {
  if constexpr (std::is_same_v<T, float>)
    hn::StoreU(v, d, p);
  else {
    const hn::Rebind<T, D> narrow;
    hn::StoreU(hn::DemoteTo(narrow, v), narrow, p);
  }
}
template <class D, class T>
void WeightedChunk(D d, const T* centre, const T* const* refs, const int* weights, int cw, int nr, T* out, int x) {
  using A = Acc<T>;
  auto c = LoadSample(d, centre + x);
  auto sum = hn::Mul(c, hn::Set(d, A(cw)));
  CheckFinite(d, sum);
  if constexpr (!std::is_same_v<T, float>)
    sum = hn::Add(sum, hn::Set(d, 128));
  for (int r = 0; r < nr; ++r) {
    auto sample = refs[r] ? LoadSample(d, refs[r] + x) : c;
    auto product = hn::Mul(sample, hn::Set(d, A(weights[r])));
    CheckFinite(d, product);
    sum = hn::Add(sum, product);
    CheckFinite(d, sum);
  }
  if constexpr (std::is_same_v<T, float>)
    StoreSample(d, hn::Mul(sum, hn::Set(d, 1.0f / 256)), out + x);
  else
    StoreSample(d, hn::ShiftRight<8>(sum), out + x);
}
template <class T>
void Weighted(const T* centre, const T* const* refs, const int* weights, int cw, int nr, T* out, int count) {
  const hn::ScalableTag<Acc<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    WeightedChunk(d, centre, refs, weights, cw, nr, out, x);
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    WeightedChunk(one, centre, refs, weights, cw, nr, out, x);
}
template <class D, class V>
HWY_INLINE void AddValue(D d, V sample, const std::uint16_t* coeff, hn::TFromD<D>* sum) {
  const hn::Rebind<std::uint16_t, D> dw;
  const hn::Rebind<std::int32_t, D> di;
  auto wi = hn::PromoteTo(di, hn::LoadU(dw, coeff));
  auto old = hn::LoadU(d, sum);
  if constexpr (std::is_same_v<hn::TFromD<D>, float>) {
    auto product = hn::Mul(sample, hn::ConvertTo(d, wi));
    CheckFinite(d, product);
    auto value = hn::Add(old, hn::Mul(product, hn::Set(d, 1.0f / 64)));
    CheckFinite(d, value);
    hn::StoreU(value, d, sum);
  } else
    hn::StoreU(hn::Add(old, hn::ShiftRight<6>(hn::Mul(sample, wi))), d, sum);
}
template <class D, class T>
void AddChunk(D d, const T* src, const std::uint16_t* coeff, Acc<T>* sum) {
  AddValue(d, LoadSample(d, src), coeff, sum);
}

template <class T>
void Add(const T* src, const std::uint16_t* coeff, Acc<T>* sum, int count) {
  const hn::ScalableTag<Acc<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    AddChunk(d, src + x, coeff + x, sum + x);
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    AddChunk(one, src + x, coeff + x, sum + x);
}
template <class D, class T>
void FinishChunk(D d, const Acc<T>* sum, T* out, std::int64_t maximum) {
  auto v = hn::LoadU(d, sum);
  if constexpr (std::is_same_v<T, float>)
    StoreSample(d, hn::Mul(v, hn::Set(d, 1.0f / 32)), out);
  else
    StoreSample(d, hn::Min(hn::ShiftRight<5>(hn::Add(v, hn::Set(d, 16))), hn::Set(d, std::int32_t(maximum))), out);
}
template <class T>
void Finish(const Acc<T>* sum, T* out, int count, std::int64_t maximum) {
  const hn::ScalableTag<Acc<T>> d;
  const int n = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - n; x += n)
    FinishChunk(d, sum + x, out + x, maximum);
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    FinishChunk(one, sum + x, out + x, maximum);
}
// Run the whole plane under one target dispatch. Short chroma rows use
// capped vectors rather than falling through a full-width loop to scalars.
template <class T>
HWY_INLINE void AddShort(const T* src, const std::uint16_t* coeff, Acc<T>* sum, int count) {
  const hn::ScalableTag<Acc<T>> full;
  const int lanes = int(hn::Lanes(full));
  int x = 0;
  for (; x <= count - lanes; x += lanes)
    AddChunk(full, src + x, coeff + x, sum + x);
  const hn::CappedTag<Acc<T>, 8> eight;
  const int n8 = int(hn::Lanes(eight));
  if (x <= count - n8) {
    AddChunk(eight, src + x, coeff + x, sum + x);
    x += n8;
  }
  const hn::CappedTag<Acc<T>, 4> four;
  const int n4 = int(hn::Lanes(four));
  if (x <= count - n4) {
    AddChunk(four, src + x, coeff + x, sum + x);
    x += n4;
  }
  const hn::CappedTag<Acc<T>, 1> one;
  for (; x < count; ++x)
    AddChunk(one, src + x, coeff + x, sum + x);
}

template <class T>
struct DirectInput {
  const T* samples;
  const std::uint16_t* coefficients;
};
template <class T>
using DirectAcc = std::conditional_t<std::is_same_v<T, std::uint8_t>, std::uint16_t, Acc<T>>;

template <int N, class D, class T>
HWY_INLINE void ComposeDirectChunk(D d, const DirectInput<T>* inputs, int x, T* output, std::int64_t maximum) {
  auto sum = hn::Zero(d);
  for (int i = 0; i < N; ++i) {
    if constexpr (std::is_same_v<T, std::uint8_t>) {
      // The plan's coefficients are <= 2048. Four contributions plus the
      // rounding bias fit uint16, and high-multiply keeps the exact >> 6.
      const hn::Rebind<std::uint8_t, D> bytes;
      const auto sample = hn::PromoteTo(d, hn::LoadU(bytes, inputs[i].samples + x));
      const auto weight = hn::LoadU(d, inputs[i].coefficients + x);
      sum = hn::Add(sum, hn::MulHigh(hn::ShiftLeft<8>(sample), hn::ShiftLeft<2>(weight)));
    } else {
      const auto sample = LoadSample(d, inputs[i].samples + x);
      const hn::Rebind<std::uint16_t, D> narrow;
      const hn::Rebind<std::int32_t, D> wide;
      const auto weight = hn::PromoteTo(wide, hn::LoadU(narrow, inputs[i].coefficients + x));
      sum = hn::Add(sum, hn::ShiftRight<6>(hn::Mul(sample, weight)));
    }
  }
  const auto rounded = hn::ShiftRight<5>(hn::Add(sum, hn::Set(d, 16)));
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    const hn::Rebind<std::uint8_t, D> bytes;
    hn::StoreU(hn::DemoteTo(bytes, hn::Min(rounded, hn::Set(d, std::uint16_t(maximum)))), bytes, output + x);
  } else
    StoreSample(d, hn::Min(rounded, hn::Set(d, std::int32_t(maximum))), output + x);
}

template <int N, class T>
HWY_INLINE void ComposeDirectSegment(const DirectInput<T>* inputs, int count, T* output, std::int64_t maximum) {
  const hn::CappedTag<DirectAcc<T>, 8> eight;
  if (count == 8 && hn::Lanes(eight) == 8) {
    ComposeDirectChunk<N>(eight, inputs, 0, output, maximum);
    return;
  }
  const hn::ScalableTag<DirectAcc<T>> full;
  const int lanes = int(hn::Lanes(full));
  int x = 0;
  for (; x <= count - lanes; x += lanes)
    ComposeDirectChunk<N>(full, inputs, x, output, maximum);
  const int n8 = int(hn::Lanes(eight));
  if (x <= count - n8) {
    ComposeDirectChunk<N>(eight, inputs, x, output, maximum);
    x += n8;
  }
  const hn::CappedTag<DirectAcc<T>, 4> four;
  const int n4 = int(hn::Lanes(four));
  if (x <= count - n4) {
    ComposeDirectChunk<N>(four, inputs, x, output, maximum);
    x += n4;
  }
  const hn::CappedTag<DirectAcc<T>, 1> one;
  for (; x < count; ++x)
    ComposeDirectChunk<N>(one, inputs, x, output, maximum);
}
#if HWY_TARGET == HWY_AVX2
void ComposeDirectPairByte(const BlockCompositionGeometry& g, const SampledRenderBlock<std::uint8_t>* blocks,
                           span2d::Plane<std::uint8_t> output, int tx, int ty, int ox, int oy,
                           std::int64_t maximum) {
  DirectInput<std::uint8_t> input[4][2];
  std::ptrdiff_t strides[4][2];
  int n = 0;
  for (int by = ty - 1; by <= ty; ++by)
    for (int hx = 0; hx < 2; ++hx) {
      const int local_y = by == ty ? 0 : 8;
      const int local_x = hx ? 0 : 8;
      for (int half = 0; half < 2; ++half) {
        const int bx = tx - 1 + hx + half;
        const auto& block = blocks[std::size_t(by) * g.blocks_x + bx];
        input[n][half] = {block.data + local_y * block.stride + local_x,
                          block.coefficients + std::size_t(local_y) * 16 + local_x};
        strides[n][half] = block.stride;
      }
      ++n;
    }
  const hn::CappedTag<std::uint16_t, 16> full;
  const hn::CappedTag<std::uint16_t, 8> half;
  const hn::Rebind<std::uint8_t, decltype(half)> half_bytes;
  const hn::Rebind<std::uint8_t, decltype(full)> full_bytes;
  auto* dst = output.row(oy).data() + ox;
  for (int row = 0; row < 8; ++row) {
    auto sum = hn::Zero(full);
    for (int i = 0; i < 4; ++i) {
      const auto lo = hn::PromoteTo(half, hn::LoadU(half_bytes, input[i][0].samples));
      const auto hi = hn::PromoteTo(half, hn::LoadU(half_bytes, input[i][1].samples));
      const auto sample = hn::Combine(full, hi, lo);
      const auto weight = hn::Combine(full, hn::LoadU(half, input[i][1].coefficients),
                                     hn::LoadU(half, input[i][0].coefficients));
      sum = hn::Add(sum, hn::MulHigh(hn::ShiftLeft<8>(sample), hn::ShiftLeft<2>(weight)));
    }
    const auto rounded = hn::ShiftRight<5>(hn::Add(sum, hn::Set(full, std::uint16_t(16))));
    hn::StoreU(hn::DemoteTo(full_bytes, hn::Min(rounded, hn::Set(full, std::uint16_t(maximum)))), full_bytes, dst);
    if (row + 1 < 8) {
      dst += output.stride();
      for (int i = 0; i < 4; ++i)
        for (int side = 0; side < 2; ++side) {
          input[i][side].samples += strides[i][side];
          input[i][side].coefficients += 16;
        }
    }
  }
}
#endif
#if HWY_TARGET == HWY_AVX3_SPR
void ComposeDirectQuadByte(const BlockCompositionGeometry& g, const SampledRenderBlock<std::uint8_t>* blocks,
                           span2d::Plane<std::uint8_t> output, int tx, int ty, int ox, int oy,
                           std::int64_t maximum) {
  DirectInput<std::uint8_t> input[4][4];
  std::ptrdiff_t strides[4][4];
  int n = 0;
  for (int by = ty - 1; by <= ty; ++by)
    for (int hx = 0; hx < 2; ++hx) {
      const int local_y = by == ty ? 0 : 8;
      const int local_x = hx ? 0 : 8;
      for (int tile = 0; tile < 4; ++tile) {
        const int bx = tx - 1 + hx + tile;
        const auto& block = blocks[std::size_t(by) * g.blocks_x + bx];
        input[n][tile] = {block.data + local_y * block.stride + local_x,
                          block.coefficients + std::size_t(local_y) * 16 + local_x};
        strides[n][tile] = block.stride;
      }
      ++n;
    }
  const hn::CappedTag<std::uint16_t, 32> full;
  const hn::CappedTag<std::uint16_t, 16> half;
  const hn::CappedTag<std::uint16_t, 8> quarter;
  const hn::Rebind<std::uint8_t, decltype(quarter)> quarter_bytes;
  const hn::Rebind<std::uint8_t, decltype(full)> full_bytes;
  auto* dst = output.row(oy).data() + ox;
  for (int row = 0; row < 8; ++row) {
    auto sum = hn::Zero(full);
    for (int i = 0; i < 4; ++i) {
      const auto sample = [&](int tile) HWY_ATTR {
        return hn::PromoteTo(quarter, hn::LoadU(quarter_bytes, input[i][tile].samples));
      };
      const auto lo = hn::Combine(half, sample(1), sample(0));
      const auto hi = hn::Combine(half, sample(3), sample(2));
      const auto samples = hn::Combine(full, hi, lo);
      const auto weight_lo = hn::Combine(half, hn::LoadU(quarter, input[i][1].coefficients),
                                         hn::LoadU(quarter, input[i][0].coefficients));
      const auto weight_hi = hn::Combine(half, hn::LoadU(quarter, input[i][3].coefficients),
                                         hn::LoadU(quarter, input[i][2].coefficients));
      const auto weight = hn::Combine(full, weight_hi, weight_lo);
      sum = hn::Add(sum, hn::MulHigh(hn::ShiftLeft<8>(samples), hn::ShiftLeft<2>(weight)));
    }
    const auto rounded = hn::ShiftRight<5>(hn::Add(sum, hn::Set(full, std::uint16_t(16))));
    hn::StoreU(hn::DemoteTo(full_bytes, hn::Min(rounded, hn::Set(full, std::uint16_t(maximum)))), full_bytes, dst);
    if (row + 1 < 8) {
      dst += output.stride();
      for (int i = 0; i < 4; ++i)
        for (int tile = 0; tile < 4; ++tile) {
          input[i][tile].samples += strides[i][tile];
          input[i][tile].coefficients += 16;
        }
    }
  }
}
#endif

#if HWY_TARGET == HWY_AVX2 || HWY_TARGET == HWY_AVX3_SPR
template <int Tiles>
void ComposeDirectPackedChromaByte(const BlockCompositionGeometry& g,
                                   const SampledRenderBlock<std::uint8_t>* blocks,
                                   span2d::Plane<std::uint8_t> output, int tx, int ty, int ox, int oy,
                                   std::int64_t maximum) {
  static_assert(Tiles == 4 || Tiles == 8);
  DirectInput<std::uint8_t> input[4][Tiles];
  std::ptrdiff_t strides[4][Tiles];
  int n = 0;
  for (int by = ty - 1; by <= ty; ++by)
    for (int hx = 0; hx < 2; ++hx) {
      const int local_y = by == ty ? 0 : 4;
      const int local_x = hx ? 0 : 4;
      for (int tile = 0; tile < Tiles; ++tile) {
        const int bx = tx - 1 + hx + tile;
        const auto& block = blocks[std::size_t(by) * g.blocks_x + bx];
        input[n][tile] = {block.data + local_y * block.stride + local_x,
                          block.coefficients + std::size_t(local_y) * 8 + local_x};
        strides[n][tile] = block.stride;
      }
      ++n;
    }
  const hn::CappedTag<std::uint16_t, 4> four;
  const hn::CappedTag<std::uint16_t, 8> eight;
  const hn::CappedTag<std::uint16_t, 16> sixteen;
  const hn::CappedTag<std::uint16_t, 4 * Tiles> full;
  const hn::Rebind<std::uint8_t, decltype(four)> four_bytes;
  const hn::Rebind<std::uint8_t, decltype(full)> full_bytes;
  auto* dst = output.row(oy).data() + ox;
  for (int row = 0; row < 4; ++row) {
    auto sum = hn::Zero(full);
    for (int i = 0; i < 4; ++i) {
      const auto sample = [&](int tile) HWY_ATTR {
        return hn::PromoteTo(four, hn::LoadU(four_bytes, input[i][tile].samples));
      };
      const auto weight = [&](int tile) HWY_ATTR { return hn::LoadU(four, input[i][tile].coefficients); };
      const auto pack4 = [&](int first, auto load) HWY_ATTR {
        const auto lo = hn::Combine(eight, load(first + 1), load(first));
        const auto hi = hn::Combine(eight, load(first + 3), load(first + 2));
        return hn::Combine(sixteen, hi, lo);
      };
      const auto pack = [&](auto load) HWY_ATTR {
        if constexpr (Tiles == 8)
          return hn::Combine(full, pack4(4, load), pack4(0, load));
        else
          return pack4(0, load);
      };
      const auto samples = pack(sample);
      const auto weights = pack(weight);
      sum = hn::Add(sum, hn::MulHigh(hn::ShiftLeft<8>(samples), hn::ShiftLeft<2>(weights)));
    }
    const auto rounded = hn::ShiftRight<5>(hn::Add(sum, hn::Set(full, std::uint16_t(16))));
    hn::StoreU(hn::DemoteTo(full_bytes, hn::Min(rounded, hn::Set(full, std::uint16_t(maximum)))), full_bytes, dst);
    if (row + 1 < 4) {
      dst += output.stride();
      for (int i = 0; i < 4; ++i)
        for (int tile = 0; tile < Tiles; ++tile) {
          input[i][tile].samples += strides[i][tile];
          input[i][tile].coefficients += 8;
        }
    }
  }
}

template <int Tiles>
void ComposeDirectPackedChromaWord(const BlockCompositionGeometry& g,
                                   const SampledRenderBlock<std::uint16_t>* blocks,
                                   span2d::Plane<std::uint16_t> output, int tx, int ty, int ox, int oy,
                                   std::int64_t maximum) {
  static_assert(Tiles == 2 || Tiles == 4);
  DirectInput<std::uint16_t> input[4][Tiles];
  std::ptrdiff_t strides[4][Tiles];
  int n = 0;
  for (int by = ty - 1; by <= ty; ++by)
    for (int hx = 0; hx < 2; ++hx) {
      const int local_y = by == ty ? 0 : 4;
      const int local_x = hx ? 0 : 4;
      for (int tile = 0; tile < Tiles; ++tile) {
        const int bx = tx - 1 + hx + tile;
        const auto& block = blocks[std::size_t(by) * g.blocks_x + bx];
        input[n][tile] = {block.data + local_y * block.stride + local_x,
                          block.coefficients + std::size_t(local_y) * 8 + local_x};
        strides[n][tile] = block.stride;
      }
      ++n;
    }
  const hn::CappedTag<std::int32_t, 4> four;
  const hn::CappedTag<std::int32_t, 8> eight;
  const hn::CappedTag<std::int32_t, 4 * Tiles> full;
  const hn::Rebind<std::uint16_t, decltype(four)> four_words;
  auto* dst = output.row(oy).data() + ox;
  for (int row = 0; row < 4; ++row) {
    auto sum = hn::Zero(full);
    for (int i = 0; i < 4; ++i) {
      const auto load_sample = [&](int tile) HWY_ATTR {
        return hn::PromoteTo(four, hn::LoadU(four_words, input[i][tile].samples));
      };
      const auto load_weight = [&](int tile) HWY_ATTR {
        return hn::PromoteTo(four, hn::LoadU(four_words, input[i][tile].coefficients));
      };
      const auto pack = [&](auto load) HWY_ATTR {
        const auto lo = hn::Combine(eight, load(1), load(0));
        if constexpr (Tiles == 4)
          return hn::Combine(full, hn::Combine(eight, load(3), load(2)), lo);
        else
          return lo;
      };
      const auto samples = pack(load_sample);
      const auto weights = pack(load_weight);
      sum = hn::Add(sum, hn::ShiftRight<6>(hn::Mul(samples, weights)));
    }
    const auto rounded = hn::ShiftRight<5>(hn::Add(sum, hn::Set(full, 16)));
    StoreSample(full, hn::Min(rounded, hn::Set(full, std::int32_t(maximum))), dst);
    if (row + 1 < 4) {
      dst += output.stride();
      for (int i = 0; i < 4; ++i)
        for (int tile = 0; tile < Tiles; ++tile) {
          input[i][tile].samples += strides[i][tile];
          input[i][tile].coefficients += 8;
        }
    }
  }
}

#endif

template <class T, int Half>
void ComposeDirectTiledHalf(const BlockCompositionGeometry& g, const SampledRenderBlock<T>* blocks,
                            span2d::Plane<T> output, std::int64_t maximum) {
  for (int ty = 0, oy = 0; oy < g.visible_height; ++ty, oy += Half) {
    const int tile_height = std::min(Half, g.visible_height - oy);
    const int first_y = ty ? ty - 1 : 0;
    const int last_y = std::min(ty, g.blocks_y - 1);
    for (int tx = 0, ox = 0; ox < g.visible_width; ++tx, ox += Half) {
#if HWY_TARGET == HWY_AVX3_SPR
      if constexpr (std::is_same_v<T, std::uint16_t> && Half == 4) {
        if (ty > 0 && ty < g.blocks_y && tx > 0 && tx + 3 < g.blocks_x && tile_height == 4 &&
            ox + 16 <= g.visible_width) {
          ComposeDirectPackedChromaWord<4>(g, blocks, output, tx, ty, ox, oy, maximum);
          tx += 3;
          ox += 12;
          continue;
        }
      }
      if constexpr (std::is_same_v<T, std::uint8_t> && Half == 4) {
        if (ty > 0 && ty < g.blocks_y && tx > 0 && tx + 7 < g.blocks_x && tile_height == 4 &&
            ox + 32 <= g.visible_width) {
          ComposeDirectPackedChromaByte<8>(g, blocks, output, tx, ty, ox, oy, maximum);
          tx += 7;
          ox += 28;
          continue;
        }
      }
      if constexpr (std::is_same_v<T, std::uint8_t> && Half == 8) {
        if (ty > 0 && ty < g.blocks_y && tx > 0 && tx + 3 < g.blocks_x && tile_height == 8 &&
            ox + 32 <= g.visible_width) {
          ComposeDirectQuadByte(g, blocks, output, tx, ty, ox, oy, maximum);
          tx += 3;
          ox += 24;
          continue;
        }
      }
#endif
#if HWY_TARGET == HWY_AVX2
      if constexpr (std::is_same_v<T, std::uint16_t> && Half == 4) {
        if (ty > 0 && ty < g.blocks_y && tx > 0 && tx + 1 < g.blocks_x && tile_height == 4 &&
            ox + 8 <= g.visible_width) {
          ComposeDirectPackedChromaWord<2>(g, blocks, output, tx, ty, ox, oy, maximum);
          ++tx;
          ox += 4;
          continue;
        }
      }
      if constexpr (std::is_same_v<T, std::uint8_t> && Half == 4) {
        if (ty > 0 && ty < g.blocks_y && tx > 0 && tx + 3 < g.blocks_x && tile_height == 4 &&
            ox + 16 <= g.visible_width) {
          ComposeDirectPackedChromaByte<4>(g, blocks, output, tx, ty, ox, oy, maximum);
          tx += 3;
          ox += 12;
          continue;
        }
      }
      if constexpr (std::is_same_v<T, std::uint8_t> && Half == 8) {
        if (ty > 0 && ty < g.blocks_y && tx > 0 && tx + 1 < g.blocks_x && tile_height == 8 &&
            ox + 16 <= g.visible_width) {
          ComposeDirectPairByte(g, blocks, output, tx, ty, ox, oy, maximum);
          ++tx;
          ox += 8;
          continue;
        }
      }
#endif
      const int tile_width = std::min(Half, g.visible_width - ox);
      const int first_x = tx ? tx - 1 : 0;
      const int last_x = std::min(tx, g.blocks_x - 1);
      DirectInput<T> inputs[4];
      std::ptrdiff_t strides[4];
      int n = 0;
      for (int by = first_y; by <= last_y; ++by)
        for (int bx = first_x; bx <= last_x; ++bx) {
          const auto& block = blocks[std::size_t(by) * g.blocks_x + bx];
          const int local_y = by == ty ? 0 : Half;
          const int local_x = bx == tx ? 0 : Half;
          inputs[n] = {block.data + local_y * block.stride + local_x,
                       block.coefficients + std::size_t(local_y) * g.block_width + local_x};
          strides[n++] = block.stride;
        }
      auto* dst = output.row(oy).data() + ox;
      for (int row = 0; row < tile_height; ++row) {
        if (n == 1)
          ComposeDirectSegment<1>(inputs, tile_width, dst, maximum);
        else if (n == 2)
          ComposeDirectSegment<2>(inputs, tile_width, dst, maximum);
        else
          ComposeDirectSegment<4>(inputs, tile_width, dst, maximum);
        if (row + 1 < tile_height) {
          dst += output.stride();
          for (int i = 0; i < n; ++i) {
            inputs[i].samples += strides[i];
            inputs[i].coefficients += g.block_width;
          }
        }
      }
    }
  }
}

template <class T>
void ComposeDirectInteger(const BlockCompositionGeometry& g, const SampledRenderBlock<T>* blocks,
                          span2d::Plane<T> output, std::int64_t maximum) {
  if constexpr (std::is_same_v<T, std::uint16_t>) {
    if (g.block_width == 16 && g.block_height == 16 && g.overlap_x == 8 && g.overlap_y == 8)
      return ComposeDirectTiledHalf<T, 8>(g, blocks, output, maximum);
    if (g.block_width == 8 && g.block_height == 8 && g.overlap_x == 4 && g.overlap_y == 4)
      return ComposeDirectTiledHalf<T, 4>(g, blocks, output, maximum);
  }
#if HWY_TARGET == HWY_AVX2
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    if (g.block_width == 16 && g.block_height == 16 && g.overlap_x == 8 && g.overlap_y == 8)
      return ComposeDirectTiledHalf<T, 8>(g, blocks, output, maximum);
    if (g.block_width == 8 && g.block_height == 8 && g.overlap_x == 4 && g.overlap_y == 4)
      return ComposeDirectTiledHalf<T, 4>(g, blocks, output, maximum);
  }
#endif
#if HWY_TARGET == HWY_AVX3_SPR
  if constexpr (std::is_same_v<T, std::uint8_t>) {
    if (g.block_width == 16 && g.block_height == 16 && g.overlap_x == 8 && g.overlap_y == 8)
      return ComposeDirectTiledHalf<T, 8>(g, blocks, output, maximum);
    if (g.block_width == 8 && g.block_height == 8 && g.overlap_x == 4 && g.overlap_y == 4)
      return ComposeDirectTiledHalf<T, 4>(g, blocks, output, maximum);
  }
#endif
  const int sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
  for (int y = 0; y < g.visible_height; ++y) {
    const int first = y < g.block_height ? 0 : (y - g.block_height) / sy + 1;
    const int last = std::min(y / sy, g.blocks_y - 1);
    auto* dst = output.row(y).data();
    for (int bx = 0, ox = 0; bx < g.blocks_x && ox < g.visible_width; ++bx, ox += sx) {
      const int stripe = std::min(bx == g.blocks_x - 1 ? g.block_width : sx, g.visible_width - ox);
      const int left = bx ? std::min(g.overlap_x, stripe) : 0;
      for (int segment = 0; segment < 2; ++segment) {
        const int start = segment ? left : 0;
        const int count = segment ? stripe - left : left;
        if (!count)
          continue;
        const bool previous = segment == 0;
        DirectInput<T> inputs[4];
        int n = 0;
        for (int by = first; by <= last; ++by) {
          const int ly = y - by * sy;
          const auto add = [&](int column, int local_x) {
            const auto& b = blocks[std::size_t(by) * g.blocks_x + column];
            inputs[n++] = {b.data + ly * b.stride + local_x,
                           b.coefficients + std::size_t(ly) * g.block_width + local_x};
          };
          if (previous)
            add(bx - 1, sx + start);
          add(bx, start);
        }
        if (n == 1)
          ComposeDirectSegment<1>(inputs, count, dst + ox + start, maximum);
        else if (n == 2)
          ComposeDirectSegment<2>(inputs, count, dst + ox + start, maximum);
        else
          ComposeDirectSegment<4>(inputs, count, dst + ox + start, maximum);
      }
    }
  }
}

template <class T, int BlockWidth = 0>
void ComposeSampled(const BlockCompositionGeometry& g, const SampledRenderBlock<T>* blocks, span2d::Plane<T> output,
                    std::int64_t maximum) {
  // Validate the complete selected blocks, including cropped-away samples.
  // Full-range integer storage cannot contain an out-of-range sample.
  const bool scan = std::is_same_v<T, float> || maximum < std::numeric_limits<T>::max();
  if (scan) {
    const hn::CappedTag<T, 8> d;
    const int lanes = int(hn::Lanes(d));
    for (std::size_t i = 0; i < std::size_t(g.blocks_x) * g.blocks_y; ++i)
      for (int y = 0; y < g.block_height; ++y) {
        const auto* row = blocks[i].data + y * blocks[i].stride;
        int x = 0;
        for (; x + lanes <= g.block_width; x += lanes) {
          const auto v = hn::LoadU(d, row + x);
          if constexpr (std::is_same_v<T, float>) {
            if (!hn::AllTrue(d, hn::IsFinite(v)))
              throw std::invalid_argument("non-finite compensation sample");
          } else if (!hn::AllTrue(d, hn::Le(v, hn::Set(d, T(maximum)))))
            throw std::invalid_argument("compensation sample exceeds bit depth");
        }
        for (; x < g.block_width; ++x)
          subpixel_detail::valid_sample(row[x], maximum);
      }
  }
  const int sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
  const bool overlap = g.overlap_x || g.overlap_y;
  if constexpr (!std::is_same_v<T, float>) {
    if (overlap) {
      ComposeDirectInteger(g, blocks, output, maximum);
      return;
    }
  }
  std::vector<Acc<T>> sums(overlap ? g.visible_width : 0);
  for (int y = 0; y < g.visible_height; ++y) {
    auto* dst = output.row(y).data();
    if (!overlap) {
      for (int bx = 0, x = 0; x < g.visible_width; ++bx, x += sx) {
        const auto& b = blocks[std::size_t(y / sy) * g.blocks_x + bx];
        std::copy_n(b.data + (y % sy) * b.stride, std::min(sx, g.visible_width - x), dst + x);
      }
      continue;
    }
    std::fill(sums.begin(), sums.end(), Acc<T>(0));
    const int first = y < g.block_height ? 0 : (y - g.block_height) / sy + 1;
    const int last = std::min(y / sy, g.blocks_y - 1);
    // Same block-row/column accumulation order and rounding as the reference.
    for (int by = first; by <= last; ++by) {
      const int ly = y - by * sy;
      for (int bx = 0, x = 0; bx < g.blocks_x && x < g.visible_width; ++bx, x += sx) {
        const auto& b = blocks[std::size_t(by) * g.blocks_x + bx];
        const auto* src = b.data + ly * b.stride;
        const auto* weights = b.coefficients + std::size_t(ly) * g.block_width;
        const int count = std::min(g.block_width, g.visible_width - x);
        if constexpr (BlockWidth != 0) {
          if (count == BlockWidth) {
            const hn::CappedTag<Acc<T>, BlockWidth> d;
            const int lanes = int(hn::Lanes(d));
            int offset = 0;
            for (; offset + lanes <= BlockWidth; offset += lanes)
              AddChunk(d, src + offset, weights + offset, sums.data() + x + offset);
            if (offset < BlockWidth)
              AddShort(src + offset, weights + offset, sums.data() + x + offset, BlockWidth - offset);
          } else
            AddShort(src, weights, sums.data() + x, count);
        } else
          AddShort(src, weights, sums.data() + x, count);
      }
    }
    Finish(sums.data(), dst, g.visible_width, maximum);
  }
}

template <class D, class T>
void LimitChunk(D d, const T* q, const T* centre, T* out, std::int64_t maximum, std::int64_t limit, float flimit) {
  const auto c = LoadSample(d, centre), v = LoadSample(d, q);
  if constexpr (std::is_same_v<T, float>) {
    const auto lo = hn::Sub(c, hn::Set(d, flimit)), hi = hn::Add(c, hn::Set(d, flimit));
    CheckFinite(d, lo);
    CheckFinite(d, hi);
    // Ordered comparisons preserve the input zero's sign when it is in range.
    StoreSample(d, hn::IfThenElse(hn::Lt(v, lo), lo, hn::IfThenElse(hn::Gt(v, hi), hi, v)), out);
  } else {
    const auto lo = hn::Max(hn::Sub(c, hn::Set(d, std::int32_t(limit))), hn::Zero(d));
    const auto hi = hn::Min(hn::Add(c, hn::Set(d, std::int32_t(limit))), hn::Set(d, std::int32_t(maximum)));
    StoreSample(d, hn::Min(hi, hn::Max(lo, v)), out);
  }
}
template <class T>
void Limit(const T* q, const T* centre, T* out, int count, bool active, std::int64_t maximum, std::int64_t limit,
           float flimit) {
  if (!active) {
    const hn::ScalableTag<T> d;
    const int n = int(hn::Lanes(d));
    int x = 0;
    for (; x <= count - n; x += n)
      hn::StoreU(hn::LoadU(d, q + x), d, out + x);
    for (; x < count; ++x)
      out[x] = q[x];
    return;
  }
  if constexpr (!std::is_same_v<T, float>) {
    const hn::ScalableTag<T> d;
    const int n = int(hn::Lanes(d));
    int x = 0;
    const auto amount = hn::Set(d, T(limit)), max = hn::Set(d, T(maximum));
    for (; x <= count - n; x += n) {
      const auto c = hn::LoadU(d, centre + x), v = hn::LoadU(d, q + x);
      const auto lo = hn::SaturatedSub(c, amount), hi = hn::Min(max, hn::SaturatedAdd(c, amount));
      hn::StoreU(hn::Min(hi, hn::Max(lo, v)), d, out + x);
    }
    for (; x < count; ++x) {
      const auto lo = std::max<std::int64_t>(0, std::int64_t(centre[x]) - limit);
      const auto hi = std::min(maximum, std::int64_t(centre[x]) + limit);
      out[x] = T(std::min(hi, std::max(lo, std::int64_t(q[x]))));
    }
  } else {
    const hn::ScalableTag<float> d;
    const int n = int(hn::Lanes(d));
    int x = 0;
    for (; x <= count - n; x += n)
      LimitChunk(d, q + x, centre + x, out + x, maximum, limit, flimit);
    const hn::CappedTag<float, 1> one;
    for (; x < count; ++x)
      LimitChunk(one, q + x, centre + x, out + x, maximum, limit, flimit);
  }
}
template <class D, class V>
HWY_INLINE void AdmitSample(D d, V value, std::int64_t maximum) {
  if constexpr (std::is_same_v<hn::TFromD<D>, float>) {
    if (!hn::AllTrue(d, hn::IsFinite(value)))
      throw std::invalid_argument("non-finite Degrain sample");
  } else if (!hn::AllTrue(d, hn::Le(value, hn::Set(d, hn::TFromD<D>(maximum)))))
    throw std::invalid_argument("Degrain sample exceeds bit depth");
}

template <int FixedReferences = 0, bool FullRange = false, class D, class T>
HWY_INLINE auto DegrainValue(D d, const SampledRenderBlock<T>* sources, const int* weights, int nr, int row, int x,
                             std::int64_t maximum) {
  const auto c = LoadSample(d, sources[0].data + row * sources[0].stride + x);
  // Full-width integer samples are admitted by their storage representation.
  if constexpr (std::is_same_v<T, float>)
    AdmitSample(d, c, maximum);
  else if constexpr (!FullRange)
    AdmitSample(d, c, maximum);
  using A = hn::TFromD<D>;
  auto sum = hn::Mul(c, hn::Set(d, A(weights[0])));
  CheckFinite(d, sum);
  if constexpr (!std::is_same_v<T, float>)
    sum = hn::Add(sum, hn::Set(d, 128));
  for (int r = 1; r <= (FixedReferences ? FixedReferences : nr); ++r) {
    const auto sample = sources[r].data ? LoadSample(d, sources[r].data + row * sources[r].stride + x) : c;
    if constexpr (std::is_same_v<T, float>)
      AdmitSample(d, sample, maximum);
    else if constexpr (!FullRange)
      AdmitSample(d, sample, maximum);
    const auto product = hn::Mul(sample, hn::Set(d, A(weights[r])));
    CheckFinite(d, product);
    sum = hn::Add(sum, product);
    CheckFinite(d, sum);
  }
  if constexpr (std::is_same_v<T, float>)
    return hn::Mul(sum, hn::Set(d, 1.0f / 256));
  else
    return hn::ShiftRight<8>(sum);
}
template <class D, class V>
HWY_INLINE void AddDegrainByte(D d, V sample, const std::uint16_t* coeff, std::uint16_t* sum) {
  const auto weight = hn::LoadU(d, coeff);
  const auto value = hn::MulHigh(hn::ShiftLeft<8>(sample), hn::ShiftLeft<2>(weight));
  hn::StoreU(hn::Add(hn::LoadU(d, sum), value), d, sum);
}
template <class D>
HWY_INLINE void FinishDegrainByteChunk(D d, const std::uint16_t* sum, std::uint8_t* out, std::uint16_t maximum) {
  const hn::Rebind<std::uint8_t, D> bytes;
  const auto value = hn::Min(hn::ShiftRight<5>(hn::Add(hn::LoadU(d, sum), hn::Set(d, std::uint16_t(16)))),
                             hn::Set(d, maximum));
  hn::StoreU(hn::DemoteTo(bytes, value), bytes, out);
}
void FinishDegrainByte(const std::uint16_t* sum, std::uint8_t* out, int count, std::uint16_t maximum) {
  const hn::ScalableTag<std::uint16_t> d;
  const int lanes = int(hn::Lanes(d));
  int x = 0;
  for (; x <= count - lanes; x += lanes)
    FinishDegrainByteChunk(d, sum + x, out + x, maximum);
  const hn::CappedTag<std::uint16_t, 1> one;
  for (; x < count; ++x)
    FinishDegrainByteChunk(one, sum + x, out + x, maximum);
}

template <class T, int Width, int FixedReferences = 0, bool FullRange = false>
void DegrainPlaneRun(const BlockCompositionGeometry& g, const DegrainPlane<T>& p, span2d::Plane<const T> centre,
                     span2d::Plane<T> out, const ChangeLimit<T>& limit) {
  using DegrainAcc = std::conditional_t<std::is_same_v<T, std::uint8_t>, std::uint16_t, Acc<T>>;
  const int references = FixedReferences ? FixedReferences : p.references;
  const int sx = g.block_width - g.overlap_x, sy = g.block_height - g.overlap_y;
  const bool overlap = g.overlap_x || g.overlap_y;
  std::vector<DegrainAcc> sums(overlap ? g.visible_width : 0);
  const int height = (g.blocks_y - 1) * sy + g.block_height;
  const hn::CappedTag<DegrainAcc, Width> d;
  const int lanes = int(hn::Lanes(d));
  const hn::CappedTag<DegrainAcc, 1> one;
  for (int y = 0; y < height; ++y) {
    std::fill(sums.begin(), sums.end(), DegrainAcc(0));
    const int first = y < g.block_height ? 0 : (y - g.block_height) / sy + 1;
    const int last = std::min(y / sy, g.blocks_y - 1);
    for (int by = first; by <= last; ++by) {
      const int ly = y - by * sy;
      for (int bx = 0; bx < g.blocks_x; ++bx) {
        const auto index = (std::size_t(by) * g.blocks_x + bx) * (references + 1);
        const auto* sources = p.sources.data() + index;
        const auto* weights = p.weights.data() + index;
        const int ox = bx * sx;
        const int visible = y < g.visible_height ? std::clamp(g.visible_width - ox, 0, g.block_width) : 0;
        const auto consume = [&](auto tag, int x, bool write) HWY_ATTR {
          const auto value = DegrainValue<FixedReferences, FullRange>(tag, sources, weights, references, ly, x,
                                                                      limit.maximum());
          if (write) {
            if (overlap) {
              if constexpr (std::is_same_v<T, std::uint8_t>)
                AddDegrainByte(tag, value, sources[0].coefficients + std::size_t(ly) * g.block_width + x,
                               sums.data() + ox + x);
              else
                AddValue(tag, value, sources[0].coefficients + std::size_t(ly) * g.block_width + x,
                         sums.data() + ox + x);
            } else
              StoreSample(tag, value, out.row(y).data() + ox + x);
          }
        };
        int x = 0;
        for (; x + lanes <= visible; x += lanes)
          consume(d, x, true);
        for (; x < visible; ++x)
          consume(one, x, true);
        // Cropping does not remove numeric admission or weighted overflow checks.
        for (; x + lanes <= g.block_width; x += lanes)
          consume(d, x, false);
        for (; x < g.block_width; ++x)
          consume(one, x, false);
      }
    }
    if (y < g.visible_height) {
      auto* dst = out.row(y).data();
      if (overlap) {
        if constexpr (std::is_same_v<T, std::uint8_t>)
          FinishDegrainByte(sums.data(), dst, g.visible_width, std::uint16_t(limit.maximum()));
        else
          Finish(sums.data(), dst, g.visible_width, limit.maximum());
      }
      if (limit.active())
        Limit(dst, centre.row(y).data(), dst, g.visible_width, true, limit.maximum(), limit.integer_limit(),
              limit.float_limit());
    }
  }
}
template <class T, int Width, bool FullRange = false>
void DegrainPlaneCommon(const BlockCompositionGeometry& g, const DegrainPlane<T>& p, span2d::Plane<const T> centre,
                        span2d::Plane<T> out, const ChangeLimit<T>& limit) {
  if constexpr (!std::is_same_v<T, float>) {
    if (p.references == 2)
      return DegrainPlaneRun<T, Width, 2, FullRange>(g, p, centre, out, limit);
    if (p.references == 4)
      return DegrainPlaneRun<T, Width, 4, FullRange>(g, p, centre, out, limit);
  }
  DegrainPlaneRun<T, Width, 0, FullRange>(g, p, centre, out, limit);
}
template <class T, bool FullRange>
void DegrainDispatchRange(const BlockCompositionGeometry& g, const DegrainPlane<T>& p,
                          span2d::Plane<const T> centre, span2d::Plane<T> out, const ChangeLimit<T>& limit) {
  switch (g.block_width) {
    case 4:
      return DegrainPlaneRun<T, 4, 0, FullRange>(g, p, centre, out, limit);
    case 8:
      return DegrainPlaneCommon<T, 8, FullRange>(g, p, centre, out, limit);
    case 16:
      return DegrainPlaneCommon<T, 16, FullRange>(g, p, centre, out, limit);
    default:
      return DegrainPlaneRun<T, 32, 0, FullRange>(g, p, centre, out, limit);
  }
}
template <class T>
void DegrainDispatch(const BlockCompositionGeometry& g, const DegrainPlane<T>& p, span2d::Plane<const T> centre,
                     span2d::Plane<T> out, const ChangeLimit<T>& limit) {
  if constexpr (!std::is_same_v<T, float>)
    if (limit.maximum() == std::numeric_limits<T>::max())
      return DegrainDispatchRange<T, true>(g, p, centre, out, limit);
  DegrainDispatchRange<T, false>(g, p, centre, out, limit);
}
#define NEO_DEGRAIN_IMPL(T, S)                                                                                         \
  void Degrain##S(const BlockCompositionGeometry& g, const DegrainPlane<T>& p, span2d::Plane<const T> c,               \
                  span2d::Plane<T> o, const ChangeLimit<T>& l) {                                                       \
    DegrainDispatch(g, p, c, o, l);                                                                                    \
  }
NEO_DEGRAIN_IMPL(std::uint8_t, U8)
NEO_DEGRAIN_IMPL(std::uint16_t, U16)
NEO_DEGRAIN_IMPL(float, F32)
#undef NEO_DEGRAIN_IMPL

template <class T>
void ComposePlane(const BlockCompositionGeometry& g, const SampledRenderBlock<T>* b, span2d::Plane<T> out,
                  std::int64_t maximum) {
  // Choose short-row width once, outside all block/pixel loops.
  switch (g.block_width) {
    case 4:
      return ComposeSampled<T, 4>(g, b, out, maximum);
    case 8:
      return ComposeSampled<T, 8>(g, b, out, maximum);
    case 16:
      return ComposeSampled<T, 16>(g, b, out, maximum);
    case 32:
      return ComposeSampled<T, 32>(g, b, out, maximum);
    default:
      return ComposeSampled<T>(g, b, out, maximum);
  }
}
#define NEO_FUSED(T, S)                                                                                                \
  void ComposeSampled##S(const BlockCompositionGeometry& g, const SampledRenderBlock<T>* b, span2d::Plane<T> out,      \
                         std::int64_t max) {                                                                           \
    ComposePlane(g, b, out, max);                                                                                      \
  }
NEO_FUSED(std::uint8_t, U8)
NEO_FUSED(std::uint16_t, U16)
NEO_FUSED(float, F32)
#undef NEO_FUSED
#define NEO_IMPL(T, A, S)                                                                                              \
  void Weighted##S(const T* c, const T* const* r, const int* w, int cw, int nr, T* o, int n) {                         \
    Weighted(c, r, w, cw, nr, o, n);                                                                                   \
  }                                                                                                                    \
  void Add##S(const T* p, const std::uint16_t* w, A* a, int n) {                                                       \
    Add(p, w, a, n);                                                                                                   \
  }                                                                                                                    \
  void Finish##S(const A* a, T* o, int n, std::int64_t m) {                                                            \
    Finish(a, o, n, m);                                                                                                \
  }                                                                                                                    \
  void Limit##S(const T* q, const T* c, T* o, int n, bool active, std::int64_t m, std::int64_t l, float f) {           \
    Limit(q, c, o, n, active, m, l, f);                                                                                \
  }
NEO_IMPL(std::uint8_t, std::int32_t, U8)
NEO_IMPL(std::uint16_t, std::int32_t, U16)
NEO_IMPL(float, float, F32)
#undef NEO_IMPL
} // namespace HWY_NAMESPACE
} // namespace neo_mv::simd::detail
HWY_AFTER_NAMESPACE();
#if HWY_ONCE
namespace neo_mv::simd::detail {
#define NEO_DEGRAIN_EXPORT(T, S)                                                                                       \
  HWY_EXPORT(Degrain##S);                                                                                              \
  void compose_degrain(const BlockCompositionGeometry& g, const DegrainPlane<T>& p, span2d::Plane<const T> c,          \
                       span2d::Plane<T> o, const ChangeLimit<T>& l) {                                                  \
    HWY_DYNAMIC_DISPATCH(Degrain##S)(g, p, c, o, l);                                                                   \
  }
NEO_DEGRAIN_EXPORT(std::uint8_t, U8)
NEO_DEGRAIN_EXPORT(std::uint16_t, U16)
NEO_DEGRAIN_EXPORT(float, F32)
#undef NEO_DEGRAIN_EXPORT
#define NEO_FUSED_EXPORT(T, S)                                                                                         \
  HWY_EXPORT(ComposeSampled##S);                                                                                       \
  void compose_sampled(const BlockCompositionGeometry& g, const SampledRenderBlock<T>* b, span2d::Plane<T> out,        \
                       std::int64_t max) {                                                                             \
    HWY_DYNAMIC_DISPATCH(ComposeSampled##S)(g, b, out, max);                                                           \
  }
NEO_FUSED_EXPORT(std::uint8_t, U8)
NEO_FUSED_EXPORT(std::uint16_t, U16)
NEO_FUSED_EXPORT(float, F32)
#undef NEO_FUSED_EXPORT
#define NEO_EXPORT(T, A, S)                                                                                            \
  HWY_EXPORT(Weighted##S);                                                                                             \
  HWY_EXPORT(Add##S);                                                                                                  \
  HWY_EXPORT(Finish##S);                                                                                               \
  HWY_EXPORT(Limit##S);                                                                                                \
  void weighted(const T* c, const T* const* r, const int* w, int cw, int nr, T* o, int n) {                            \
    HWY_DYNAMIC_DISPATCH(Weighted##S)(c, r, w, cw, nr, o, n);                                                          \
  }                                                                                                                    \
  void overlap_add(const T* p, const std::uint16_t* w, A* a, int n) {                                                  \
    HWY_DYNAMIC_DISPATCH(Add##S)(p, w, a, n);                                                                          \
  }                                                                                                                    \
  void overlap_finish(const A* a, T* o, int n, std::int64_t m) {                                                       \
    HWY_DYNAMIC_DISPATCH(Finish##S)(a, o, n, m);                                                                       \
  }                                                                                                                    \
  void change_limit(const T* q, const T* c, T* o, int n, bool active, std::int64_t m, std::int64_t l, float f) {       \
    HWY_DYNAMIC_DISPATCH(Limit##S)(q, c, o, n, active, m, l, f);                                                       \
  }
NEO_EXPORT(std::uint8_t, std::int32_t, U8)
NEO_EXPORT(std::uint16_t, std::int32_t, U16)
NEO_EXPORT(float, float, F32)
#undef NEO_EXPORT
} // namespace neo_mv::simd::detail
#endif
