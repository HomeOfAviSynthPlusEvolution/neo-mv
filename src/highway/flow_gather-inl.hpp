// Included inside each Highway target namespace; intentionally no include guard.
#if HWY_TARGET <= HWY_AVX2
// Read only complete words wholly inside a phase row. Narrow/odd external
// phases keep the general byte sampler; no padding or overread is assumed.
template <class T>
class PhaseGather {
  const RenderPhaseGeometry& geometry_;
  std::array<const std::byte*, 16> sources_{};
  std::array<std::ptrdiff_t, 16> source_strides_{};
  std::int32_t strides_[4]{};

public:
  PhaseGather(const RenderPhaseGeometry& geometry, const SubpixelPhases<T>& image) : geometry_(geometry) {
    for (int a = 0; a < image.pel * image.pel; ++a) {
      sources_[a] = reinterpret_cast<const std::byte*>(image.planes[a].row(0).data());
      source_strides_[a] = image.planes[a].stride_bytes();
    }
  }
  PhaseGather(const RenderPhaseGeometry& geometry, const FlowSampleStorage& storage)
      : geometry_(geometry), sources_(storage.planes), source_strides_(storage.strides) {}
  bool prepare(int width, int height) {
    const auto& g = geometry_;
    if (g.pel > 2 || std::int64_t(g.pad_x) + width + 32768 > INT32_MAX / int(sizeof(T)) ||
        std::int64_t(g.pad_y) + height + 32768 > INT32_MAX)
      return false;
    for (int a = 0; a < g.pel * g.pel; ++a) {
      const auto stride = source_strides_[a];
      const auto bytes = std::int64_t(g.phases[a].width) * sizeof(T);
      if (bytes < 4 || bytes % 4 || stride <= 0 || stride % 4 || stride > INT32_MAX || bytes > INT32_MAX ||
          std::int64_t(g.phases[a].height - 1) > (INT32_MAX - bytes) / stride)
        return false;
      strides_[a] = static_cast<std::int32_t>(stride / 4);
    }
    return true;
  }
  template <class D>
  HWY_INLINE auto sample(D d, const DenseFlowField& field, int time, int x, int y, int bits, int bias = 0) const {
    const hn::Rebind<std::int32_t, D> di;
    const hn::Rebind<std::uint32_t, D> du;
    const hn::Rebind<std::int16_t, D> ds;
    const auto i = std::size_t(y) * field.width + x;
    const auto dx = hn::ShiftRight<8>(
        hn::Add(hn::Mul(hn::PromoteTo(di, hn::LoadU(ds, field.x.data() + i)), hn::Set(di, time)), hn::Set(di, bias)));
    const auto dy = hn::ShiftRight<8>(
        hn::Add(hn::Mul(hn::PromoteTo(di, hn::LoadU(ds, field.y.data() + i)), hn::Set(di, time)), hn::Set(di, bias)));
    const int shift = geometry_.pel == 2 ? 1 : 0;
    const auto fraction = hn::Set(di, geometry_.pel - 1);
    const auto phase = hn::Add(hn::And(dx, fraction), hn::ShiftLeftSame(hn::And(dy, fraction), shift));
    const auto sx = hn::Add(hn::Iota(di, x + geometry_.pad_x), hn::ShiftRightSame(dx, shift));
    const auto sy = hn::Add(hn::Set(di, y + geometry_.pad_y), hn::ShiftRightSame(dy, shift));
    const auto byte_x = hn::Mul(sx, hn::Set(di, int(sizeof(T))));
    auto words = hn::Zero(du);
    const int first_phase = hn::GetLane(phase);
    if (hn::AllTrue(di, hn::Eq(phase, hn::Set(di, first_phase)))) {
      const auto index = hn::Add(hn::Mul(sy, hn::Set(di, strides_[first_phase])), hn::ShiftRight<2>(byte_x));
      const auto* source = reinterpret_cast<const std::uint32_t*>(sources_[first_phase]);
      // x86's smallest unmasked gather reads four lanes, even for a capped tag.
      if (hn::Lanes(du) >= 4)
        words = hn::GatherIndex(du, source, index);
      else
        words = hn::MaskedGatherIndexOr(words, hn::FirstN(du, hn::Lanes(du)), du, source, index);
    } else
      for (int a = 0; a < geometry_.pel * geometry_.pel; ++a) {
        const auto selected = hn::And(hn::FirstN(du, hn::Lanes(du)), hn::RebindMask(du, hn::Eq(phase, hn::Set(di, a))));
        if (hn::AllFalse(du, selected))
          continue;
        const auto index = hn::Add(hn::Mul(sy, hn::Set(di, strides_[a])), hn::ShiftRight<2>(byte_x));
        words =
            hn::MaskedGatherIndexOr(words, selected, du, reinterpret_cast<const std::uint32_t*>(sources_[a]), index);
      }
    if constexpr (std::is_same_v<T, float>) {
      const auto value = hn::BitCast(d, words);
      if (!hn::AllTrue(d, hn::IsFinite(value)))
        throw std::invalid_argument("non-finite SIMD interpolation intermediate");
      return value;
    } else if constexpr (sizeof(T) == 4) {
      return words;
    } else {
      const auto shift_bits = hn::BitCast(du, hn::ShiftLeft<3>(hn::And(byte_x, hn::Set(di, 3))));
      const auto value = hn::And(words >> shift_bits, hn::Set(du, (1u << (sizeof(T) * 8)) - 1));
      if (bits < int(sizeof(T) * 8) && !hn::AllTrue(du, hn::Le(value, hn::Set(du, (1u << bits) - 1))))
        throw std::invalid_argument("interpolation sample exceeds bit depth");
      return value;
    }
  }
};
#endif
