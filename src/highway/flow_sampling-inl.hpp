// Included inside each Highway target namespace; intentionally no include guard.
template <std::size_t Bytes, class Lane, bool HasStorage, bool CheckCoordinates>
void FlowSampleImpl(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                    PhaseRounding rounding, int first_row = 0, int row_count = -1) {
  const auto& g = plan.geometry();
  const hn::ScalableTag<Lane> d;
  const int lanes = static_cast<int>(hn::Lanes(d));
  const hn::Rebind<std::int16_t, decltype(d)> d16;
  const hn::Rebind<std::int32_t, decltype(d)> d32;
  HWY_ALIGN Lane columns[hn::MaxLanes(d)], rows[hn::MaxLanes(d)], phases[hn::MaxLanes(d)];
  HWY_ALIGN std::int32_t offsets[hn::MaxLanes(d)], narrow_strides[16]{};
  HWY_ALIGN Lane widths[16]{}, heights[16]{};
  Lane common_width = 0, common_height = 0;
  Lane edge_width = 0, edge_height = 0;
  bool separable = true;
  if constexpr (CheckCoordinates) {
    for (int a = 0; a < g.pel * g.pel; ++a) {
      widths[a] = g.phases[a].width;
      heights[a] = g.phases[a].height;
    }
    common_width = widths[0], common_height = heights[0];
    for (int a = 1; a < g.pel * g.pel; ++a) {
      common_width = (std::min)(common_width, widths[a]);
      common_height = (std::min)(common_height, heights[a]);
    }
    // Built-in quarter phases trim only the last fractional column/row.
    // External phases may have arbitrary extents, so retain the general lookup.
    edge_width = widths[g.pel == 4 ? 3 : 0];
    edge_height = heights[g.pel == 4 ? 12 : 0];
    for (int a = 0; a < g.pel * g.pel; ++a)
      separable &= widths[a] == (a % g.pel == 3 ? edge_width : widths[0]) &&
                   heights[a] == (a / g.pel == 3 ? edge_height : heights[0]);
  }
  const int shift = g.pel == 4 ? 2 : g.pel == 2 ? 1 : 0;
  const auto time = hn::Set(d32, plan.time_coefficient());
  const auto half = hn::Set(d32, rounding == PhaseRounding::nearest ? 128 : 0);
  const auto fraction = hn::Set(d, g.pel - 1), zero = hn::Zero(d);
  std::array<const std::byte*, 16> source_planes{};
  std::array<std::ptrdiff_t, 16> source_strides{};
  std::byte* output_base = nullptr;
  std::ptrdiff_t output_stride = 0;
  std::size_t pixel_stride = 0;
  if constexpr (HasStorage) {
    source_planes = storage->planes;
    source_strides = storage->strides;
    output_base = storage->output;
    output_stride = storage->output_stride;
    pixel_stride = storage->output_pixel_stride;
  }
  bool narrow_offsets = false;
  if constexpr (HasStorage && !CheckCoordinates && sizeof(Lane) == 4) {
    narrow_offsets = true;
    for (int a = 0; a < g.pel * g.pel; ++a) {
      const auto stride = source_strides[a];
      const auto last_column = std::int64_t(g.phases[a].width - 1) * Bytes;
      if (stride <= 0 || stride > INT32_MAX || last_column > INT32_MAX ||
          std::int64_t(g.phases[a].height - 1) > (INT32_MAX - last_column) / stride) {
        narrow_offsets = false;
        break;
      }
      narrow_strides[a] = static_cast<std::int32_t>(stride);
    }
  }
  for (int y = first_row; y < (row_count < 0 ? plan.height() : first_row + row_count); ++y) {
    auto* output_row = HasStorage ? output_base + std::ptrdiff_t(y - first_row) * output_stride : nullptr;
    for (int x = 0; x < plan.width();) {
      const int used = std::min(lanes, plan.width() - x);
      const auto index = std::size_t(y) * plan.width() + x;
      const auto vx =
          used == lanes ? hn::LoadU(d16, field.x.data() + index) : hn::LoadN(d16, field.x.data() + index, used);
      const auto vy =
          used == lanes ? hn::LoadU(d16, field.y.data() + index) : hn::LoadN(d16, field.y.data() + index, used);
      // int16 * [0,256] + [0,128] fits int32. Widen only after the
      // floor displacement; image coordinates and domain checks stay int64.
      const auto scaled_x = hn::Add(hn::Mul(hn::PromoteTo(d32, vx), time), half);
      const auto scaled_y = hn::Add(hn::Mul(hn::PromoteTo(d32, vy), time), half);
      const auto widen = [&](auto v) HWY_ATTR {
        if constexpr (sizeof(Lane) == 8)
          return hn::PromoteTo(d, v);
        else
          return v;
      };
      if constexpr (sizeof(Lane) == 8) {
        if constexpr (!HasStorage) {
          const auto unit = hn::Set(d, g.pel * 256);
          const auto px = hn::Add(hn::Mul(hn::Add(hn::Iota(d, x), hn::Set(d, g.pad_x)), unit), widen(scaled_x));
          const auto py = hn::Add(hn::Set(d, (std::int64_t(g.pad_y) + y) * g.pel * 256), widen(scaled_y));
          const auto valid = hn::And(hn::And(hn::Ge(px, zero), hn::Lt(px, hn::Set(d, common_width * g.pel * 256))),
                                     hn::And(hn::Ge(py, zero), hn::Lt(py, hn::Set(d, common_height * g.pel * 256))));
          if (hn::AllTrue(d, hn::Or(hn::Not(hn::FirstN(d, used)), valid))) {
            x += used;
            continue;
          }
        }
      }
      const auto dx = widen(hn::ShiftRight<8>(scaled_x));
      const auto dy = widen(hn::ShiftRight<8>(scaled_y));
      const auto ax = hn::And(dx, fraction), ay = hn::And(dy, fraction);
      const auto phase = hn::Add(ax, hn::ShiftLeftSame(ay, shift));
      const auto sx = hn::Add(hn::Set(d, g.pad_x), hn::Add(hn::Iota(d, x), hn::ShiftRightSame(dx, shift)));
      const auto sy = hn::Add(hn::Set(d, std::int64_t(g.pad_y) + y), hn::ShiftRightSame(dy, shift));
      if constexpr (CheckCoordinates) {
        const auto width =
            separable ? hn::IfThenElse(hn::Eq(ax, hn::Set(d, 3)), hn::Set(d, edge_width), hn::Set(d, widths[0]))
                      : hn::GatherIndex(d, widths, phase);
        const auto height =
            separable ? hn::IfThenElse(hn::Eq(ay, hn::Set(d, 3)), hn::Set(d, edge_height), hn::Set(d, heights[0]))
                      : hn::GatherIndex(d, heights, phase);
        const auto valid_x = hn::And(hn::Ge(sx, zero), hn::Lt(sx, width));
        const auto valid_y = hn::And(hn::Ge(sy, zero), hn::Lt(sy, height));
        if (!hn::AllTrue(d, hn::Or(hn::Not(hn::FirstN(d, used)), hn::And(valid_x, valid_y))))
          throw std::invalid_argument("Flow sample exceeds its logical phase domain");
      }
      if constexpr (HasStorage) {
        hn::Store(phase, d, phases);
        hn::Store(sx, d, columns);
        hn::Store(sy, d, rows);
        const bool use_offsets = narrow_offsets && used == lanes;
        if constexpr (sizeof(Lane) == 4) {
          if (use_offsets) {
            const auto stride = hn::GatherIndex(d, narrow_strides, phase);
            const auto offset = hn::Add(hn::Mul(sy, stride), hn::Mul(sx, hn::Set(d, std::int32_t(Bytes))));
            hn::Store(offset, d, offsets);
          }
        }
        for (int i = 0; i < used; ++i) {
          const auto a = static_cast<std::size_t>(phases[i]);
          const auto* source = use_offsets ? source_planes[a] + offsets[i]
                                           : source_planes[a] + rows[i] * source_strides[a] + columns[i] * Bytes;
          auto* output = output_row + std::size_t(x + i) * pixel_stride;
          std::memcpy(output, source, Bytes);
        }
      }
      x += used;
    }
  }
}

// Narrow coordinates double useful lanes for ordinary frames. Huge admitted
// images keep the wide path; the bound includes every int16 displacement.
template <std::size_t Bytes, bool HasStorage, bool CheckCoordinates>
void FlowSampleMode(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                    PhaseRounding rounding, int first_row, int row_count) {
  const auto& g = plan.geometry();
  if constexpr (HasStorage && !CheckCoordinates) {
    if (std::int64_t(g.pad_x) + plan.width() + 32768 <= INT32_MAX &&
        std::int64_t(g.pad_y) + plan.height() + 32768 <= INT32_MAX)
      return FlowSampleImpl<Bytes, std::int32_t, HasStorage, CheckCoordinates>(plan, field, storage, rounding,
                                                                               first_row, row_count);
  }
  FlowSampleImpl<Bytes, std::int64_t, HasStorage, CheckCoordinates>(plan, field, storage, rounding, first_row,
                                                                    row_count);
}
template <std::size_t Bytes>
void FlowSample(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                PhaseRounding rounding, int first_row = 0, int row_count = -1) {
  if (!storage)
    return FlowSampleMode<Bytes, false, true>(plan, field, storage, rounding, first_row, row_count);
  if (storage->coordinates_validated)
    return FlowSampleMode<Bytes, true, false>(plan, field, storage, rounding, first_row, row_count);
  FlowSampleMode<Bytes, true, true>(plan, field, storage, rounding, first_row, row_count);
}
