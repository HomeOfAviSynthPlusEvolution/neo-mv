// Included inside each Highway target namespace; intentionally no include guard.
template <std::size_t Bytes>
void FlowSample(const neo_mv::FlowSamplingPlan& plan, const DenseFlowField& field, const FlowSampleStorage* storage,
                PhaseRounding rounding, int first_row = 0, int row_count = -1) {
  const auto& g = plan.geometry();
  const hn::ScalableTag<std::int64_t> d;
  const int lanes = static_cast<int>(hn::Lanes(d));
  const hn::Rebind<std::int16_t, decltype(d)> d16;
  const hn::Rebind<std::int32_t, decltype(d)> d32;
  HWY_ALIGN std::int64_t columns[hn::MaxLanes(d)], rows[hn::MaxLanes(d)], phases[hn::MaxLanes(d)];
  HWY_ALIGN std::int64_t widths[16]{}, heights[16]{};
  for (int a = 0; a < g.pel * g.pel; ++a) {
    widths[a] = g.phases[a].width;
    heights[a] = g.phases[a].height;
  }
  // Built-in quarter phases trim only the last fractional column/row.
  // External phases may have arbitrary extents, so retain the general lookup.
  const auto edge_width = widths[g.pel == 4 ? 3 : 0];
  const auto edge_height = heights[g.pel == 4 ? 12 : 0];
  bool separable = true;
  for (int a = 0; a < g.pel * g.pel; ++a)
    separable &= widths[a] == (a % g.pel == 3 ? edge_width : widths[0]) &&
                 heights[a] == (a / g.pel == 3 ? edge_height : heights[0]);
  const int shift = g.pel == 4 ? 2 : g.pel == 2 ? 1 : 0;
  const auto time = hn::Set(d32, plan.time_coefficient());
  const auto half = hn::Set(d32, rounding == PhaseRounding::nearest ? 128 : 0);
  const auto fraction = hn::Set(d, g.pel - 1), zero = hn::Zero(d);
  for (int y = first_row; y < (row_count < 0 ? plan.height() : first_row + row_count); ++y) {
    for (int x = 0; x < plan.width();) {
      const int used = std::min(lanes, plan.width() - x);
      const auto index = std::size_t(y) * plan.width() + x;
      const auto vx =
          used == lanes ? hn::LoadU(d16, field.x.data() + index) : hn::LoadN(d16, field.x.data() + index, used);
      const auto vy =
          used == lanes ? hn::LoadU(d16, field.y.data() + index) : hn::LoadN(d16, field.y.data() + index, used);
      // int16 * [0,256] + [0,128] fits int32. Widen only after the
      // floor displacement; image coordinates and domain checks stay int64.
      const auto dx = hn::PromoteTo(d, hn::ShiftRight<8>(hn::Add(hn::Mul(hn::PromoteTo(d32, vx), time), half)));
      const auto dy = hn::PromoteTo(d, hn::ShiftRight<8>(hn::Add(hn::Mul(hn::PromoteTo(d32, vy), time), half)));
      const auto ax = hn::And(dx, fraction), ay = hn::And(dy, fraction);
      const auto phase = hn::Add(ax, hn::ShiftLeftSame(ay, shift));
      const auto sx = hn::Add(hn::Set(d, g.pad_x), hn::Add(hn::Iota(d, x), hn::ShiftRightSame(dx, shift)));
      const auto sy = hn::Add(hn::Set(d, std::int64_t(g.pad_y) + y), hn::ShiftRightSame(dy, shift));
      if (!storage || !storage->coordinates_validated) {
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
      if (storage) {
        hn::Store(sx, d, columns);
        hn::Store(sy, d, rows);
        hn::Store(phase, d, phases);
        for (int i = 0; i < used; ++i) {
          const auto a = static_cast<std::size_t>(phases[i]);
          const auto* source = storage->planes[a] + rows[i] * storage->strides[a] + columns[i] * Bytes;
          auto* output = storage->output + std::ptrdiff_t(y - first_row) * storage->output_stride +
                         std::size_t(x + i) * storage->output_pixel_stride;
          std::memcpy(output, source, Bytes);
        }
      }
      x += used;
    }
  }
}
