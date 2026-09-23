// Intentionally included in both the core and selected SIMD namespaces.
// Requires SearchParams/SearchResult and an evaluator with improve(v, p, best).
// NEO_MV_MOTION_ATTR supplies the target attribute for lambda call operators.
template <class Evaluator>
SearchResult refine_impl(SearchResult initial, const SearchParams& p, Evaluator&& evaluate) {
  const auto omega = p.domain;
  if (p.lambda < 0 || p.penalty < 0 || p.penalty > 256 || p.type < 0 || p.type > 5 || p.range <= 0 ||
      initial.cost < 0 || initial.raw < 0 || omega.left < INT32_MIN || omega.top < INT32_MIN ||
      omega.right > std::int64_t(INT32_MAX) + 1 || omega.bottom > std::int64_t(INT32_MAX) + 1 ||
      omega.left >= omega.right || omega.top >= omega.bottom)
    throw std::invalid_argument("invalid motion refinement inputs");
  SearchResult best = initial;
  const auto consider = [&](std::int64_t x, std::int64_t y) NEO_MV_MOTION_ATTR {
    if (x < omega.left || x >= omega.right || y < omega.top || y >= omega.bottom)
      return false;
    const MotionVector v{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
    return evaluate.improve(v, p, best);
  };
  const auto offset = [&](MotionVector center, std::int64_t x, std::int64_t y) NEO_MV_MOTION_ATTR {
    return consider(std::int64_t(center.x) + x, std::int64_t(center.y) + y);
  };
  const auto ring = [&](MotionVector center, std::int64_t radius) NEO_MV_MOTION_ATTR {
    for (auto i = -radius + 1; i < radius; ++i) {
      offset(center, i, -radius);
      offset(center, i, radius);
    }
    for (auto j = -radius + 1; j < radius; ++j) {
      offset(center, -radius, j);
      offset(center, radius, j);
    }
    offset(center, -radius, -radius);
    offset(center, -radius, radius);
    offset(center, radius, -radius);
    offset(center, radius, radius);
  };
  const auto hexagonal = [&]() NEO_MV_MOTION_ATTR {
    if (p.range == 1) {
      ring(best.vector, 1);
      return;
    }
    constexpr MotionVector h[] = {{-2, 0}, {-1, 2}, {1, 2}, {2, 0}, {1, -2}, {-1, -2}};
    auto center = best.vector;
    int direction = -1;
    for (int i = 0; i < 6; ++i)
      if (offset(center, h[i].x, h[i].y))
        direction = i;
    if (direction >= 0)
      for (int step = 0; step < p.range / 2 - 1; ++step) {
        center = best.vector;
        int winner = -1;
        for (int k = -1; k <= 1; ++k) {
          const int i = (direction + k + 6) % 6;
          if (offset(center, h[i].x, h[i].y))
            winner = i;
        }
        if (winner < 0)
          break;
        direction = winner;
      }
    ring(best.vector, 1);
  };

  const MotionVector origin = initial.vector;
  switch (p.type) {
    case 0: {
      // Index 0 is the unrestricted hint; 1..4 are axial and 5..8 diagonal.
      constexpr MotionVector directions[] = {
          {0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      constexpr int fallback[][4] = {{5, 6, 7, 8}, {5, 7, 0, 0}, {6, 8, 0, 0},
                                     {5, 6, 0, 0}, {7, 8, 0, 0}, {5, 6, 7, 0},
                                     {5, 6, 8, 0}, {5, 8, 7, 0}, {8, 6, 7, 0}};
      for (std::int64_t d = p.range; d > 0; d /= 2) {
        int hint = 0;
        for (;;) {
          const auto center = best.vector;
          int winner = 0;
          for (int i = 1; i <= 4; ++i) {
            const auto e = directions[i], h = directions[hint];
            if ((hint == 0 || e.x * h.x + e.y * h.y > 0) && offset(center, d * e.x, d * e.y))
              winner = i;
          }
          if (winner != 0) {
            // Both perpendicular choices use the selected axial position.
            const auto axial = best.vector;
            const int first = winner <= 2 ? 3 : 1;
            hint = winner;
            for (int i = first; i < first + 2; ++i) {
              const auto e = directions[i];
              if (offset(axial, d * e.x, d * e.y))
                hint = i; // Carry only the perpendicular direction forward.
            }
            continue;
          }
          for (const int i : fallback[hint]) {
            if (i == 0)
              break;
            const auto e = directions[i];
            if (offset(center, d * e.x, d * e.y))
              winner = i;
          }
          if (winner == 0)
            break;
          hint = winner;
        }
      }
      break;
    }
    case 1:
      for (std::int64_t r = 1; r <= p.range; ++r)
        ring(origin, r);
      break;
    case 2:
      hexagonal();
      break;
    case 3: {
      const auto center = p.cross_center.value_or(origin);
      for (std::int64_t d = 1; d < p.range; d += 2) {
        offset(center, -d, 0);
        offset(center, d, 0);
      }
      for (std::int64_t d = 1; d < p.range; d += 2) {
        offset(center, 0, -d);
        offset(center, 0, d);
      }
      constexpr MotionVector group[] = {{-4, 2}, {-4, 1}, {-4, 0}, {-4, -1}, {-4, -2}, {4, -2},  {4, -1}, {4, 0},
                                        {4, 1},  {4, 2},  {2, 3},  {0, 4},   {-2, 3},  {-2, -3}, {0, -4}, {2, -3}};
      for (std::int64_t k = 1; k <= std::max(1, p.range / 4); ++k)
        for (auto v : group)
          offset(center, k * v.x, k * v.y);
      hexagonal();
      break;
    }
    case 4:
    case 5:
      for (std::int64_t d = 1; d <= p.range; ++d) {
        if (p.type == 4) {
          offset(origin, -d, 0);
          offset(origin, d, 0);
        } else {
          offset(origin, 0, -d);
          offset(origin, 0, d);
        }
      }
      break;
  }
  return best;
}
