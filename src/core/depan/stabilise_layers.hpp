#pragma once
#include "core/depan/stabilise_smoothing.hpp"
#include "core/depan/diagnostics.hpp"
#include <optional>

namespace neo_mv::depan::stabilise {
struct Layer {
  int frame;
  Transform map;
};
struct Layers {
  std::optional<Layer> previous, next;
  Layer current;
};
template <class GetMotion>
Layers select_layers(Correction correction, int n, int frames, const Parameters& p, const Coefficients& c,
                     GetMotion&& get) {
  frame_valid(n, frames);
  Layers result{{}, {}, {n, correction.map}};
  if (p.prev > 0) {
    const int end = (std::max)(correction.begin, n - p.prev);
    Layer selected{end, correction.map};
    float best = 10000;
    for (int k = n - 1; k >= end; --k) {
      const auto m = get(k + 1);
      if (!m.good)
        throw std::invalid_argument("invalid DepanStabilise previous step");
      selected.map = compose(selected.map, coordinates(m, c.aspect, c.cx, c.cy, 1, true));
      const auto movement = motion(selected.map, c.aspect, c.cx, c.cy, true);
      const float score = sub(add(add(std::abs(movement.dx), std::abs(movement.dy)), f32(n)), f32(k));
      if (score < best) {
        best = score;
        selected.frame = k;
      }
    }
    result.previous = selected;
  }
  if (p.next > 0) {
    const int end = static_cast<int>((std::min)(std::int64_t(frames - 1), std::int64_t(n) + p.next));
    // Decode the entire range before inspecting validity or selecting a map.
    std::vector<Motion> motions;
    motions.reserve(std::size_t(end - n));
    for (std::int64_t k = std::int64_t(n) + 1; k <= end; ++k)
      motions.push_back(get(static_cast<int>(k)));
    Layer selected{end, correction.map};
    float best = 1000;
    for (std::int64_t k = std::int64_t(n) + 1; k <= end; ++k) {
      const auto m = motions[std::size_t(k - n - 1)];
      if (!m.good) {
        selected.frame = static_cast<int>(k) - 1;
        break;
      }
      selected.map = compose(inverse(coordinates(m, c.aspect, c.cx, c.cy, 1, true)), selected.map);
      const auto movement = motion(selected.map, c.aspect, c.cx, c.cy, true);
      const float score = sub(add(add(std::abs(movement.dx), std::abs(movement.dy)), f32(double(k))), f32(n));
      if (score < best) {
        best = score;
        selected.frame = static_cast<int>(k);
      }
    }
    result.next = selected;
  }
  return result;
}
inline std::string diagnostic(int n, Correction correction, const Coefficients& c) {
  const auto m = motion(correction.map, c.aspect, c.cx, c.cy, true);
  auto text = "frame=" + std::to_string(n) + (correction.begin == n ? " BASE!=" : " base =") +
              std::to_string(correction.begin) + " dx=" + fixed(m.dx, 2) + " dy=" + fixed(m.dy, 2) +
              " rot=" + fixed(m.rotation, 3) + " zoom=" + fixed(m.zoom, 5);
  return text.substr(0, 127);
}
} // namespace neo_mv::depan::stabilise
