#pragma once
#include "core/depan/transform.hpp"
#include <charconv>
#include <string>

namespace neo_mv::depan {
inline std::string fixed(float x, int precision) {
  finite(x);
  char buffer[128];
  const auto r = std::to_chars(buffer, buffer + sizeof(buffer), double(x), std::chars_format::fixed, precision);
  if (r.ec != std::errc{})
    throw std::runtime_error("Depan diagnostic formatting failed");
  return {buffer, r.ptr};
}
inline std::string analysis_info(int n, int iteration, float error, Motion m) {
  auto text = "fn=" + std::to_string(n) + " iter=" + std::to_string(iteration) + " error=" + fixed(error, 3) +
              " dx=" + fixed(m.dx, 2) + " dy=" + fixed(m.dy, 2) + " rot=" + fixed(m.rotation, 3) +
              " zoom=" + fixed(m.zoom, 5) + " bad=" + (m.good ? "0" : "1");
  return text.substr(0, 127);
}
inline std::string compensation_info(float offset, int source, int n, Motion m) {
  auto text = "offset=" + fixed(offset, 2) + ", " + std::to_string(source) + " to " + std::to_string(n) +
              ", dx=" + fixed(m.dx, 2) + ", dy=" + fixed(m.dy, 2) + ", rot=" + fixed(m.rotation, 3) +
              " zoom=" + fixed(m.zoom, 5);
  return text.substr(0, 127);
}
} // namespace neo_mv::depan
