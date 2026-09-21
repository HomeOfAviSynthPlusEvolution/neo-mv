#pragma once

#include "core/base/plane.hpp"
#include "core/depan/diagnostics.hpp"

#include <vector>

namespace neo_mv::depan::estimate {
namespace image_detail {
template <class T>
std::size_t rectangle(span2d::Plane<T> plane, int left, int top, int width, int height) {
  validate_plane(plane);
  if (left < 0 || top < 0 || width <= 0 || height <= 0 || left > plane.width() || top > plane.height() ||
      width > plane.width() - left || height > plane.height() - top)
    throw std::invalid_argument("invalid DepanEstimate image rectangle");
  const auto w = static_cast<std::size_t>(width), h = static_cast<std::size_t>(height);
  if (w > std::vector<float>().max_size() / h)
    throw std::overflow_error("DepanEstimate image rectangle is too large");
  return w * h;
}
template <class T>
float maximum(int bits) {
  mask_detail::validate_storage<T>(bits);
  if constexpr (std::is_same_v<T, float>)
    return 1;
  else
    return static_cast<float>((std::uint32_t(1) << bits) - 1);
}
} // namespace image_detail

template <class T>
std::vector<float> extract_window(span2d::Plane<const T> source, int left, int top, int width, int height, int bits) {
  const auto maximum = image_detail::maximum<T>(bits);
  std::vector<float> result(image_detail::rectangle(source, left, top, width, height));
  for (int y = 0; y < height; ++y) {
    const auto row = source.row(top + y);
    for (int x = 0; x < width; ++x) {
      const float value = finite(static_cast<float>(row[left + x]));
      if constexpr (!std::is_same_v<T, float>) {
        if (value > maximum)
          throw std::invalid_argument("DepanEstimate sample exceeds nominal precision");
      }
      result[std::size_t(y) * width + x] = value;
    }
  }
  return result;
}

template <class T>
void display_surface(span2d::Plane<T> output, const std::vector<float>& surface, int left, int top, int width,
                     int height, int bits) {
  const float maximum = image_detail::maximum<T>(bits);
  const auto count = image_detail::rectangle(output, left, top, width, height);
  if (surface.size() != count)
    throw std::invalid_argument("DepanEstimate correlation surface size mismatch");
  float minimum = finite(surface.front()), peak = minimum;
  for (float value : surface) {
    finite(value);
    if (value < minimum)
      minimum = value;
    if (value > peak)
      peak = value;
  }
  const float difference = sub(peak, minimum);
  if (difference <= 0)
    throw std::invalid_argument("constant DepanEstimate correlation display");
  const float norm = div(maximum, difference);
  // Complete numeric validation before modifying any output sample.
  std::vector<T> samples(count);
  for (std::size_t i = 0; i < count; ++i) {
    const float q = mul(sub(surface[i], minimum), norm);
    if constexpr (std::is_same_v<T, float>) {
      samples[i] = q;
    } else {
      const double integer = std::trunc(double(q));
      if (integer < 0 || integer > double(maximum))
        throw std::invalid_argument("DepanEstimate display integer is outside nominal precision");
      samples[i] = static_cast<T>(integer);
    }
  }
  for (int y = 0; y < height; ++y) {
    auto row = output.row(top + y);
    for (int x = 0; x < width; ++x)
      row[left + x] = samples[std::size_t(y) * width + x];
  }
}

inline std::string diagnostic(int n, Motion motion, float confidence) {
  auto text = "fn=" + std::to_string(n) + " dx=" + fixed(motion.dx, 2) + " dy=" + fixed(motion.dy, 2) +
              " zoom=" + fixed(motion.zoom, 5) + " trust=" + fixed(confidence, 2) + " bad=" +
              (motion.good ? "0" : "1");
  return text.substr(0, 127);
}
} // namespace neo_mv::depan::estimate
