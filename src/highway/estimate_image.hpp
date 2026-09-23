#pragma once
#include "core/depan/estimate_image.hpp"
#include <algorithm>

namespace neo_mv::simd::estimate {
void extract_row(const std::uint8_t*, float*, std::size_t, float);
void extract_row(const std::uint16_t*, float*, std::size_t, float);
void extract_row(const float*, float*, std::size_t, float);
// Compare against a previously admitted window, including the sign of zero.
bool matches_row(const std::uint8_t*, const float*, std::size_t);
bool matches_row(const std::uint16_t*, const float*, std::size_t);
bool matches_row(const float*, const float*, std::size_t);
template <class T>
bool matches_window(span2d::Plane<const T> source, int left, int top, int width, int height,
                    const std::vector<float>& admitted) {
  if (depan::estimate::image_detail::rectangle(source, left, top, width, height) != admitted.size())
    return false;
  for (int y = 0; y < height; ++y)
    if (!matches_row(source.row(top + y).data() + left, admitted.data() + std::size_t(y) * width, width))
      return false;
  return true;
}
void display_row(const float*, std::uint8_t*, std::size_t, float, float, float);
void display_row(const float*, std::uint16_t*, std::size_t, float, float, float);
void display_row(const float*, float*, std::size_t, float, float, float);
void extrema(const float*, std::size_t, float&, float&);

template <class T>
std::vector<float> extract_window(span2d::Plane<const T> source, int left, int top, int width, int height, int bits) {
  const auto maximum = depan::estimate::image_detail::maximum<T>(bits);
  std::vector<float> result(depan::estimate::image_detail::rectangle(source, left, top, width, height));
  for (int y = 0; y < height; ++y)
    extract_row(source.row(top + y).data() + left, result.data() + std::size_t(y) * width, width, maximum);
  return result;
}

template <class T>
void display_surface(span2d::Plane<T> output, const std::vector<float>& surface, int left, int top, int width,
                     int height, int bits) {
  const auto maximum = depan::estimate::image_detail::maximum<T>(bits);
  const auto count = depan::estimate::image_detail::rectangle(output, left, top, width, height);
  if (surface.size() != count)
    throw std::invalid_argument("DepanEstimate correlation surface size mismatch");
  float minimum, peak;
  extrema(surface.data(), count, minimum, peak);
  const float difference = depan::sub(peak, minimum);
  if (difference <= 0)
    throw std::invalid_argument("constant DepanEstimate correlation display");
  const float norm = depan::div(maximum, difference);
  // Preserve the scalar contract: all numeric checks precede output writes.
  std::vector<T> samples(count);
  display_row(surface.data(), samples.data(), count, minimum, norm, maximum);
  for (int y = 0; y < height; ++y)
    std::copy_n(samples.data() + std::size_t(y) * width, width, output.row(top + y).data() + left);
}
} // namespace neo_mv::simd::estimate
