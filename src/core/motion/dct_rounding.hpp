#pragma once

#include <array>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <type_traits>

namespace neo_mv::dct_detail {

// Tests whether the exact normalized AC coefficient equals k + 1/2.
// Samples are contiguous unsigned 8/16-bit integers. This is an equality
// predicate, not a test for proximity to a boundary or for rounding direction.
// DC has a different normalization and must not use this predicate.
template <class T>
bool exact_half(const T* samples, int width, int height, int u, int v, std::int64_t k) {
  static_assert(std::is_integral_v<T> && std::is_unsigned_v<T> && sizeof(T) <= 2);
  if (!samples || width < 1 || width > 128 || height < 1 || height > 128 || u < 0 || u >= width || v < 0 ||
      v >= height || (u == 0 && v == 0))
    throw std::invalid_argument("invalid DCT AC coefficient");
  const int order = std::lcm(std::lcm(4 * width, 4 * height), 8);
  const bool third = order % 3 == 0;
  const int power = third ? order / 3 : order;
  if (order > 512 || (power & (power - 1)) != 0)
    throw std::invalid_argument("unsupported DCT cyclotomic order");

  // |AC| <= 2*sqrt(2)*65535 < 185364. Reject impossible boundaries before
  // computing 2*k+1, including callers passing the int64 limits.
  if (k < -185364 || k >= 185364)
    return false;

  // For z a primitive order-th root, the unnormalized DCT is
  // F = sum(sample * (z^a + z^-a) * (z^b + z^-b)). Since sqrt(2) equals
  // z^(order/8) + z^(-order/8), AC=F/(sqrt(2)*area) is k+1/2 precisely when
  // 2F - (2*k+1)*area*(z^(order/8)+z^(-order/8)) has zero remainder mod Phi.
  std::array<std::int64_t, 512> polynomial{};
  const auto add = [&](int exponent, std::int64_t value) {
    exponent %= order;
    if (exponent < 0)
      exponent += order;
    polynomial[exponent] += value;
  };
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      const int a = (2 * x + 1) * u * (order / (4 * width));
      const int b = (2 * y + 1) * v * (order / (4 * height));
      const auto value = 2 * std::int64_t(samples[y * width + x]);
      add(a + b, value);
      add(a - b, value);
      add(-a + b, value);
      add(-a - b, value);
    }
  const auto threshold = (2 * k + 1) * std::int64_t(width) * height;
  add(order / 8, -threshold);
  add(-order / 8, -threshold);

  // Phi_(2^m) = X^(order/2)+1;
  // Phi_(3*2^m) = X^(order/3)-X^(order/6)+1.
  // The initial coefficient L1 norm is < 2^35. The first reduction never
  // increases it. In the second, each branch lowers its degree by at least
  // order/6; a monomial needs at most four such steps. Even ignoring all
  // cancellations gives a factor <= 16, so intermediates remain below 2^39.
  const int degree = third ? order / 3 : order / 2;
  for (int i = order - 1; i >= degree; --i) {
    const auto value = polynomial[i];
    polynomial[i] = 0;
    polynomial[i - degree] -= value;
    if (third)
      polynomial[i - degree + degree / 2] += value;
  }
  for (int i = 0; i < degree; ++i)
    if (polynomial[i] != 0)
      return false;
  return true;
}

} // namespace neo_mv::dct_detail
