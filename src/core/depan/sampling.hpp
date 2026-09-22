#pragma once

#include "core/base/plane.hpp"
#include "core/depan/numeric.hpp"
#include "core/depan/transform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace neo_mv::depan {
enum class SamplingClass { translation, scale, affine };
struct SamplingCoordinates {
  double i = 0, j = 0;
  float fx = 0, fy = 0;
};

inline std::array<std::int64_t, 4> cubic_coefficients(int a) {
  if (a < 0 || a > 256)
    throw std::invalid_argument("invalid cubic sampling coefficient");
  const auto x = std::int64_t(a), other = 256 - x;
  return {-x * other * other / 8192, (16777216 - 512 * x * x + x * x * x) / 8192, x * (65536 + 256 * x - x * x) / 8192,
          -x * x * other / 8192};
}

class SamplingPlan {
  int width_, height_, bits_, mode_, mirror_, blur_, border_;
  Transform map_;
  SamplingClass class_;
  float translation_fraction_ = 0;

  static bool valid(double value, int length) { return value >= 0 && value < length; }
  static double reflect(double value, int length, bool low, bool high) {
    if (value < 0 && low)
      value = -value;
    if (value >= length && high)
      value = 2.0 * length - value - 2;
    // All potentially in-range results are exact binary64 integers. A huge
    // finite float coordinate remains far outside after these two operations;
    // any unrepresented low bits cannot affect admission or a clamped read.
    return value;
  }
  static std::int64_t floor_div(std::int64_t value, int denominator) {
    return value >= 0 ? value / denominator : -1 - ((-1 - value) / denominator);
  }
  SamplingCoordinates split(float x, float y, bool truncate_nearest = false) const {
    SamplingCoordinates result;
    if (mode_ == 0) {
      const double a = add(x, 0.5f), b = add(y, 0.5f);
      result.i = truncate_nearest ? std::trunc(a) : std::floor(a);
      result.j = truncate_nearest ? std::trunc(b) : std::floor(b);
    } else {
      result.i = std::floor(double(x));
      result.j = std::floor(double(y));
      result.fx = sub(x, f32(result.i));
      result.fy = sub(y, f32(result.j));
    }
    return result;
  }
  template <class T>
  static T read(span2d::Plane<const T> source, double x, double y) {
    // Every caller establishes the logical domain before these conversions.
    return source.row(static_cast<int>(y))[static_cast<int>(x)];
  }
  template <class T>
  std::optional<T> general(span2d::Plane<const T> source, double i, double j) const {
    j = reflect(j, height_, mirror_ & 1, mirror_ & 2);
    i = reflect(i, width_, mirror_ & 4, mirror_ & 8);
    return valid(i, width_) && valid(j, height_) ? read(source, i, j) : std::optional<T>{};
  }
  template <class T>
  T clamped_mean(span2d::Plane<const T> source, double start, std::uint32_t count, int row) const {
    const auto values = source.row(row);
    if (start >= width_ - 1)
      return values[width_ - 1];
    if (start + count - 1 <= 0)
      return values[0];
    // Reaching this branch bounds start to (-count,width), well inside int64.
    // Count endpoint repeats algebraically; only distinct visible taps iterate.
    const auto first = static_cast<std::int64_t>(start), end = first + count;
    const auto left = std::min<std::int64_t>(count, std::max<std::int64_t>(0, -first));
    const auto begin = std::max<std::int64_t>(0, first), finish = std::min<std::int64_t>(width_, end);
    const auto middle = finish - begin;
    const auto right = std::int64_t(count) - left - middle;
    std::uint64_t sum = std::uint64_t(left) * values[0] + std::uint64_t(right) * values[width_ - 1];
    for (auto x = begin; x < finish; ++x)
      sum += values[static_cast<int>(x)];
    return static_cast<T>(sum / count);
  }
  template <class T>
  std::optional<T> horizontal(span2d::Plane<const T> source, double i, int row, bool bilinear, int blur) const {
    double start = 0, length = 1;
    if (i < 0 && (mirror_ & 4)) {
      start = -i;
      if (blur > 0) {
        length = std::min(double(blur), -i);
        start = -i - length + 1;
      }
    } else if (i >= width_ - (bilinear ? 1 : 0) && (mirror_ & 8)) {
      start = 2.0 * width_ - i - 2;
      if (blur > 0)
        length = std::min(double(blur), i - width_ + (bilinear ? 2 : 1));
    } else {
      return std::nullopt;
    }
    if (blur == 0)
      return read(source, std::clamp(start, 0.0, double(width_ - 1)), row);
    return clamped_mean(source, start, static_cast<std::uint32_t>(length), row);
  }
  template <class T>
  T bilinear(span2d::Plane<const T> source, const SamplingCoordinates& q) const {
    const int x = static_cast<int>(q.i), y = static_cast<int>(q.j);
    const auto a = static_cast<std::uint32_t>(std::floor(double(mul(32.0f, q.fx))));
    const auto b = static_cast<std::uint32_t>(std::floor(double(mul(32.0f, q.fy))));
    const auto top = source.row(y), bottom = source.row(y + 1);
    const std::uint64_t sum = std::uint64_t(32 - a) * (32 - b) * top[x] + std::uint64_t(a) * (32 - b) * top[x + 1] +
                              std::uint64_t(32 - a) * b * bottom[x] + std::uint64_t(a) * b * bottom[x + 1];
    return static_cast<T>(sum / 1024);
  }
  template <class T>
  T cubic(span2d::Plane<const T> source, const SamplingCoordinates& q) const {
    const auto cx = cubic_coefficients(static_cast<int>(mul(256.0f, q.fx)));
    const auto cy = cubic_coefficients(static_cast<int>(mul(256.0f, q.fy)));
    const int x = static_cast<int>(q.i), y = static_cast<int>(q.j);
    std::int64_t sum = 0;
    for (int f = 0; f < 4; ++f)
      for (int e = 0; e < 4; ++e) {
        const auto coefficient = class_ == SamplingClass::translation ? cx[e] * cy[f] / 2048 : cx[e] * cy[f];
        sum += coefficient * source.row(y + f - 1)[x + e - 1];
      }
    const auto value = class_ == SamplingClass::translation ? floor_div(sum + 1024, 2048) : floor_div(sum, 4194304);
    return static_cast<T>(std::clamp(value, std::int64_t{0}, std::int64_t((1u << bits_) - 1)));
  }
  template <class T>
  T near_edge(span2d::Plane<const T> source, SamplingCoordinates q) const {
    const int x = static_cast<int>(q.i), y = static_cast<int>(q.j);
    const double fy = class_ == SamplingClass::translation ? translation_fraction_ : q.fy;
    const double fx = q.fx, complement = sub64(1.0, fx);
    const auto top = source.row(y), bottom = source.row(y + 1);
    const double a = add64(mul64(complement, double(top[x])), double(mul(q.fx, f32(top[x + 1]))));
    const double b = add64(mul64(complement, double(bottom[x])), double(mul(q.fx, f32(bottom[x + 1]))));
    auto value = std::trunc(add64(mul64(sub64(1.0, fy), a), mul64(fy, b)));
    if (class_ == SamplingClass::scale)
      value = std::clamp(value, 0.0, double((1u << bits_) - 1));
    return static_cast<T>(value);
  }

public:
  SamplingPlan(int width, int height, int bits, int mode, int mirror, int blur, int border, Transform map)
      : width_(width), height_(height), bits_(bits), mode_(mode), mirror_(mirror), blur_(blur), border_(border),
        map_(map) {
    if (width <= 0 || height <= 0 || bits < 8 || bits > 16 || mode < 0 || mode > 2 || mirror < 0 || mirror > 15 ||
        blur < 0 || border < 0 || border > int((1u << bits) - 1) || (mode == 2 && height < 2))
      throw std::invalid_argument("invalid Depan sampling geometry or controls");
    for (const float coefficient : {map.tx, map.ty, map.u, map.v, map.w, map.h})
      if (!std::isfinite(coefficient))
        throw std::invalid_argument("non-finite Depan sampling map");
    const float limit = f32(double(std::int64_t(width) + height + 64));
    map_.tx = std::clamp(map_.tx, -limit, limit);
    map_.ty = std::clamp(map_.ty, -limit, limit);
    class_ = map_.v == 0 && map_.w == 0
                 ? (map_.u == 1 && map_.h == 1 ? SamplingClass::translation : SamplingClass::scale)
                 : SamplingClass::affine;
    if (class_ == SamplingClass::translation && mode == 2)
      translation_fraction_ = sub(map_.ty, f32(std::floor(double(map_.ty))));
  }
  int width() const { return width_; }
  int height() const { return height_; }
  int bits() const { return bits_; }
  int mode() const { return mode_; }
  SamplingClass sampling_class() const { return class_; }
  const Transform& map() const { return map_; }

  template <class Visit>
  void row_coordinates(int y, Visit&& visit) const {
    if (y < 0 || y >= height_)
      throw std::invalid_argument("Depan sampling row is outside output");
    const float fy = f32(y);
    if (class_ == SamplingClass::translation) {
      const auto row = split(map_.tx, add(map_.ty, fy));
      for (int x = 0; x < width_; ++x) {
        auto at = row;
        at.i += x; // Translation's horizontal origin addition is exact integer arithmetic.
        visit(x, at);
      }
    } else if (class_ == SamplingClass::scale) {
      const float Y = add(map_.ty, mul(map_.h, fy));
      for (int x = 0; x < width_; ++x)
        visit(x, split(add(map_.tx, mul(map_.u, f32(x))), Y));
    } else if (mode_ == 2) {
      for (int x = 0; x < width_; ++x) {
        const float fx = f32(x);
        visit(x, split(add(add(map_.tx, mul(map_.u, fx)), mul(map_.v, fy)),
                       add(add(map_.ty, mul(map_.w, fx)), mul(map_.h, fy))));
      }
    } else {
      float X = add(map_.tx, mul(map_.v, fy)), Y = add(map_.ty, mul(map_.h, fy));
      for (int x = 0; x < width_; ++x) {
        visit(x, split(X, Y, mode_ == 0));
        if (x + 1 < width_) {
          X = add(X, map_.u);
          Y = add(Y, map_.w);
        }
      }
    }
  }

  // Called with coordinates produced by this plan and a validated source.
  // Separate from row iteration so dispatched implementations can reuse the
  // exact scalar border table while batching complete interior footprints.
  template <class T>
  std::optional<T> evaluate_result(span2d::Plane<const T> source, SamplingCoordinates q) const {
    const bool complete2 = q.i >= 0 && q.i < width_ - 1 && q.j >= 0 && q.j < height_ - 1;
    const bool complete4 = q.i >= 1 && q.i < width_ - 2 && q.j >= 1 && q.j < height_ - 2;
    if (class_ == SamplingClass::affine) {
      if (mode_ == 1 && complete2)
        return bilinear(source, q);
      if (mode_ == 2 && complete4)
        return cubic(source, q);
      return general(source, q.i, q.j);
    }
    q.j = reflect(q.j, height_, mirror_ & 1, mirror_ & 2);
    if (!valid(q.j, height_))
      return std::nullopt;
    const int row = static_cast<int>(q.j);
    if (mode_ == 0)
      return valid(q.i, width_) ? read(source, q.i, q.j) : horizontal(source, q.i, row, false, blur_);
    if (mode_ == 1) {
      if (q.j < height_ - 1)
        return q.i >= 0 && q.i < width_ - 1 ? bilinear(source, q) : horizontal(source, q.i, row, true, blur_);
      if (valid(q.i, width_))
        return read(source, q.i, q.j);
      return class_ == SamplingClass::translation ? horizontal(source, q.i, row, false, 0) : std::optional<T>{};
    }
    if (q.j >= 1 && q.j < height_ - 2) {
      if (q.i >= 1 && q.i < width_ - 2)
        return cubic(source, q);
      return valid(q.i, width_) ? read(source, q.i, q.j) : horizontal(source, q.i, row, false, blur_);
    }
    if (q.j == 0 || q.j == height_ - 2) {
      if (q.i >= 0 && q.i < width_ - 1)
        return near_edge(source, q);
      return q.i == width_ - 1 ? read(source, q.i, q.j) : horizontal(source, q.i, row, false, 0);
    }
    if (valid(q.i, width_)) {
      const auto value = read(source, q.i, q.j);
      return class_ == SamplingClass::translation ? value : T((std::uint32_t(value) + read(source, q.i, q.j - 1)) / 2);
    }
    return horizontal(source, q.i, row, false, 0);
  }

  template <class T>
  T evaluate(span2d::Plane<const T> source, SamplingCoordinates q) const {
    return evaluate_result(source, q).value_or(T(border_));
  }
  template <class T>
  void write_sample(span2d::Plane<const T> source, T& destination, SamplingCoordinates q, bool preserve) const {
    const auto result = evaluate_result(source, q);
    if (result)
      destination = *result;
    else if (!preserve)
      destination = T(border_);
  }
  template <class T>
  void validate(span2d::Plane<const T> source, span2d::Plane<T> output) const {
    static_assert(std::is_same_v<T, std::uint8_t> || std::is_same_v<T, std::uint16_t>);
    if ((std::is_same_v<T, std::uint8_t> && bits_ != 8) || (std::is_same_v<T, std::uint16_t> && bits_ == 8))
      throw std::invalid_argument("Depan sampling precision does not match storage");
    validate_plane(source);
    validate_plane(output);
    if (source.width() != width_ || source.height() != height_ || output.width() != width_ ||
        output.height() != height_ || active_rows_overlap(source, output))
      throw std::invalid_argument("Depan sampling storage mismatch or alias");
    const auto maximum = (1u << bits_) - 1;
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x)
        if (source.row(y)[x] > maximum)
          throw std::invalid_argument("Depan source sample exceeds precision");
  }
  template <class T>
  void render(span2d::Plane<const T> source, span2d::Plane<T> output, bool preserve = false) const {
    validate(source, output);
    for (int y = 0; y < height_; ++y)
      row_coordinates(
          y, [&](int x, const SamplingCoordinates& q) { write_sample(source, output.row(y)[x], q, preserve); });
  }
};
} // namespace neo_mv::depan
