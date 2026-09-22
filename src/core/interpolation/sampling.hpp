#pragma once

#include "core/flow/sampling.hpp"
#include "core/interpolation/pixel.hpp"

namespace neo_mv {
template <class T>
struct InterpolationSamples {
  T A{}, C{}, A0{}, C0{}, E{}, K{};
};

// One visible plane. The frame caller preflights every plane before asking
// any plane to sample, so a late invalid coordinate cannot expose pixels.
class InterpolationSamplingPlan {
protected:
  FlowSamplingPlan left_, right_;
  int time_, bits_;
  struct Location {
    int phase, x, y;
  };
  struct Locations {
    Location A{}, C{}, A0{}, C0{}, E{}, K{};
  };
  static std::int64_t floor_div(std::int64_t value, int divisor) {
    // Positive-numerator division avoids signed-remainder optimization issues
    // at exact negative multiples and is defined even for INT64_MIN.
    return value >= 0 ? value / divisor : -1 - ((-1 - value) / divisor);
  }
  template <bool Validated = false>
  static Location locate(const RenderPhaseGeometry& g, int x, int y, std::int16_t vx, std::int16_t vy, int t) {
    const auto ax = std::int64_t(g.pel) * x + floor_div(std::int64_t(vx) * t, 256);
    const auto ay = std::int64_t(g.pel) * y + floor_div(std::int64_t(vy) * t, 256);
    const auto qx = floor_div(ax, g.pel), qy = floor_div(ay, g.pel);
    const int phase = static_cast<int>((ay - qy * g.pel) * g.pel + ax - qx * g.pel);
    const auto sx = g.pad_x + qx, sy = g.pad_y + qy;
    if constexpr (!Validated)
      if (sx < 0 || sy < 0 || sx >= g.phases[phase].width || sy >= g.phases[phase].height)
        throw std::invalid_argument("interpolation sample exceeds its logical phase domain");
    return {phase, static_cast<int>(sx), static_cast<int>(sy)};
  }
  void validate_fields(const DenseFlowField& B, const DenseFlowField& F, const DenseFlowField* BB,
                       const DenseFlowField* FF) const {
    if ((BB == nullptr) != (FF == nullptr))
      throw std::invalid_argument("interpolation extra fields must be paired");
    const auto count = std::uint64_t(width()) * height();
    for (const auto* field : {&B, &F, BB, FF})
      if (field && (field->width != width() || field->height != height() || field->x.size() != count ||
                    field->y.size() != count))
        throw std::invalid_argument("inconsistent interpolation dense field");
  }
  template <bool Validated = false>
  Locations positions(int x, int y, const DenseFlowField& B, const DenseFlowField& F, const DenseFlowField* BB,
                      const DenseFlowField* FF) const {
    const auto i = std::size_t(y) * width() + x;
    Locations out;
    out.A = locate<Validated>(left_.geometry(), x, y, F.x[i], F.y[i], time_);
    out.C = locate<Validated>(right_.geometry(), x, y, B.x[i], B.y[i], 256 - time_);
    if (BB) {
      out.E = locate<Validated>(left_.geometry(), x, y, FF->x[i], FF->y[i], time_);
      out.K = locate<Validated>(right_.geometry(), x, y, BB->x[i], BB->y[i], 256 - time_);
    } else {
      out.A0 = locate<Validated>(left_.geometry(), x, y, 0, 0, 0);
      out.C0 = locate<Validated>(right_.geometry(), x, y, 0, 0, 0);
    }
    return out;
  }
  template <class T>
  static void validate_image(const SubpixelPhases<T>& image, const RenderPhaseGeometry& geometry) {
    if (image.pel != geometry.pel)
      throw std::invalid_argument("interpolation image pel mismatch");
    for (int phase = 0; phase < geometry.pel * geometry.pel; ++phase) {
      const auto view = image.planes[phase];
      validate_plane(view);
      if (view.width() != geometry.phases[phase].width || view.height() != geometry.phases[phase].height)
        throw std::invalid_argument("interpolation phase storage mismatch");
    }
  }
  template <class T>
  T read(const SubpixelPhases<T>& image, Location position) const {
    const T value = image.planes[position.phase].row(position.y)[position.x];
    interpolation_detail::sample(value, bits_);
    return value;
  }

public:
  InterpolationSamplingPlan(RenderPhaseGeometry left, RenderPhaseGeometry right, int width, int height, int time,
                            int bits)
      : left_(left, width, height, 0), right_(right, width, height, 0), time_(time), bits_(bits) {
    interpolation_detail::controls(time);
    if (left.pel != right.pel || (bits != 32 && (bits < 8 || bits > 16)))
      throw std::invalid_argument("invalid interpolation image geometry or precision");
  }
  int width() const { return left_.width(); }
  int height() const { return left_.height(); }
  int time_coefficient() const { return time_; }
  void preflight(const DenseFlowField& B, const DenseFlowField& F, const DenseFlowField* BB = nullptr,
                 const DenseFlowField* FF = nullptr) const {
    validate_fields(B, F, BB, FF);
    for (int y = 0; y < height(); ++y)
      for (int x = 0; x < width(); ++x)
        (void)positions(x, y, B, F, BB, FF);
  }
  template <class T, bool Preflighted = false>
  std::vector<InterpolationSamples<T>>
  sample(const SubpixelPhases<T>& left, const SubpixelPhases<T>& right, const DenseFlowField& B,
         const DenseFlowField& F, const DenseFlowField* BB = nullptr, const DenseFlowField* FF = nullptr) const {
    mask_detail::validate_storage<T>(bits_);
    validate_image(left, left_.geometry());
    validate_image(right, right_.geometry());
    if constexpr (!Preflighted)
      preflight(B, F, BB, FF);
    const auto count = std::uint64_t(width()) * height();
    if (count > std::vector<InterpolationSamples<T>>().max_size())
      throw std::overflow_error("interpolation sample storage size is unrepresentable");
    std::vector<InterpolationSamples<T>> out(static_cast<std::size_t>(count));
    for (int y = 0; y < height(); ++y)
      for (int x = 0; x < width(); ++x) {
        const auto at = positions<true>(x, y, B, F, BB, FF);
        auto& value = out[std::size_t(y) * width() + x];
        value.A = read(left, at.A);
        value.C = read(right, at.C);
        if (BB) {
          value.E = read(left, at.E);
          value.K = read(right, at.K);
        } else {
          value.A0 = read(left, at.A0);
          value.C0 = read(right, at.C0);
        }
      }
    return out;
  }
};
} // namespace neo_mv
