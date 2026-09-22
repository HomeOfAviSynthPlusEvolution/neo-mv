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
  template <bool Validated = false>
  static Location locate(const RenderPhaseGeometry& g, int x, int y, std::int16_t vx, std::int16_t vy, int t) {
    const auto at = flow_coordinates::locate(g, x, y, flow_coordinates::floor_shift(std::int64_t(vx) * t, 8),
                                             flow_coordinates::floor_shift(std::int64_t(vy) * t, 8));
    const auto sx = at.x, sy = at.y;
    const int phase = at.phase;
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
    const flow_coordinates::CommonDomain left(left_.geometry()), right(right_.geometry());
    for (int y = 0; y < height(); ++y)
      for (int x = 0; x < width(); ++x) {
        const auto i = std::size_t(y) * width() + x;
        const auto admit = [&](const auto& domain, const auto& g, const DenseFlowField& field, int t) {
          if (!domain.contains_scaled(x, y, std::int64_t(field.x[i]) * t, std::int64_t(field.y[i]) * t))
            (void)locate(g, x, y, field.x[i], field.y[i], t);
        };
        admit(left, left_.geometry(), F, time_);
        admit(right, right_.geometry(), B, 256 - time_);
        if (BB) {
          admit(left, left_.geometry(), *FF, time_);
          admit(right, right_.geometry(), *BB, 256 - time_);
        }
      }
  }
  void preflight_generated(const DenseFlowField& B, const DenseFlowField& F, const DenseFlowField* BB = nullptr,
                           const DenseFlowField* FF = nullptr) const {
    validate_fields(B, F, BB, FF);
    const flow_coordinates::CommonDomain left(left_.geometry()), right(right_.geometry());
    const auto covered = [&](const auto& domain, const DenseFlowField& field, int time) {
      return field.generated_bounds && domain.covers_bounds(width(), height(), *field.generated_bounds, time, 0);
    };
    if (covered(left, F, time_) && covered(right, B, 256 - time_) &&
        (!BB || (covered(left, *FF, time_) && covered(right, *BB, 256 - time_))))
      return;
    preflight(B, F, BB, FF);
  }
  // Frame-only fused entry: image storage, fields and all plane coordinates
  // were admitted; masks match the visible dimensions and output is independent.
  template <class T, class MaskAllocator>
  void render_preflighted(const SubpixelPhases<T>& left, const SubpixelPhases<T>& right, const DenseFlowField& B,
                          const DenseFlowField& F, const DenseFlowField* BB, const DenseFlowField* FF,
                          const std::vector<std::uint8_t, MaskAllocator>& mF,
                          const std::vector<std::uint8_t, MaskAllocator>& mB, span2d::Plane<T> out) const {
    for (int y = 0; y < height(); ++y)
      for (int x = 0; x < width(); ++x) {
        const auto at = positions<true>(x, y, B, F, BB, FF);
        const auto i = std::size_t(y) * width() + x;
        const T A = read(left, at.A), C = read(right, at.C);
        if (BB)
          out.row(y)[x] =
              interpolation_extra<T, true>(A, C, read(left, at.E), read(right, at.K), mF[i], mB[i], time_, bits_);
        else
          out.row(y)[x] =
              interpolation_basic<T, true>(A, C, read(left, at.A0), read(right, at.C0), mF[i], mB[i], time_, bits_);
      }
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
