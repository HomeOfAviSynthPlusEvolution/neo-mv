#pragma once

#include "core/super/border_extension.hpp"
#include "core/super/geometry.hpp"
#include "core/super/pyramid_reduction.hpp"
#include "core/super/subpixel.hpp"
#include "kernels/scalar.hpp"

namespace neo_mv {
namespace super_detail {
template <class T>
std::size_t sample_count(int width, int height) {
  if (width <= 0 || height <= 0)
    throw std::invalid_argument("invalid owned plane dimensions");
  const auto count = std::uint64_t(width) * height;
  if (count > std::uint64_t(PTRDIFF_MAX) / sizeof(T) || count > std::vector<T>{}.max_size())
    throw std::overflow_error("owned plane allocation is unrepresentable");
  return static_cast<std::size_t>(count);
}

// Views are obtained on demand, so copying/moving the owner never leaves stored
// pointers into another object's allocation. Natural alignment is sufficient.
template <class T>
class PlaneBuffer {
  int width_, height_;
  std::vector<T> data_;

public:
  PlaneBuffer(int width, int height) : width_(width), height_(height), data_(sample_count<T>(width, height)) {}
  span2d::Plane<T> view() {
    return checked_plane(data_.data(), width_, height_, std::ptrdiff_t(width_) * sizeof(T), data_.size() * sizeof(T));
  }
  span2d::Plane<const T> view() const {
    return checked_plane(data_.data(), width_, height_, std::ptrdiff_t(width_) * sizeof(T), data_.size() * sizeof(T));
  }
};
} // namespace super_detail

// Immutable geometry/control plan. Construction checks all geometry and buffer
// sizes without reading pixels or allocating the pyramid's sample buffers.
template <class T>
class SuperPlan {
  SuperGeometryParams params_;
  SuperGeometry geometry_;
  int bits_, sharp_, filter_;
  bool external_;

public:
  SuperPlan(SuperGeometryParams params, int bits, int sharp = 2, int filter = 1, bool external = false)
      : params_(params), geometry_(make_super_geometry(params)), bits_(bits), sharp_(sharp), filter_(filter),
        external_(external) {
    static_assert(supported_sample<T>, "unsupported Super storage");
    subpixel_detail::sample_max<T>(bits);
    if (sharp < 0 || sharp > 2 || filter < 0 || filter > 2)
      throw std::invalid_argument("invalid Super controls");
    for (int k = 0; k < geometry_.plane_count; ++k) {
      const auto& plane = geometry_.planes[k];
      const auto& base = plane.levels[0];
      if (params.pel > 1 && !external && (base.padded_width < 2 * (sharp + 1) || base.padded_height < 2 * (sharp + 1)))
        throw std::invalid_argument("padded plane too small for interpolation");
      if (params.pel > 1 && external) {
        geometry_detail::dimension(std::int64_t(params.pel) * plane.actual_width);
        geometry_detail::dimension(std::int64_t(params.pel) * plane.actual_height);
      }
      for (std::size_t l = 0; l < plane.levels.size(); ++l) {
        const auto& level = plane.levels[l];
        super_detail::sample_count<T>(level.padded_width, level.padded_height);
        if (l > 0) {
          super_detail::sample_count<T>(level.width, level.height);
          if (filter != 0)
            super_detail::sample_count<T>(geometry_detail::dimension(2LL * level.width), level.height);
        }
      }
    }
  }
  const SuperGeometryParams& params() const { return params_; }
  SuperPlan(const SuperPlan&) = default;
  SuperPlan(SuperPlan&&) noexcept = default;
  SuperPlan& operator=(SuperPlan other) noexcept {
    swap(*this, other);
    return *this;
  }
  friend void swap(SuperPlan& a, SuperPlan& b) noexcept {
    using std::swap;
    swap(a.params_, b.params_);
    swap(a.geometry_, b.geometry_);
    swap(a.bits_, b.bits_);
    swap(a.sharp_, b.sharp_);
    swap(a.filter_, b.filter_);
    swap(a.external_, b.external_);
  }
  const SuperGeometry& geometry() const { return geometry_; }
  int bits() const { return bits_; }
  int sharp() const { return sharp_; }
  int filter() const { return filter_; }
  bool external() const { return external_; }
};

// Owning, host-neutral logical payload. No global registry or borrowed input
// pixels; const phase views remain valid for the lifetime of this owner.
template <class T>
class SuperPyramid {
  SuperPlan<T> plan_;
  using Level = std::vector<super_detail::PlaneBuffer<T>>;
  std::array<std::vector<Level>, 3> planes_;

public:
  template <class Kernels = ScalarKernels<T>>
  SuperPyramid(const SuperPlan<T>& plan, const std::array<span2d::Plane<const T>, 3>& source,
               const std::array<span2d::Plane<const T>, 3>& external = {}, Kernels = {})
      : plan_(plan) {
    const auto& g = plan_.geometry();
    const int pel = plan_.params().pel;
    // Check all borrowed views before reading any plane's samples.
    for (int k = 0; k < g.plane_count; ++k) {
      const auto& p = g.planes[k];
      validate_plane(source[k]);
      if (source[k].width() != p.actual_width || source[k].height() != p.actual_height)
        throw std::invalid_argument("Super source dimensions mismatch");
      if (plan_.external() && pel > 1) {
        validate_plane(external[k]);
        if (external[k].width() != std::int64_t(pel) * p.actual_width ||
            external[k].height() != std::int64_t(pel) * p.actual_height)
          throw std::invalid_argument("Super external dimensions mismatch");
      }
    }
    const auto maximum = subpixel_detail::sample_max<T>(plan_.bits());
    for (int k = 0; k < g.plane_count; ++k) {
      const auto& p = g.planes[k];
      for (int y = 0; y < source[k].height(); ++y)
        for (int x = 0; x < source[k].width(); ++x)
          subpixel_detail::valid_sample(source[k].row(y)[x], maximum);
      auto& levels = planes_[k];
      levels.reserve(p.levels.size());
      for (std::size_t l = 0; l < p.levels.size(); ++l) {
        const auto& size = p.levels[l];
        levels.emplace_back();
        auto& phases = levels.back();
        phases.reserve(size.phase_count);
        for (int a = 0; a < size.phase_count; ++a)
          phases.emplace_back(size.padded_width, size.padded_height);
        if (l == 0)
          Kernels::extend_border_validated(source[k], phases[0].view(), size.width, size.height, p.pad_x, p.pad_y);
        else {
          super_detail::PlaneBuffer<T> working(size.width, size.height);
          if (plan_.filter() == 0) {
            Kernels::reduce_pyramid_validated(levels[l - 1][0].view(), p.pad_x, p.pad_y, working.view(), 0, {});
          } else {
            super_detail::PlaneBuffer<T> scratch(geometry_detail::dimension(2LL * size.width), size.height);
            Kernels::reduce_pyramid_validated(levels[l - 1][0].view(), p.pad_x, p.pad_y, working.view(), plan_.filter(),
                                              scratch.view());
          }
          Kernels::extend_border_validated(working.view(), phases[0].view(), size.width, size.height, p.pad_x, p.pad_y);
        }
      }
      auto& base = levels[0];
      std::array<span2d::Plane<T>, 16> storage{};
      for (int a = 1; a < pel * pel; ++a)
        storage[a] = base[a].view();
      if (plan_.external())
        Kernels::extract_external_subpixels(base[0].view(), external[k], p.actual_width, p.actual_height, p.pad_x,
                                            p.pad_y, pel, plan_.bits(), storage);
      else
        Kernels::interpolate_subpixels_validated(base[0].view(), pel, plan_.sharp(), plan_.bits(), storage);
    }
  }
  const SuperPlan<T>& plan() const { return plan_; }
  SuperPyramid(const SuperPyramid&) = default;
  SuperPyramid(SuperPyramid&&) noexcept = default;
  // Build the entire replacement before changing either geometry or samples.
  // An allocation failure must leave the old payload internally consistent.
  SuperPyramid& operator=(SuperPyramid other) noexcept {
    swap(*this, other);
    return *this;
  }
  friend void swap(SuperPyramid& a, SuperPyramid& b) noexcept {
    using std::swap;
    swap(a.plan_, b.plan_);
    swap(a.planes_, b.planes_);
  }
  span2d::Plane<const T> phase(int plane, int level, int ax = 0, int ay = 0) const {
    if (plane < 0 || plane >= plan_.geometry().plane_count || level < 0 || std::size_t(level) >= planes_[plane].size())
      throw std::out_of_range("Super plane or level out of range");
    const int pel = level == 0 ? plan_.params().pel : 1;
    if (ax < 0 || ax >= pel || ay < 0 || ay >= pel)
      throw std::out_of_range("Super phase out of range");
    const auto view = planes_[plane][level][ay * pel + ax].view();
    const bool quarter = pel == 4 && !plan_.external();
    return view.subplane(0, 0, view.width() - (quarter && ax == 3), view.height() - (quarter && ay == 3));
  }
};
} // namespace neo_mv
