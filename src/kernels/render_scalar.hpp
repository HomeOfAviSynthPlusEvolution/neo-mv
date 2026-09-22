#pragma once

#include "core/render/change_limit.hpp"
#include "core/render/compensation.hpp"
#include "core/render/overlap.hpp"
#include "core/render/weighted_samples.hpp"

namespace neo_mv {
// Admission and reference-selection rules stay in core; counting and pixel
// kernels can be selected independently of their shared orchestration.
template <class T>
struct ScalarRenderKernels {
  static constexpr auto scene_count = &scalar_scene_count;
  static constexpr auto sample_render_block = &neo_mv::sample_render_block<T>;
  static constexpr auto sample_compensated_block = &neo_mv::sample_compensated_block<T>;
  static constexpr auto weighted_render_block = &neo_mv::weighted_render_block<T>;
  static constexpr auto compose_render_blocks = &neo_mv::compose_render_blocks<T>;
  static constexpr auto limit_render_plane = &neo_mv::limit_render_plane<T>;
};
} // namespace neo_mv
