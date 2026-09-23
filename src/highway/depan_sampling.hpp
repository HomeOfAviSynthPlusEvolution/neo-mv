#pragma once
#include "core/depan/sampling.hpp"
namespace neo_mv::simd::depan_rows {
// Admitted frame storage; affine coordinates are consumed in small batches.
void linear_render(const depan::SamplingPlan& plan, span2d::Plane<const std::uint8_t> source,
                   span2d::Plane<std::uint8_t> output, bool preserve);
void linear_render(const depan::SamplingPlan& plan, span2d::Plane<const std::uint16_t> source,
                   span2d::Plane<std::uint16_t> output, bool preserve);
// Output has plan.width() entries; arithmetic keeps separate binary32 rounding.
void coordinates(const depan::SamplingPlan& plan, int y, depan::SamplingCoordinates* output);
// Storage and coordinates are already admitted by the frame renderer.
void linear_row(const depan::SamplingPlan& plan, span2d::Plane<const std::uint8_t> source, std::uint8_t* output,
                const depan::SamplingCoordinates* coordinates, bool preserve);
void linear_row(const depan::SamplingPlan& plan, span2d::Plane<const std::uint16_t> source, std::uint16_t* output,
                const depan::SamplingCoordinates* coordinates, bool preserve);
} // namespace neo_mv::simd::depan_rows
