#pragma once
#include "core/depan/sampling.hpp"
namespace neo_mv::simd::depan_rows {
// Output has plan.width() entries; arithmetic keeps separate binary32 rounding.
void coordinates(const depan::SamplingPlan& plan, int y, depan::SamplingCoordinates* output);
} // namespace neo_mv::simd::depan_rows
