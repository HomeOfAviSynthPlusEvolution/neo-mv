#pragma once
#include "core/motion/prediction.hpp"

namespace neo_mv::simd {
void interpolate_predictions(const MotionGrid& parent, PredictionGeometry geometry, MotionGrid& child);
} // namespace neo_mv::simd
