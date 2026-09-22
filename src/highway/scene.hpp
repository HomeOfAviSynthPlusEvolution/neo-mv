#pragma once
#include "core/motion/scene_classification.hpp"

namespace neo_mv::simd {
// SceneClassifier admits metadata and storage before invoking this counter.
// No early exit: malformed entries after an established scene cut still fail.
std::int64_t scene_count(const AnalysisMetadata&, const MotionGrid&, std::int64_t threshold);
std::int64_t scene_count_validated(const AnalysisMetadata&, const MotionGrid&, std::int64_t threshold);
} // namespace neo_mv::simd
