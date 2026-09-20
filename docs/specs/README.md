# Implementation specifications

[Phase 1](phase-1/README.md): Super, Analyse, AnalyseMany, Recalculate, and SCDetection.

[Phase 2](phase-2/README.md): Compensate, Degrain, and Degrain1 through Degrain25.

[Phase 3](phase-3/README.md): VectorLengthMask, SADMask, OcclusionMask, and Flow.

[Phase 4](phase-4/README.md): FlowInter, FlowFPS, and FlowBlur.

Each function directory contains its mathematical operators in `kernel-*.md` and its public interface in `plugin.md`. Shared operators are referenced rather than duplicated. Implement the scalar definitions before adding optimized paths.
