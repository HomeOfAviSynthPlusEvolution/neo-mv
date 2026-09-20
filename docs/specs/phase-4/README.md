# Phase 4: temporal interpolation and motion blur

This phase implements FlowInter, FlowFPS and FlowBlur using the grid and phase-image contracts already established in Phase 3. Implement the shared scalar operators first, then their plugins. Optimized paths must preserve the specified operations and errors.

## Implementation order

1. [Paired vector inputs](flow-inter/kernel-pair-input.md): geometry, direction, field eligibility and dependency selection.
2. [Dense occlusion fields](flow-inter/kernel-dense-occlusion.md): shared motion fields and plane-specific mask resampling.
3. [Bidirectional sampling](flow-inter/kernel-bidirectional-sampling.md): main/extra sample positions and domain checks.
4. [Pixel composition](flow-inter/kernel-pixel-composition.md) and [fallback blending](flow-inter/kernel-fallback-blend.md).
5. [FlowInter](flow-inter/plugin.md): fixed relative-time interpolation.
6. [Frame-rate mapping](flow-fps/kernel-frame-rate-mapping.md) and [FlowFPS](flow-fps/plugin.md).
7. [Blur trajectories](flow-blur/kernel-trajectory.md), [sample accumulation](flow-blur/kernel-sample-average.md) and [FlowBlur](flow-blur/plugin.md).

## Common contracts

The [Phase 3 common contracts](../phase-3/README.md#common-contracts) apply to numeric notation, immutable data, memory views, independent requests, argument types and error handling. Integer calculations are exact with sufficiently wide intermediates or exact rearrangements. A convenient native type overflowing does not authorize rejecting a supported mathematical input. Sizes, final indices and storage extents must fit their declared representations before use. Phase 4 frame counts are in [1,2147483647], additionally subject to host support.

fl32 and fl64 denote independently rounded binary32 and binary64 operations, round-to-nearest, ties-to-even. No fused multiply-add, reassociation or subnormal flushing is part of the scalar baseline. Arithmetic image kernels require finite used float samples and finite intermediates, even for a sample multiplied by zero. Explicit sample-copy branches preserve the stored representation without arithmetic. There is no blanket final-pixel tolerance. All visible planes of the render clip are processed. No sample is read from row gaps or an undefined Super phase position.

All plugins return one video node under `clip`. Optional values default only when omitted. Malformed types are errors. Prefix defaults to `MVUtensils` and uses the existing public property naming. Unless a branch explicitly replaces a property, preserve the selected source map's types, counts and values. Required dependency failure, invalid used memory or allocation failure yields a controlled error, never a successful partial frame. Different inputs may have different valid positive strides.

No filter in this phase has a fields/tff argument or reads `_Field` to shift motion. Such properties may still be copied with their source frame. FlowInter and FlowFPS use floor(v*t/256) displacements; Flow's +128 displacement bias does not apply. FlowBlur defines its own integer trajectory. Creation validates geometry and zero-displacement image access; actual displaced coordinates are checked at frame evaluation, not by imposing a whole hypothetical vector-domain restriction.
