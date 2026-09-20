# Phase 2: block compensation and temporal denoising

Implement scalar operators in the dependency order below, then compose each plugin. The prerequisites are the [Phase 1 specifications](../phase-1/README.md), especially logical Super samples, typed analysis data, vector validation and scene classification.

| Order | Operators | Used by |
| --- | --- | --- |
| 1 | [Reference availability and render geometry](compensate/kernel-reference-availability.md) | Compensate, Degrain |
| 2 | [Block sampling](compensate/kernel-block-sampling.md) | Compensate, Degrain |
| 3 | [Overlap windows](compensate/kernel-overlap-window.md), [block composition](compensate/kernel-overlap-composition.md) | Compensate, Degrain |
| 4 | [Compensation selection](compensate/kernel-compensation.md) | [Compensate](compensate/plugin.md) |
| 5 | [Thresholds and reference weights](degrain/kernel-reference-weights.md), [weighted samples](degrain/kernel-weighted-samples.md), [change limiting](degrain/kernel-change-limit.md) | [Degrain and Degrain1 through Degrain25](degrain/plugin.md) |

## Common contracts

Use Phase 1's integer arithmetic, sample storage, natural alignment, positive byte-stride, ownership, error and concurrency requirements. Inputs are immutable; writable outputs and scratch belong to the current call. Kernels neither access host APIs nor create worker pools. Independently scheduled frames and plugin instances must yield the same results without shared writable numerical state.

In these documents fl32 and fl64 mean a separate round-to-nearest-even operation to binary32 and binary64. Unless explicitly specified otherwise, all floating intermediates must be finite; fused multiply-add and reassociation do not define the scalar baseline. Integer samples must lie in [0,2^b-1]. Float samples may lie outside [0,1]. Copying a sample or property preserves its value and type; generated float samples follow the specified arithmetic.

Trigonometric formulas define a scalar target at their stated precision P (binary32 or binary64). For the already-rounded angle u, let c*=RN_P(cos(u)), where the cosine is evaluated as a real function before rounding to P. A permitted cosine result is c*, its immediately preceding representable value at P, or its immediately following representable value at P, restricted to [-1,1]. This set defines the tolerance even where adjacent floating-point spacings differ. A chosen implementation must return the same permitted value for the same angle and precision on repeated calls.

Apply all subsequent arithmetic and quantization exactly as specified to that permitted cosine value. Different final integer values are allowed only when produced by propagating such a value through the complete formula; there is no separate permission to choose an adjacent integer. Other integer operations and typed property transport remain exact. Different SIMD paths must preserve the declared numerical contract; no optimization is required for this phase.

Both plugins output visible video and preserve clip[n]'s properties. Their kernels' blocks, coefficients, availability flags and weights are intermediate values, not new public frame properties. In particular, neither plugin recomputes or replaces inherited Analysis arrays or scene flags.

Analysis sample precision controls interpretation of stored SAD and threshold scaling. Render sample precision controls generated pixel values. These precisions may differ. Matching analysis geometry and matching render geometry are separately defined by the reference-availability kernel; do not compare the vector carrier's visible format to the render clip.

The plugins consume the supplied vector fields without searching for motion. Their whole-domain geometry checks use their own block-sampling map, which differs from the Phase 1 analysis-error map for negative chroma displacements. These checks do not change Phase 1's kernels or accepted typed-data format.
