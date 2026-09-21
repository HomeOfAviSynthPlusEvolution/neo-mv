# Phase 6: image-based global motion estimation

Implement DepanEstimate. It estimates translation, and optionally scale, from the first plane of consecutive images and writes the shared Depan motion properties. It does not consume block vectors or Super images. Stabilization belongs to the following phase.

## Implementation order

1. [Window geometry](depan-estimate/kernel-window-geometry.md): parameter normalization and admitted sampling rectangles.
2. [FFT execution](depan-estimate/kernel-fft-execution.md): precision, dependency configuration, ownership and concurrency.
3. [Window correlation](depan-estimate/kernel-fft-correlation.md): sample extraction, transform direction, complex product and normalization.
4. [Peak and confidence](depan-estimate/kernel-peak-confidence.md): ordered search, ties, mean and displacement penalty.
5. [Subpixel motion](depan-estimate/kernel-subpixel-motion.md): periodic interpolation, displacement limits, fields and aspect.
6. [One/two-window motion](depan-estimate/kernel-two-window-motion.md): translation/scale combination and frame-zero override.
7. [Temporal validity](depan-estimate/kernel-temporal-validity.md): nonrecursive neighboring confidence tests and final properties.
8. [Correlation display](depan-estimate/kernel-correlation-display.md) and [diagnostics](depan-estimate/kernel-diagnostics.md).
9. [DepanEstimate](depan-estimate/plugin.md): public arguments, dependencies, pixels and property changes.

Implement scalar operators first. Each operator has its own input/output contract; a public plugin need not expose their intermediate buffers or organize them as separate video nodes.

## Common contracts

The memory, ownership, argument and controlled-error rules in [Phase 5](../phase-5/README.md#common-contracts) apply. Source and output strides are independent; natural sample alignment and positive byte strides divisible by sample size are required, but SIMD alignment is not. No row gap is a sample. A controlled error produces no partially successful frame. Allocation or required dependency failure is an error. Cache state, request order and another filter instance must not alter the meaning of an output.

Integer geometry and counts are exact with sufficiently wide arithmetic. Unless stated otherwise, floating expressions outside FFT calls use binary32, rounding after each operation to nearest, ties to even, without contraction, reassociation or subnormal flushing. Integer operands convert to binary32 when combined with floating operands. Multiplication/division bind before addition/subtraction; equal-precedence operators associate left to right. Decimal constants denote their nearest binary32 values. fl32/fl64 mark explicit rounding boundaries. Required intermediates must be finite, and required divisors nonzero. Failure during parameter-only normalization is a creation error; otherwise it is a frame error. Do not convert an arithmetic failure into ordinary bad motion. Unused branches are not evaluated.

Float arguments must be finite binary64 before conversion, and their rounded binary32 values must remain finite before domain checks. All scalar arguments read element zero, ignoring later elements; empty or wrongly typed supplied values are errors. Defaults apply only when omitted. Boolean arguments use int64 zero/nonzero truth.

The [FFT execution contract](depan-estimate/kernel-fft-execution.md) defines the numerical boundary at the correlation surface. The ideal DFT equations fix meaning, direction and scale, not the last bit of a different FFT library. Given a concrete binary32 correlation surface, the downstream scalar operators and their decisions follow the equations here. Do not independently relax a confidence comparison, substitute another peak, or add a final-pixel tolerance. Copies and untouched samples/properties retain their stored values exactly.

All successful frames replace the five [Depan motion properties](../phase-5/depan-analyse/kernel-motion-properties.md). Rotation is always +0. Invalid final motion uses (0,0,0,1,0). No prefix argument or externally supplied intermediate spectrum is part of this interface.
