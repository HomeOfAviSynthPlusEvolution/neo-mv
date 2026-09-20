# Phase 3: masks and single-reference Flow

This batch implements VectorLengthMask, SADMask, OcclusionMask and Flow. The masks convert a public block field into a visible grayscale image. Flow converts a block field into a dense displacement field and samples one reference image. Bidirectional interpolation, frame-rate conversion, motion blur and stabilization belong to later batches.

## Implementation order

1. [Grid resampling](vector-length-mask/kernel-grid-resampling.md): shared scalar operator for unsigned masks, float masks and signed displacement components.
2. [Mask input and numeric contract](vector-length-mask/kernel-mask-input.md): public data validation, scene eligibility, normalization and output conversion.
3. [Vector length](vector-length-mask/kernel-vector-length.md) and [VectorLengthMask](vector-length-mask/plugin.md).
4. [Projected SAD](sad-mask/kernel-sad-projection.md) and [SADMask](sad-mask/plugin.md).
5. [Occlusion events](occlusion-mask/kernel-occlusion.md) and [OcclusionMask](occlusion-mask/plugin.md).
6. [Dense vector field](flow/kernel-dense-vector-field.md), [Flow sampling](flow/kernel-flow-sampling.md) and [Flow](flow/plugin.md).

Implement and verify the scalar contracts before optimizing them. A library or SIMD implementation must satisfy these equations, rounding rules and domains. The resampling contract is fully defined here; no external resizer's unspecified defaults determine the result.

## Common contracts

Use Phase 1's [numeric and memory contract](../phase-1/README.md#common-numerical-and-memory-contracts), [public analysis format](../phase-1/analyse/data-format.md), [decoder](../phase-1/analyse/kernel-vector-validation.md) and [scene classifier](../phase-1/sc-detection/kernel-scene-classification.md). Integer operations in formulas are exact with sufficiently wide intermediates; reject unrepresentable sizes, offsets or required results before conversion or allocation. Division inside floor or trunc is a mathematical quotient. Other explicitly floating operations round at each indicated fl32 or fl64 boundary, using round-to-nearest, ties-to-even, without contraction.

Inputs are immutable. Generated outputs must not overlap inputs or another concurrent invocation's writable storage. Memory views require natural sample alignment, positive byte stride divisible by sample size and sufficient row capacity; SIMD alignment is unnecessary. No access to row gaps or undefined phase samples is permitted. Row gaps have no output-value contract. Allocation/dependency failure is a controlled error, with no successful partial output. Independent and repeated frame requests must give the same allowed result without relying on previous requests.

Masks use analysis dimensions and precision; Flow uses render dimensions and precision. A host unable to represent the requested sample format must report a creation error. No implicit depth or color conversion is permitted. All four interfaces return one node under `clip`. Optional arguments default only when omitted; malformed argument types are errors. Integers in plugin interfaces are int64 unless specified otherwise; boolean parameters use zero/nonzero truth. Prefix is a string concatenated with the public property suffixes, without another separator.

The individual documents include normative examples at both kernel and plugin boundaries. Unless a transcendental result is involved, stated integer results are exact. The power-function allowance is defined in the mask input contract; it is not a blanket tolerance on final pixels.
