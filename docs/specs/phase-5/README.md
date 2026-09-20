# Phase 5: global motion analysis and compensation

Implement DepanAnalyse and DepanCompensate. The first converts a public block-vector field into global motion properties. The second uses those properties to sample a temporally selected image. Implement scalar operators in the order below before optimizing them.

## Implementation order

1. [Motion properties](depan-analyse/kernel-motion-properties.md): typed transport, validity, units and field parity.
2. [Motion transforms](depan-analyse/kernel-motion-transform.md): motion/coordinate conversion, composition and inverse.
3. [Block observations](depan-analyse/kernel-block-observations.md), [block weights](depan-analyse/kernel-block-weights.md) and [global fitting](depan-analyse/kernel-global-fit.md).
4. [Diagnostics](depan-analyse/kernel-diagnostics.md) and [DepanAnalyse](depan-analyse/plugin.md).
5. [Temporal transform](depan-compensate/kernel-temporal-transform.md).
6. [Sampling coordinates](depan-compensate/kernel-sampling-coordinates.md) and [border sampling](depan-compensate/kernel-border-sampling.md).
7. [Nearest](depan-compensate/kernel-nearest.md), [bilinear](depan-compensate/kernel-bilinear.md) and [bicubic](depan-compensate/kernel-bicubic.md) sampling.
8. [DepanCompensate](depan-compensate/plugin.md).

## Common contracts

The [Phase 3 common contracts](../phase-3/README.md#common-contracts) apply to memory views, immutable inputs, independent requests, argument types and errors. Source and destination strides are independent. Require natural sample alignment and positive byte strides divisible by sample size; no SIMD alignment is required. Each output sample is initialized. Row gaps have no value contract and are never sample inputs. Allocation failure, required dependency failure and invalid used memory produce controlled errors.

All frame counts are in [1,2147483647], subject to host support. Preserve clip dimensions, format, rate and frame count. Unsupported host formats are creation errors. Optional parameters default only when omitted; booleans use int64 zero/nonzero truth. A scalar parameter consumes element zero; an empty or wrongly typed supplied value is an error. Both plugins return one video node under `clip`. Neither has a public prefix parameter. Analysis inputs use the fixed `MVUtensils` prefix; Depan property names are fixed and unprefixed.

Unless stated otherwise, floating expressions use binary32, with each arithmetic operation rounded independently to nearest, ties to even; integer operands are converted to binary32 when combined with a floating operand. Multiplication/division bind before addition/subtraction, and operations of the same precedence associate left to right. Decimal constants denote their nearest binary32 values. No contraction, reassociation or subnormal flushing is allowed in the scalar definition. `fl32` and `fl64` explicitly identify other conversion/rounding boundaries. Integer geometry, squares, products, sums and divisions inside floor/trunc are exact. Do not inherit a narrow integer overflow from a convenient implementation type.

Floating parameters are first required to be finite binary64 and converted to binary32 nearest-even; require the converted value finite before applying the parameter's domain. Require finite used arithmetic intermediates, nonzero divisors, and nonnegative square-root arguments. A failure is a controlled creation error for parameter-only normalization, otherwise a frame error. Unused branches are not evaluated. Do not replace an arithmetic failure by invalid-motion output. Signed zero is retained by arithmetic and formatted diagnostics; numeric comparisons treat both zeros equally.

Square root is correctly rounded binary32. For sin, cos, atan, log and exp, let r be the correctly rounded binary32 result at the specified binary32 argument. Permit r or either immediately adjacent finite representable value, with the function's mathematical sign/range preserved. Require log's argument positive; reject a non-finite correctly rounded result. Fix sin(0)=0 with its sign, cos(0)=1, atan(0)=0 with its sign, log(1)=+0 and exp(0)=1 exactly. Choose deterministic results independent of scheduling. Subsequent operations, decisions and quantization must follow from that choice; there is no separate final-pixel or property tolerance. This permits bounded math-library variation without requiring an identical library build.

Image passthrough preserves stored sample bits, including float NaNs, without interpreting them. DepanAnalyse's numerical inputs are vectors, errors and an optional integer mask, not clip pixels. DepanCompensate's renderer accepts integer samples only. Extra inherited properties retain their names, types, counts and values unless a plugin explicitly replaces them. No implicit range, color-matrix or scene-tag rewrite is performed.
