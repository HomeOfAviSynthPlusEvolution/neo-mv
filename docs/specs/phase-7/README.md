# Phase 7: global motion stabilization

Implement DepanStabilise. It consumes the public Depan motion properties and resamples a clip using either inertial smoothing or symmetric window smoothing. Optional neighboring images fill exposed borders. It does not estimate motion from image samples.

## Implementation order

1. [Parameters and coefficients](depan-stabilise/kernel-parameters.md).
2. [Motion intervals](depan-stabilise/kernel-motion-interval.md) and [cumulative transforms](depan-stabilise/kernel-cumulative-motion.md).
3. [Inertial smoothing](depan-stabilise/kernel-inertial-smoothing.md).
4. [Geometric zoom bound](depan-stabilise/kernel-zoom-bound.md) and [inertial adaptive zoom](depan-stabilise/kernel-inertial-zoom.md).
5. [Window smoothing](depan-stabilise/kernel-window-smoothing.md).
6. [End taper and correction limits](depan-stabilise/kernel-correction-limits.md), including [inertial numerical recovery](depan-stabilise/kernel-numerical-recovery.md).
7. [Neighbor selection](depan-stabilise/kernel-border-selection.md) and [layer composition](depan-stabilise/kernel-layer-composition.md).
8. [Diagnostics](depan-stabilise/kernel-diagnostics.md) and the [plugin interface](depan-stabilise/plugin.md).

Implement the scalar definitions first. Kernel outputs are mathematical interfaces, not additional public video nodes or prescribed storage layouts.

## Common contracts

The [Phase 5 common contracts](../phase-5/README.md#common-contracts) apply to memory, ownership, scalar arguments, floating arithmetic and elementary functions. In particular, use binary32 with independent nearest-even rounding after every operation, no contraction or reassociation, no subnormal flushing, and exact sufficiently wide integer geometry. Multiplication/division bind before addition/subtraction; equal-precedence operations associate left to right. Integer operands in floating expressions first convert to binary32. Preserve signed zeros. Square root is correctly rounded; the Phase 5 bounded elementary-function rule applies to trigonometric functions, logarithms and exponentials. A displayed decimal approximation in an example does not authorize a final-output tolerance.

Require natural sample alignment and positive byte strides divisible by sample size, with no SIMD alignment requirement. Source planes and output planes have independent strides. Do not read row gaps, mutate inputs, or return uninitialized samples. Output meaning must be independent of request order, parallelism, cache state and other filter instances.

Required arithmetic intermediates must be finite, divisors nonzero and square-root arguments nonnegative, except the explicitly capped lookback intermediate and the bounded [inertial numerical recovery](depan-stabilise/kernel-numerical-recovery.md) contract. Parameter-only failures are creation errors; unrecovered data-dependent failures are frame errors with no partial frame. Only that recovery contract permits non-finite internal values on the way to a finite current correction; maps admitted to neighbor selection and sampling must be finite. Recovery does not synthesize motion properties or create persistent scene state. Unused branches are not evaluated. Allocation and required dependency failures are controlled errors.

Use the [Phase 5 motion decoder](../phase-5/depan-analyse/kernel-motion-properties.md) with fixed, unprefixed property names. Frame zero is the explicit exception described in the interval kernel. `fields` changes the geometric aspect only: it creates no `_Field` dependency, parity alternation or half-line displacement.

The notation is shared throughout this phase: F is clip length, n is the requested frame, W,H are full-resolution dimensions, b is the final lower motion bound, h is its upper bound, a is effective aspect, z0 is reciprocal initial zoom, I is identity, C is cumulative motion, S is smoothed motion and Q is the final backward sampling map. A map's coefficient named h is distinguished from the interval endpoint by context. Definitions of M, Z, composition and Inv are in the cumulative-transform kernel.

Unless an example supplies different geometry, examples involving motion conversion use W=H=48, a=1 and center (24,24). In translation-tuple examples, unspecified displacement and rotation components are +0, zoom=1 and motion is valid unless marked invalid. A fixture given directly as map coefficients is used as supplied; it need not be regenerated through M. Exact identities in these small examples do not permit algebraic simplification of the general binary32 equations.
