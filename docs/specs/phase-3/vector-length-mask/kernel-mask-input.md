# Shared mask input, eligibility and numeric rules

Inputs are the vectors node's video description, its creation metadata, current vectors[n] properties and the mask parameters. Outputs are a fixed mask description, scene eligibility, normalized constants and fallback sample. The carrier's visible sample format and dimensions do not determine the mask format. No carrier pixels or Super image are read.

## Creation and current metadata

Require a positive vectors frame count. Read frame-zero analysis metadata with the Phase 1 [metadata-only decoder](../../phase-1/analyse/kernel-vector-validation.md). InvalidMetadata is a creation error; MetadataOnly is sufficient. Save every effective analysis scalar. Use W,H,Wr,Hr, Bx,By,Ox,Oy,Nx,Ny,p and analysis precision ba from the [public format](../../phase-1/analyse/data-format.md). Require Wr<=WB<=W and Hr<=HB<=H, with WB,HB from [grid resampling](kernel-grid-resampling.md). The decoder's scalar domains otherwise apply; a mask does not impose a render block-shape table or require Super sampling to be possible.

At output n, perform a vector-data read. InvalidMetadata means ineligible. For valid metadata, all effective scalars except Levels must equal their saved values, including DeltaFrame. A mismatch is a frame error even if arrays are unavailable; Levels may vary in its positive domain. Matching MetadataOnly is ineligible. Malformed complete arrays are errors. A Complete field is eligible precisely when the [scene classifier](../../phase-1/sc-detection/kernel-scene-classification.md) gives change=0 using the creation descriptor and configured thresholds.

Eligibility has no temporal-reference condition: n+d need not be inside the clip. Masks neither request that frame nor inspect its pixels. DeltaFrame affects OcclusionMask's direction only. Invalid current metadata cannot change the established output dimensions or fallback value.

The output is GRAY with actual dimensions Wr,Hr, frame count and frame rate from vectors. For ba=8..16 use that integer precision, with u8 storage at 8 and u16 at 9..16. For ba=32 use float32. Define M=2^ba-1 for integer masks, M=1 for float masks. Analysis chroma and analysis subsampling affect scene-threshold scaling only, not this grayscale output geometry.

## Parameters and arithmetic

Convert ml, gamma and scval to binary32 by round-to-nearest, ties-to-even, before checks. Require finite ml>0, finite gamma>=0 and finite scval. time is finite binary64 in [0,100]. thscd1 is int64 in [0,16320]; thscd2 is converted to binary32 and must be finite in [0,100]. Define

$$f=\operatorname{fl32}(1/ml),\qquad t=\operatorname{trunc}(\operatorname{fl64}(\operatorname{fl64}(time\,256)/100)).$$

Require finite f and all function-specific normalization constants at creation. Zero caused by floating underflow is allowed, with subnormal values preserved rather than flushed. Check these even if every requested frame might be ineligible. t is in [0,256]. All geometric/SAD products are exact integers until the explicitly stated conversion; do not overflow a native signed intermediate.

For a nonnegative computed score L, require finite L and define the block sample

$$Q(L)=\begin{cases}\operatorname{trunc}(\min(L,M))&\text{integer mask},\\\operatorname{fl32}(\min(L,M))&\text{float mask}.\end{cases}$$

Scores and required intermediate values must be finite before applying Q; overflow does not silently become M. Computed integer scores truncate; they are not rounded like scval or the resizer.

For an integer mask, the fallback sample is q=trunc(fl32(scval+0.5)); require the mathematical q in [0,M] before converting to storage. For a float mask the fallback is scval directly, with no [0,1] restriction. Ineligible frames fill every visible sample with this value, bypassing score generation and resizing.

## Power function

pow32 and pow64 operate on nonnegative finite bases and nonnegative finite exponents at the indicated precision. Define pow(0,0)=1, pow(x,0)=1, pow(0,g)=0 for g>0, and pow(x,1)=x exactly. Otherwise let r be the correctly rounded result of the real power at the specified precision. If r is non-finite, report a frame error. The permitted result is r or its immediately adjacent representable predecessor or successor at that precision, restricted to finite nonnegative values. For fixed input, choose a stable result independent of scheduling. Preserve subnormals.

Subsequent arithmetic, clipping, integer truncation and grid rounding must follow from that permitted power result. There is no independent final-pixel error allowance. Other score intermediates are still evaluated and checked when gamma=0. Occlusion events with an empty destination interval are discarded before evaluating their contribution.

## Examples

- An integer8 mask with scval=12.5 fills an ineligible frame with 13; scval=-1 gives trunc(-0.5)=0 and is accepted, whereas scval=-1.5 gives -1 and is rejected. Acceptance is on the converted result, not a separate raw scval interval.
- A float mask with scval=1.25 fills with 1.25. NaN or infinity scval is a creation error for every output type.
- A complete field at the last frame with d=1 can produce a mask if scene-eligible. A complete negative SAD is an error even on that last frame; a missing SAD array selects the fallback.
- With integer8 Q(127.5)=127. Float Q(0.5)=0.5. A non-finite score is an error, not a saturated pixel.
