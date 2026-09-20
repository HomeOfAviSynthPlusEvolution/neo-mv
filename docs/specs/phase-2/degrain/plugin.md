# Degrain and Degrain1 through Degrain25

Returns one video node under `clip`. Degrain derives R from the number of input vector members; DegrainR requires exactly 2R members for its named R in [1,25]. Given the same valid arguments, both forms have identical pixels and properties. Parameters are in interface order:

| Parameter | Default | Domain and behavior |
| --- | --- | --- |
| clip | Required node | Output video information, properties and unprocessed planes |
| super | Required node | Centre and reference logical image samples |
| vectors | Required node array | 2R members, even count 2 through 50 |
| thsad | [400,400] | Nonnegative int64 pair for luma/chroma near thresholds |
| thsad2 | Effective thsad pair | Nonnegative int64 pair for far thresholds |
| planes | All actual planes | Integer indices 0,1,2 without duplicates |
| limit | [+infinity,+infinity] | Luma/chroma floating pair, converted to binary32; finite values must be >0 |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Converted to binary32; finite percentage in [0,100] |
| weights | 2R+1 ones | Integer user coefficients in the order defined below |
| prefix | MVUtensils | String selecting Super and Analysis property names |

Threshold/limit arrays accept one value replicated or two values for luma/chroma. An explicit empty array selects the same fallback as omission. More than two elements is an error. U and V share the chroma value. Reject malformed types; threshold values remain int64 without int32 saturation. Scale all near/far thresholds at creation, including unprocessed-plane values, and require results below 2147483647.

For planes, omission or an empty list selects all actual planes. Each explicitly given value must be 0,1 or 2 and cannot repeat. A permitted index absent from the actual format selects no plane; for example planes=[1] on GRAY processes none and leaves visible samples equal to clip. Validation of the other arguments and required input fields still applies.

weights must contain exactly 2R+1 integers when supplied; an empty array is an error. Read each with Phase 1's int32 saturation, then require 0<=u<=floor(2147483646/(256(2R+1))). The centre may have weight zero. There is no fields, time, satd or motion-search parameter.

## Pairing and creation

Let vectors=[a1,b1,...,aR,bR]. Save their creation-time deltas daj,dbj. Require daj!=0, dbj=-daj and |da(j+1)|>|daj|. Either sign may lead each pair. Perform negation and absolute-value checks in a wide type; an unrepresentable opposite delta is an error. No sorting or sign-based reordering is performed.

User weights=[aR,...,a1,centre,b1,...,bR]. This order follows pair positions, not a newly sorted time line. Derive normalized weights with the [weight kernel](kernel-reference-weights.md).

Establish the common [reference and render geometry](../compensate/kernel-reference-availability.md), validate current integer-phase samples' logical domains and [all public vectors' render footprints](../compensate/kernel-block-sampling.md) in every processed plane. All members share the analysis descriptor except their delta and positive Levels. Arrays may be absent in their frame-zero metadata. Negative thresholds, inconsistent descriptors/pairs, inadequate coverage, unsafe sampling geometry, invalid arguments or unrepresentable arithmetic are creation errors.

## Frame dependencies and computation

For frame n require clip[n], Super[n] and every vectors member's frame n. Decode and validate every current field; obtain its available flag with the common scene thresholds. A complete corrupt field or changed valid descriptor is a frame error, even if its user coefficient is zero. Missing arrays, invalid metadata, a scene change or n+d outside the sequence make that member unavailable according to the shared rules.

For each available member require and validate Super[n+d]. No out-of-range or unavailable reference image is required. Centre Super[n] is always required and must have valid geometry/storage. A failed required dependency is a frame error, not a zero-weight reference.

For each processed plane:

1. Compute pair thresholds and block reference weights from stored raw SAD values.
2. Generate [weighted samples](kernel-weighted-samples.md), using Super[n] as centre and available references at their supplied vectors.
3. Compose blocks into the visible plane with the [overlap rules](../compensate/kernel-overlap-composition.md).
4. Apply [change limiting](kernel-change-limit.md) relative to Super[n]'s visible integer-phase samples.

All-unavailable references and all-zero user coefficients do not bypass these operations. They give centre-only weights; overlap arithmetic and limit still apply. Unprocessed actual planes are copied exactly from clip[n].

## Output and independence

Output dimensions, sample format, frame count and frame rate equal clip. Copy all properties from clip[n] with their exact types and counts; add or replace no Analysis arrays, scene flags, Super metadata or diagnostic keys. Rendered centre samples may differ from clip[n] if the caller supplied a different compatible Super image.

Frame evaluation is independent of previously requested output frames. Read-only inputs and coefficient tables may be shared; writable generated blocks, accumulations and output planes cannot be shared between simultaneous calls. Allocation or numerical failure must not return a partly initialized successful frame.

## End-to-end examples

- GRAY8, R=1, one 8x8 block, overlap=0,p=1,pad=4, interior output frame, valid zero vectors with SAD=0, default thresholds and weights, no limit: C=100 and two reference constants 80,140 give output 107. Properties come from clip[n] even when its visible constant is 5.
- With only the 80-valued reference available, the same configuration gives 90. With both arrays absent, it gives centre 100. A negative SAD in a complete array gives an error instead of either output.
- R=2 with deltas [-1,+1,+4,-4] is accepted; [1,-1,1,-1] fails increasing distance, and [1,-2] fails symmetry. Degrain2 and Degrain agree for the accepted four-member input.
- With all weights zero, no overlap and integer render samples, processed output equals current Super; unprocessed planes still equal clip. A single-block grid with nonzero overlap also has unit outer windows.
- Using C=100 and the ordinary two-reference result 107, limit=2.2f produces 102; limit=0.1f produces 100; limit=0 fails creation.
