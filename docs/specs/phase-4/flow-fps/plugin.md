# FlowFPS

Changes frame rate using the Phase 4 bidirectional interpolator. Parameters in interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| clip | Required node | Source video, source frame rate, properties and fallback samples |
| super | Required node | Motion-path image samples |
| vectors | Required two-node array | Ordered [bw,fw] with positive/negative paired distance |
| num | 25 | Nonnegative int64 output-rate numerator; zero triggers source-rate doubling |
| den | 1 | Nonnegative int64 output-rate denominator; zero triggers source-rate doubling |
| extramask | True | Integer truth value enabling extra-vector composition |
| ml | 100.0 | Convert to binary32, finite and positive; occlusion normalization |
| blend | True | Integer truth value; original-image blend for unavailable main pairs |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Convert to binary32, finite in [0,100] |
| prefix | MVUtensils | String selecting public properties |

At creation apply [pair geometry and scene rules](../flow-inter/kernel-pair-input.md), [occlusion constant validation](../flow-inter/kernel-dense-occlusion.md), and [frame-rate mapping](kernel-frame-rate-mapping.md). Pair/Super compatibility and auxiliary lengths use the original Nv, not the output No. All checks apply even if every output will select an endpoint. Output dimensions, sample format and color family equal clip; frame count/rate are No and P/Q.

## Frame branches in order

1. Map output n to l,r,t. If t=0, copy visible pixels and initial properties from clip[min(l,Nv-1)]. If t=256, copy from clip[min(r,Nv-1)]. These branches do not read current vector arrays or Super images and ignore extramask/blend for pixel selection.
2. Otherwise require clip[min(l,Nv-1)] as the property source and apply [interpolation pair selection](../flow-inter/kernel-pair-input.md#interpolation-pair-selection). If the pair is usable, enable BB/FF only when extramask=true, build dense fields/masks, preflight all required positions and run the selected pixel-composition formula. Image sources are Super[l]/Super[r]. Extra fields require no more distant Super images.
3. If the main pair is unavailable or source pair indices are out of range, blend=true uses [ordinary blending](../flow-inter/kernel-fallback-blend.md) of clip[min(l,Nv-1)] and clip[min(r,Nv-1)]. blend=false copies clip[min(l,Nv-1)]. Both retain the left source's initial properties. Failed required validation or sampling does not select this fallback.
4. In every successful branch replace DurationNum/DurationDen as the mapping kernel requires, using the exact keys `_DurationNum` and `_DurationDen`. Preserve all other source properties unchanged, including any scene flags, analysis properties or existing diagnostics.

The endpoint branch precedes vector evaluation, unlike FlowInter at its fixed time endpoints. An endpoint can therefore succeed with malformed current vector arrays; valid creation metadata is still mandatory. extramask=false removes extra-field dependencies on every non-endpoint. With main fields unavailable, extra fields are also not required even when extramask=true.

## Examples

- Three GRAY8 8x8 source frames at 24 fps, matching one-block Super,pad=4,p=1,d=1, output48fps: No=6. At n=1,l=0,r=1,t=128, eligible zero main vectors, absent extra arrays and Super values 10/21 give output15. Properties start from clip[0], then duration becomes [1]/[48].
- At n=2,t=0, output copies clip[1], not Super[1]. Even if those images differ and vectors at frame 1 are malformed, this endpoint succeeds and replaces duration. A scene flag on clip[1] is retained.
- At n=5,l=2,r=3,t=128 the pair is outside the source range. blend=false copies clip[2]; blend=true blends clip[2] with itself. Both replace duration. Neither requests nonexistent vector frame3.
- In an interior interpolation, eligible main fields and malformed BB cause an error with extramask=true; extramask=false ignores BB entirely and uses basic composition. Missing BB arrays with extramask=true also select basic composition, provided both extra fields are otherwise valid or ordinarily ineligible.
