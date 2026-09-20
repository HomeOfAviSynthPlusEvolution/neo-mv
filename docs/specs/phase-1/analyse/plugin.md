# Analyse

## Interface and parameters

Returns one video node under `clip`. Parameters below are in registration order. Numeric controls use int32 saturation except boolean parameters, which use integer truth semantics; node and string parameters are identified explicitly.

| Parameter | Default | Domain and effect |
| --- | --- | --- |
| super | Required node | Source of visible output and current/reference logical samples |
| blksize | Inherit Super | One/two integer values; [Super block pairs](../super/plugin.md) |
| levels | 0 | Positive requests a count; nonpositive offsets the computed maximum; formula in analysis kernel |
| search | 2 | 0 logarithmic, 1 square-ring exhaustive, 2 hexagonal, 3 multiscale, 4 horizontal, 5 vertical |
| searchparam | 2 | Effective max(1,value); range for layers above zero |
| pelsearch | SuperPel | Strictly positive range for level zero, in vector units |
| mvlambda | 1000 | Nonnegative prediction-distance weight before area/depth scaling |
| chroma | True | Include U/V SAD when Super provides chroma; GRAY forces false |
| delta | 1 | Nonzero signed frame offset; reference n+delta |
| lsad | 400 | Signed threshold controlling adaptive prediction penalty |
| plevel | 1 | 0,1,2; multiplies per-layer base penalty by 2^(l*plevel) |
| globalmv | True | Enable parent-derived global prediction |
| pnew | 25 | 0..256; candidate error penalty |
| pzero | Effective pnew | 0..256; zero-seed penalty |
| pglobal | 0 | 0..256; global-seed penalty; effective pzero when globalmv=false |
| overlap | Inherit Super | One/two values; 0..half block, chroma-aligned |
| badsad | 10000 | Signed raw-error threshold before scaling for bad-block expansion |
| badrange | 24 | For blocks exceeding the scaled badsad threshold Bs: positive multiscale expansion, negative ring expansion, zero skips that step; final pel rings still apply |
| meander | True | Alternate block dependency direction on odd rows |
| trymany | 0 | 0 disabled, 1 coarse layers only, 2 all layers |
| fields | False | Enable field metadata validation and applicable vertical half-pixel shift |
| tff | Omitted | Explicit first-frame top-field truth value; omission instead reads each frame's _Field |
| satd | False | Replace luma SAD with SATD; forbidden for block 16x2 |
| prefix | MVUtensils string | Names both input Super and output Analysis data |

Optional empty blksize/overlap arrays, if delivered by the host, inherit the respective Super pair. One value replicates; more than two fails. Reject invalid enumerations, penalty ranges, zero delta, nonpositive pelsearch, unsupported geometry, missing required Super data, or insufficient valid layers. Negative lsad/badsad/badrange are meaningful and are not rejected merely for their sign.

## Frame dependencies and fields

At creation read Super metadata from frame 0 and validate the [analysis geometry](kernel-analysis.md). Source/reference frames used together must have matching sample format and logical Super geometry. Frame n requests super[n] and, only if 0<=n+delta<N, super[n+delta]. Requests may execute out of order; analysis of one frame must not depend on results from another frame request.

Creation validation includes the [whole-domain sampling precondition](kernel-block-error.md#sampling-admissibility-and-geometry-errors) for every block/layer and every zero-seed shift admitted by the analysis kernel. The complete logical geometry, including phase read domains, must support the unchanged Omega in all enabled planes. This requirement does not depend on search settings, pixel values or reference availability. A later frame with incompatible logical geometry or an invalid memory view is a frame error before motion/error evaluation. A naturally aligned positive-stride view need not be SIMD-aligned; the [common view contract](../README.md#common-numerical-and-memory-contracts) applies.

With fields=false, ignore tff and _Field for motion. With fields=true, establish current field parity even at sequence boundaries. When tff is supplied, top(k)=bool(tff) XOR (k is odd). Otherwise the used frame must contain readable integer _Field; nonzero means top. When a reference actually exists and is used, SuperPel>1 and delta is odd, also establish reference parity and set finest-layer f=+pel/2 for current top/reference bottom, -pel/2 for current bottom/reference top, otherwise zero. Other layers/conditions use f=0. An unavailable reference requires no parity lookup or motion computation; only current parity is validated on that boundary path. Exported vy includes the computed shift; do not subtract it afterward.

## Output computation and properties

With an available reference run [analysis composition](kernel-analysis.md), using its error, prediction and search kernels. Output video information, frame count/rate and visible pixels equal super. Start properties from super[n], not the reference.

Write the [analysis scalar fields](data-format.md), replacing every listed key with one integer: Width/Height are Super working W,H; RealWidth/Height, padding, pel, ratios and precision come from Super; Levels is effective L; Chroma is requested chroma AND SuperChroma (false for GRAY); B/O/N are the target finest block grid; DeltaFrame is delta. Width/Height remain Super dimensions even when target grid coverage WB,HB differs.

On a successful analysis replace AnalysisVectors and AnalysisSAD with equal-length Nx*Ny int64 arrays in row-major order. Each packed vector contains the winning X/Y components and each SAD contains the raw metric without search penalties. No coarse-level vectors are concatenated. Preserve unrelated properties.

If n+delta is outside the sequence, write the same scalar metadata and preserve visible pixels, but ensure that AnalysisVectors and AnalysisSAD under the selected prefix are absent: remove either key if inherited from super[n]. Preserve other prefixes and unrelated properties. This unavailable field is distinct from measured zero motion. Invalid current Super/field data still fails.

## Example

Single-level GRAY8 4x4, block 4x4, overlap 0, pad 4,pel=1,fields=false,trymany=0,search=4,pelsearch=1,pnew=0: if current rows are [10,20,30,30] and reference rows [0,10,20,30], output has one packed vector [1], SAD [0], NBlkX=NBlkY=1. Visible output stays [10,20,30,30] on each row. A boundary frame without a reference keeps metadata but does not manufacture [0] arrays.
