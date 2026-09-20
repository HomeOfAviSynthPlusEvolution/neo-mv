# Recalculate

Returns a video-node array under `clip`, one output for each input member of `vectors`, preserving member order. Parameters are in registration order:

| Parameter | Default | Domain and effect |
| --- | --- | --- |
| super | Required node | New image samples and visible output |
| vectors | Required nonempty node array | Old analysis fields, one per output |
| thsad | 200 | Signed int32; scaled error threshold below/equal to which mapped vectors are not refined |
| smooth | True | Interpolate the old grid when true; nearest selection otherwise |
| blksize | Inherit Super | Target block pair from [allowed sizes](../super/plugin.md) |
| search | 2 | Int32 in 0..5; [candidate relation](../analyse/kernel-motion-search.md) |
| searchparam | 2 | Int32, effective max(1,value), used at the target pel |
| mvlambda | 1000 | Nonnegative int32 prediction penalty |
| chroma | True | Include chroma metrics; GRAY forces false |
| pnew | 25 | Int32 in 0..256; refinement error penalty |
| overlap | Inherit Super | Target overlap pair, each 0..half block and chroma-aligned |
| meander | True | Alternate evaluation direction; no change to independent target-block mathematical results |
| fields | False | Enables field validation described below |
| tff | Omitted | Explicit field parity instead of _Field lookup |
| satd | False | Luma SATD, except unsupported 16x2 blocks |
| prefix | MVUtensils string | Super and Analysis property prefix |

Axis-array decoding, including optional empty-array inheritance, follows Analyse. Boolean parameters use zero/nonzero semantics. An empty vectors list is outside the supported interface and must fail explicitly. A failed member creation makes the whole call fail; identify its zero-based member index instead of returning a successful partial array.

## Input compatibility and frames

For each member read vector frame 0's metadata at creation and save its delta d. Validate its analysis precision against the supplied Super. Do not require its old block size, grid, pel or actual dimensions to equal the target: they are inputs to remapping. Require valid positive old geometry and a valid target grid that fits the Super working image. Supported sequences keep each member's analysis geometry and precision fixed across frames; reject changes that invalidate those creation-time assumptions.

At creation, the target Super geometry must satisfy the [whole-domain sampling precondition](../analyse/kernel-block-error.md#sampling-admissibility-and-geometry-errors) for all target blocks. Validate all of Omega, regardless of the incoming vectors, thsad or search settings. An unsafe in-Omega candidate makes creation fail rather than being skipped or detected only during refinement. Incoming public-vector validation remains a separate requirement. A later incompatible Super geometry or invalid [memory view](../README.md#common-numerical-and-memory-contracts) is a frame error before motion/error evaluation.

For output frame n request vectors[n],super[n],super[k], where k=clamp(n+d,0,Nsuper-1). The member's saved d, not a subsequently changed frame property, determines k and output DeltaFrame. The current/reference Super formats and logical geometry must match. A vectors node must provide every requested n; failure to retrieve one is a frame error. Clamped reference positions are analyzed rather than treated as absent references.

## Computation and fields

Decode old data with [vector validation](../analyse/kernel-vector-validation.md). Invalid metadata or malformed complete arrays fails. MetadataOnly data uses zero old vectors for the [refinement kernel](kernel-vector-refinement.md), which remaps, freshly measures errors and conditionally searches. Do not pass missing arrays through unchanged.

fields=true requires old AnalysisPel>1 at creation. At frame evaluation establish current parity using the [Analyse field rule](../analyse/plugin.md), including its missing-_Field error when tff is omitted. If target SuperPel>1 and d is odd, also establish parity for reference frame k. This validation does not add a half-pixel shift to the remapped vector: the refinement equations already specify the complete displacement. fields=false does not require parity metadata.

## Output

Video information, length/rate, visible pixels and initial properties come from super[n], never vectors[n]. Replace all [analysis scalar fields](../analyse/data-format.md): working/actual dimensions, padding, pel, ratios and precision come from this Super; block/overlap/grid describe the target; Levels=1; DeltaFrame=saved d; Chroma=requested chroma (false for GRAY). If chroma is enabled require actual valid U/V Super samples; do not silently remove them from the request.

Replace both vector and SAD arrays with the target Nx*Ny results in row-major order. Recomputed SAD is always written even when a vector was retained without search. Unrelated super[n] properties remain unchanged. Separate outputs and simultaneous requests may share immutable inputs but not writable result storage.

Example: mapped vector (0,0) has old SAD 999, new measured SAD 100, threshold 100. Output vector stays (0,0) and SAD becomes 100. At n=0 with saved d=-1, compare against super[0]; this is not Analyse's missing-reference path.
