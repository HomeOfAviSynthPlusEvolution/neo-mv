# Paired vector geometry and eligibility

Inputs are clip and Super descriptions, exactly two vector nodes in ordered array [bw,fw], their creation descriptors, scene thresholds and selected vector frame indices. Outputs are a fixed render description, saved descriptors, positive temporal distance d and eligibility flags. There are no output image samples or property mutations.

## Creation

clip has fixed positive dimensions Wr,Hr and fixed GRAY or YUV420/422/440/444 format, integer8..16 or float32. Super matches actual dimensions, color family, sample type/depth, plane ratios and frame count Nv. Nv is in [1,2147483647]. Both vector nodes have at least Nv frames; their visible pixels and formats are irrelevant. Auxiliary frame rates need not match clip.

Use the [metadata-only decoder](../../phase-1/analyse/kernel-vector-validation.md) on bw[0] and fw[0]. InvalidMetadata is a creation error; valid MetadataOnly suffices without any frame-zero arrays. Save their effective scalars. Require bw DeltaFrame=+d, fw DeltaFrame=-d with 1<=d<=2147483647. Zero, reversed order or unequal magnitudes are creation errors. The two descriptors agree on every effective scalar except DeltaFrame and Levels; Levels is independently positive. No private producer identity or carrier payload may override public values.

Each descriptor must meet these render conditions:

- W,H,Wr,Hr,hp,vp,p match Super level zero.
- Bx,By use [Super's block-size table](../../phase-1/super/plugin.md). Overlaps meet public domains. Block dimensions/overlaps and actual dimensions are divisible by each actual render plane's ratios.
- With sx=Bx-Ox,sy=By-Oy, require Wr<=WB=Nx*sx+Ox<=W and Hr<=HB=Ny*sy+Oy<=H.
- AnalysisChroma=true requires a YUV render clip and equal analysis/render ratios. AnalysisChroma=false does not suppress rendered chroma or change its ratios.
- Analysis precision may differ from render depth. The positive Levels value does not select the render level: all image access uses level zero.

Every processed Super plane must expose the required logical phases and their defined domains. Require the integer-phase zero-displacement position to be sampleable at every visible plane pixel. Do not require every possible vector in a public block bound to be renderable. Actual sample preflight is defined by [bidirectional sampling](kernel-bidirectional-sampling.md) or [blur trajectories](../flow-blur/kernel-trajectory.md).

thscd1 is int64 in [0,16320]; thscd2 converts to binary32 and must be finite in [0,100]. Use fw[0]'s saved descriptor and the exact [scene-classification equations](../../phase-1/sc-detection/kernel-scene-classification.md) for both directions and all requested fields. In particular thresholds use analysis precision and chroma contribution, not render depth.

## Reading a selected field

For a selected node/frame, use a vector-data read. InvalidMetadata gives eligible=false. For valid metadata, all effective scalars except Levels must match that node's saved descriptor. Changed valid metadata is a frame error even with unavailable arrays; Levels may vary within its positive domain. Matching MetadataOnly gives eligible=false. Malformed complete arrays, negative SAD and out-of-public-bound vectors are errors. A matching Complete field is eligible iff the scene classifier gives change=0.

This eligibility does not check the field's own k+DeltaFrame. The caller selects which vector frames and images are required. In particular extra fields may be usable even when their nominal motion target lies outside the clip; they still sample only the two images identified below.

## Interpolation pair selection

Given l and r=l+d from a plugin, first check both indices against [0,Nv). If either is outside, select the whole-frame fallback without reading any vector fields or Super frames. clip fallback indices are clamped by the plugin. This boundary branch is distinct from the in-range validation sequence.

When both indices are in range, read and validate both main fields B=bw[l] and F=fw[r], even if the first is ineligible. A malformed second field is not hidden by the first field's scene change or absent arrays. If either is ineligible, select whole-frame fallback and do not require extra fields or Super images.

If both main fields are eligible and the plugin enables extra fields, read and validate both BB=bw[r] and FF=fw[l], even if one is ineligible. Use extra-field composition iff both are eligible; otherwise use basic composition. Malformed extra data is an error, not a reason to choose basic composition. When extra fields are disabled, they are not required and their data cannot affect the result.

The selected motion path uses only L=Super[l] and R=Super[r]. All used Super frames must match the creation format, geometry, logical phase domains and valid storage. There is no Super[l-d] or Super[r+d] image dependency. Dependency order may vary provided it does not add errors from inputs the selected branch does not require. Failure of any required dependency is a frame error.

## Examples

- Nv=5,d=1,l=2 gives main B=bw[2],F=fw[3], extras BB=bw[3],FF=fw[2], images Super[2]/Super[3].
- At l=4,r=5, fallback does not examine even bw[4]'s arrays. At l=3,r=4, a malformed main field is an error. If both main fields are eligible, a valid BB=bw[4] may participate despite its nominal target 5 being outside the clip.
- In-range B with absent arrays and F with complete negative SAD produces an error. In-range B with absent arrays and valid F selects fallback and does not inspect corrupt BB or FF.
- The array [fw,bw], [bw], a three-element array, or a pair with d=0 fails creation. Two valid metadata-only creation frames with d=+2/-2 are accepted.
