# DepanAnalyse

Converts block motion into global motion properties while preserving clip's video description and base pixels. Parameters in interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| clip | Required node | Constant video format/dimensions; pixels and inherited properties |
| vectors | Required node | Public analysis data; at least as many frames as clip |
| mask | Omitted | Optional integer8 first-plane fitting weights; see observations |
| zoom | true | Enable scale fitting after the first five updates |
| rot | true | Enable rotation fitting after the first five updates |
| pixaspect | 1.0 | Finite positive binary32 pixel aspect |
| error | 15.0 | Finite binary32 strict acceptance limit |
| info | false | Diagnostic property and host text rendering |
| wrong | 10.0 | Finite binary32 neighbor disagreement threshold |
| zerow | 0.05 | Finite binary32 multiplier for zero observations |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Binary32 finite percentage in [0,100] |
| fields | false | Use field aspect and vertical motion correction |
| tff | Omitted | Optional boolean field parity override |

There is no prefix parameter; read analysis keys beginning with `MVUtensils`. Depan output keys have no prefix. clip may use any constant format the host can copy, including RGB and float; its pixels are not used for fitting. vectors' carrier format and dimensions do not determine the output. No implicit format conversion is permitted.

## Creation

Validate node descriptions, parameters and [observation metadata](kernel-block-observations.md) from vectors[0], using metadata-only decoding without inspecting its arrays. Failure to retrieve that frame or invalid creation metadata is a creation error. Save d in {-1,+1} and the scaled scene thresholds. Set a=pixaspect/(fields?2:1), cx=fl32(width)/2 and cy=fl32(height)/2. Require finite positive a and finite nonzero a*a; require finite 2*error. Validate mask description even if no requested field later proves eligible. info=true requires the [diagnostic renderer](kernel-diagnostics.md).

## Frame n

Select vector index k=max(0,n-1) for the saved d=+1; k=n for the saved d=-1. Require clip[n], vectors[k] and, when supplied, mask[n] to be retrievable. A required frame failure is an output-frame error, including mask failure when the vector field would be ineligible. Begin with pixels and all properties from clip[n]. Decode vectors[k] using its own current metadata and the [observation rules](kernel-block-observations.md), retaining the creation-time numeric scene thresholds. Do not require current metadata to equal the creation descriptor. Current metadata determines array counts, vector bounds, grid, centers and pel; its DeltaFrame does not change the saved selection or conversion direction. Ineligibility skips fitting and retains an identity map, E=2*error and iter=0. Malformed complete arrays are errors. No pixel sample from vectors is consumed.

For eligible data, construct observations using mask[n] when supplied and run [global fitting](kernel-global-fit.md). For either eligibility branch, if E>=error export the standard invalid tuple. Otherwise for d=-1 convert the retained map to motion with forward=true; for d=+1 first apply the analysis inverse and convert it with forward=false. Both use clip's center and the same a. The positive-delta result therefore uses the preceding vector frame and reverses its map; changing only the sign of dx/dy is insufficient when rotation/scale are present. Because negative error is permitted, an ineligible field with error<0 passes E<error, converts its initial identity map and follows the valid-motion field-parity branch below. Do not force good=false merely because fitting was skipped.

For valid motion with fields=true, obtain parity from clip[n] or explicit tff using [field parity](kernel-motion-properties.md#field-parity). Add +1 to dy for top and -1 for bottom, in binary32. Do not scale all motion components by two and do not inspect vector-carrier parity. Invalid motion needs no parity. With fields=false, tff has no effect.

Replace the five [motion properties](kernel-motion-properties.md). No other input property is removed or modified. For info=true, replace the diagnostic key and invoke the renderer. For info=false, every visible output sample is exactly the corresponding clip[n] sample. Output property mutation must not modify clip or vectors. Only eligible fitting reads mask samples; retrieving a supplied mask[n] remains required for both eligibility branches.

## Examples

- At n=0,d=+1, read vectors[0]; there is no unconditional first-frame invalidation. At n=5,d=+1 read vectors[4]; at n=5,d=-1 read vectors[5].
- With all required frames retrievable, an ineligible field over an RGB or float clip with error>=0 preserves every base pixel and inherited property, replacing only the five motion keys with `(0,0,0,1,0)` when info=false. A missing `_Field` does not change this result.
- With a 48x48 clip, d=-1, pixaspect=1, an ineligible vector frame, error=-1 and fields=false, output motion is `(0,0,-0.0,1,1)`, E=-2 and iter=0. If fields=true and the target is top, dy becomes +1; a required missing `_Field` is a frame error when tff is omitted. No mask samples are read on either branch, but a supplied mask[n] must still be retrievable; otherwise output n is a frame error.
- With a complete scene-eligible zero-vector field, an all-zero mask and every block center inside that mask, error=15 and fields=false, the fit gives E=1,iter=10 and valid identity motion. With fields=true and top parity, dy becomes +1; bottom gives -1. With error=1 the output is instead invalid and no parity is read. This example uses clip width/height 16 and block size 8 with a 2x2 grid.
- A valid fitted translation `(tx,ty)=(2,0)` at a=1 becomes dx=2 for d=-1, but dx=-2 for d=+1 after inversion. These are conversion-stage examples, not an assertion that finite fitting always recovers an observation exactly.

### Complete fitting example

Use a 48x48 GRAY16 clip and a same-sized integer8 mask whose samples are all 1. The public analysis descriptor has working and real dimensions 16x16, block size 8x8, overlap zero, Nx=Ny=2, padding 16x16, pel=2, Levels=1, AnalysisChroma=0, plane ratios 1,1, analysis depth 8 and DeltaFrame=-1. Every vector frame, including vectors[0], carries this metadata and the row-major vector components [(1,-3),(0,0),(4,2),(-2,1)] with four zero SAD values. Use zoom=false, rot=false, fields=false, info=false and all other parameter defaults. The clip, vectors and mask each have exactly three frames; request n=2.

The observations are [(0.5,-1.5),(0,0),(2,1),(-1,0.5)]. Under the specified separately rounded binary32 recurrence, the final map has tx=0.49161040782928467, ty=0, u=h=1 and v=w=0 numerically; E=1.6047950983047485 and iter=10. Conversion at center (24,24) produces `(Depan_dx,Depan_dy,Depan_rot,Depan_zoom,Depan_goodmotion)=(0.4916095733642578,0,-0.0,1,1)`. The four floating properties contain exact float64 promotions of these binary32 results. Output pixels remain those of clip[2]. This example also distinguishes the final motion property from the fitted map's translation: preserve the specified center arithmetic and separate multiply/subtract rounding.
