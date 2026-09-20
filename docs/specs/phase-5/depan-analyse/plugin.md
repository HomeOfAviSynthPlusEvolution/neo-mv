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

Validate node descriptions, parameters and [observation metadata](kernel-block-observations.md). Save d in {-1,+1} and the scaled scene thresholds. Set a=pixaspect/(fields?2:1), cx=fl32(width)/2 and cy=fl32(height)/2. Require finite positive a and finite nonzero a*a; require finite 2*error. Validate mask description even if no requested field later proves eligible. info=true requires the [diagnostic renderer](kernel-diagnostics.md).

## Frame n

Select vector index k=max(0,n-1) for d=+1; k=n for d=-1. Begin with pixels and all properties from clip[n]. Decode vectors[k] under the saved descriptor before choosing an eligible branch. Ineligibility gives the standard invalid tuple, E=2*error and iter=0. Malformed complete arrays or changed valid metadata are errors. No pixel sample from vectors is consumed.

For eligible data, construct observations using mask[n] when supplied and run [global fitting](kernel-global-fit.md). If E>=error, export the standard invalid tuple. Otherwise for d=-1 convert the fitted map to motion with forward=true; for d=+1 first apply the analysis inverse and convert it with forward=false. Both use clip's center and the same a. The positive-delta result therefore uses the preceding vector frame and reverses its map; changing only the sign of dx/dy is insufficient when rotation/scale are present.

For valid motion with fields=true, obtain parity from clip[n] or explicit tff using [field parity](kernel-motion-properties.md#field-parity). Add +1 to dy for top and -1 for bottom, in binary32. Do not scale all motion components by two and do not inspect vector-carrier parity. Invalid motion needs no parity. With fields=false, tff has no effect.

Replace the five [motion properties](kernel-motion-properties.md). No other input property is removed or modified. For info=true, replace the diagnostic key and invoke the renderer. For info=false, every visible output sample is exactly the corresponding clip[n] sample. Output property mutation must not modify clip or vectors. Only eligible fitting requires mask[n]; unused frame dependencies must not introduce errors.

## Examples

- At n=0,d=+1, read vectors[0]; there is no unconditional first-frame invalidation. At n=5,d=+1 read vectors[4]; at n=5,d=-1 read vectors[5].
- An ineligible field over an RGB or float clip preserves every base pixel and inherited property, replacing only the five motion keys with `(0,0,0,1,0)` when info=false. A missing `_Field` does not change this result.
- With a complete scene-eligible zero-vector field, an all-zero mask and every block center inside that mask, error=15 and fields=false, the fit gives E=1,iter=10 and valid identity motion. With fields=true and top parity, dy becomes +1; bottom gives -1. With error=1 the output is instead invalid and no parity is read. This example uses clip width/height 16 and block size 8 with a 2x2 grid.
- A valid fitted translation `(tx,ty)=(2,0)` at a=1 becomes dx=2 for d=-1, but dx=-2 for d=+1 after inversion. These are conversion-stage examples, not an assertion that finite fitting always recovers an observation exactly.
