# OcclusionMask

Returns a grayscale mask of converging neighboring vectors. The interface, in order, is

`OcclusionMask(vectors, ml=100.0, gamma=1.0, time=100.0, scval=0.0, thscd1=400, thscd2=51.0, prefix="MVUtensils") -> clip`.

Argument types, conversions, domains, metadata requirements, scene eligibility, fallback, output video description and exact output-property rule are those of [VectorLengthMask](../vector-length-mask/plugin.md) and the [shared mask contract](../vector-length-mask/kernel-mask-input.md). Here ml normalizes convergence, time controls event extent, and the sign of DeltaFrame chooses the interval orientation in [the occlusion kernel](kernel-occlusion.md). Zero DeltaFrame uses the d<=0 case and is accepted. There is no reference-image dependency or temporal-boundary fallback.

At creation validate the event kernel's constants. Eligible frames generate its quantized block mask, then use [grid resampling](../vector-length-mask/kernel-grid-resampling.md). Ineligible frames fill every visible sample with converted scval. Complete-data errors, changed valid metadata and failed required arithmetic produce errors, not a mask fill. Result properties consist only of `_Range=[1]`.

For a complete GRAY8 analysis field with actual/working dimensions 8x4, Nx=2,Ny=1,4x4 blocks,overlap=0,pad=8,p=1,d=-1,SAD=[0,0],vx=[4,0],vy=[0,0], ml=80,gamma=1,time=100, the block grid is [255,0]. Every visible output row is [255,255,223,159,96,32,0,0]. This result also applies at n=0: there is no need for frame -1. A missing vector array with default scval instead produces all zero.
