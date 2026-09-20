# Temporal selection and accumulated transform

Inputs are output index n, clip length F, binary32 offset in [-10,10], motion-property frames, clip dimensions, aspect and field options. Outputs are either a bypass of clip[n], a selected source index s plus a six-coefficient sampling map, or a controlled error. No image sample is read by this operator.

Set I=ceil(offset) when offset>0, otherwise I=floor(offset), as an exact integer. Let s=n-I. If I=0 or s is outside [0,F), bypass to clip[n]. Do not read data properties, require field parity or construct a new diagnostic on this branch. An info wrapper is still applied as specified by [diagnostics](../depan-analyse/kernel-diagnostics.md).

Otherwise let forward=(I>0), f=fl32(fl32(offset+(forward?1:-1))-fl32(I)), a=pixaspect/(fields?2:1), cx=fl32(width)/2 and cy=fl32(height)/2. Start with identity. Consume data[k] in ascending order for

$$k=\min(s,n)+1,\ldots,\max(s,n).$$

Decode each tuple with the [motion-property contract](../depan-analyse/kernel-motion-properties.md). A malformed tuple is a frame error. On the first successfully decoded good=false tuple, reset the entire map to identity and stop consuming further tuples. Later data frames are semantically unused and cannot cause a malformed-data or dependency error. A malformed tuple before this stopping point still fails, even if a later tuple would be invalid.

For each good tuple, form Tk with [motion-to-coordinates](../depan-analyse/kernel-motion-transform.md), the same f, forward, a and center. Replace the accumulated map T by Tk composed with T. Every segment gets f; do not apply f only to an endpoint segment. Preserve all six coefficients throughout composition. Any used arithmetic failure is a frame error.

If fields=true and matchfields=true, read target parity from clip[n] or explicit tff. Add -0.5 to the final ty for top, +0.5 for bottom. This is done even after an invalid tuple reset the map to identity. No source-frame parity is required. If either flag is false, no parity is read. fields=true still halves a when matchfields=false.

The render branch always uses clip[s] for image samples and inherited properties. Identity resulting from invalid motion does not select clip[n] and does not bypass interpolation. Only the initial I/source-index test is a bypass.

## Examples

These examples use a 16x16 clip with enough frames for each stated in-range source.

- offset=1 gives I=1,s=n-1,f=1 and consumes only data[n]. dx=2,dy=r=0,z=1,a=1 gives X=x+2 on clip[n-1].
- offset=1.5 gives I=2,s=n-2,f=0.5. Two dx=2 translations contribute +1 each, so total X=x+2, not x+3.
- offset=-1.5 gives I=-2,s=n+2,f=-0.5. Consume data[n+1] then data[n+2]; two dx=2 translations give X=x-2. Ascending order is preserved for either offset sign.
- If the first of those tuples has good=0, the result is identity and the second tuple is unused. With fields=true,matchfields=true and bottom parity, identity is then changed to ty=+0.5.
- At n=0,offset=1 the result bypasses to clip[0], even if data[0] is malformed. At offset=0 every frame bypasses.
