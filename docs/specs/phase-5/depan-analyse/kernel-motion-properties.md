# Global motion properties and field parity

Input is a typed frame-property map. Output is either a decoded motion tuple `(dx,dy,r,z,good)` or a controlled data error. No image samples or private producer identifiers are consulted.

| Property | Producer type/count | Meaning |
| --- | --- | --- |
| `Depan_dx` | Float64, one | Horizontal motion, in full-resolution pixel units |
| `Depan_dy` | Float64, one | Vertical motion in aspect-normalized units; coordinate displacement also depends on pixel aspect |
| `Depan_rot` | Float64, one | Rotation in degrees |
| `Depan_zoom` | Float64, one | Multiplicative scale |
| `Depan_goodmotion` | Int64, one | 0 invalid, 1 valid |

A producer computes each floating result in binary32, then promotes it exactly to float64 for storage. Replace all five keys even when an inherited key has another type or count. The standard invalid tuple is `(0,0,0,1,0)`. A producer must specify whether it uses this tuple or preserves a rejected estimate; DepanAnalyse uses the standard tuple.

Consumers read element zero of all five keys, ignoring later elements. Missing, empty or wrongly typed keys are errors, including when goodmotion is zero. Require each floating value finite before conversion, round to binary32, then clamp dx/dy to [-1000000,1000000], rotation to [-360000,360000], and zoom to [fl32(0.01),100]. For a finite float64 whose binary32 rounding overflows, use the appropriate clamp endpoint; do not mistake this for a non-finite input. Any nonzero int64 goodmotion is true. Finite zero or negative zoom therefore becomes fl32(0.01). No consistency test compares the tuple to image pixels.

## Field parity

When a caller requires field parity at frame n, an explicitly supplied tff overrides frame properties: `top(n)=bool(tff) XOR (n is odd)`. Otherwise read integer element zero of `_Field` on the frame designated by that caller; nonzero means top. Missing, empty or wrongly typed data is a frame error. Do not infer parity from `_FieldBased`, frame rate or a previous request. When parity is not required, neither `_Field` nor tff creates a dependency or error.

## Examples

- `(2,-1,0,1,1)` with center (10,8) at aspect 1 produces a full-step translation (+2,-1). At aspect 2 the vertical coordinate translation is -2; dy is still -1.
- dx=2000000, zoom=-2 and goodmotion=-3 decode as dx=1000000, zoom=fl32(0.01), good=true.
- A missing rotation with goodmotion=0 is an error. A complete `(0,0,0,1,0)` is ordinary invalid motion.
- tff=true means top at n=0 and bottom at n=1 even if both frames carry `_Field=0`. Without tff, those two properties both mean bottom.
