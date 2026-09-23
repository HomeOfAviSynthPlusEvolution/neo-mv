# FlowInter: compose two frames at a fixed relative time

## 1. What the function computes

For output n, `FlowInter` selects left n and right n+d and performs bidirectional motion sampling with occlusion-aware composition at a fixed time. Output frame count/rate match clip; it does not resample the output timeline.

## 2. Objects and notation

vectors is the signed-distance pair `[bw,fw]`, d>0, l=n, r=n+d. Time first converts to binary32, then:

$$t=trunc(fl32(fl32(time\cdot256)/100)).$$

Unlike Flow, this conversion uses binary32 rather than binary64. t lies in 0–256.

## 3. Overall calculation

Select frames and main/extra fields; build dense motion and occlusion; validate and sample Super phases; use ordinary/extra composition. An out-of-range pair or unavailable main field selects original-image blending or left-image copying through blend.

## 4. Step-by-step calculation

Main fields are bw[l], fw[r]. FlowInter always attempts extras bw[r], fw[l]; there is no extramask parameter. See [bidirectional interpolation](shared/bidirectional-interpolation.md) for complete selection, coordinates, and formulas.

The available path reads only Super[l]/Super[r]. Extra fields do not extend image distance. Every actual plane is processed even with luma-only analysis.

For boundary/unavailable-main fallback, blend=true combines clip[l] and clip[min(r, Nv−1)] with t; blend=false copies clip[l]. All branches inherit clip[l] properties without changing duration.

Time 0 or 100 does not bypass motion: main/extra fields and every selected-mode sample position are still validated and formulas run. Endpoints are not unconditional input copies.

## 5. A complete numerical example

Three GRAY8 frames, one nonoverlapping `8×8` block, p=1, pad=4, d=1, n=0. Main fields have zero vectors/SAD; both extras have metadata only. time=50 gives t=128.

Main fields are available and extras unavailable, selecting ordinary mode. Zero vectors produce no convergence, mB=mF=0. With Super[0]=10, Super[1]=21, A=10, C=21 gives U=11, V=22 and `floor((11·128+22·128)/256)-1=15`. Properties come from clip[0] even if its pixels differ from 10.

At the last frame r is out of range. blend=true blends the last original image with itself; false copies it exactly.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,super,vectors` | Required; vectors exactly `[bw,fw]` | Output, samples, fields |
| `time` | 50.0; finite binary32 in 0–100 | Relative time |
| `ml` | 100.0; finite positive binary32 | Occlusion strength |
| `blend` | true | Original-image fallback blend |
| `thscd1,thscd2` | 400, 51.0 | Scene decision |
| `prefix` | `MVUtensils` | Data names |

## 7. Boundaries, missing data, and errors

Bounds precede per-frame vector reads. In range, both main fields are checked, so one missing field cannot hide corruption in the other. Unavailable main fields skip extras. Missing extras select ordinary mode; malformed extras fail. Required Super/sampling errors do not become blending fallback.

## 8. Precision and determinism

Time uses binary32, displacement unbiased floor, occlusion maximum 255, and integer H bias 256. Each has its own scale. Outputs are independent with no accumulated prior motion state.

[Back to the English index](README.md)
