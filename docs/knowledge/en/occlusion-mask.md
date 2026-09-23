# OcclusionMask: convergence events between neighboring vectors

## 1. What the function computes

`OcclusionMask` compares horizontal or vertical vector components of neighboring blocks. A larger preceding component creates a convergence event over a grid interval. Overlapping events take the maximum, then expand to pixels.

This represents the defined vector-convergence calculation, not a per-pixel determination of real object occlusion. SAD affects prior scene checks only.

## 2. Objects and notation

Block steps sx, sy, pel p, saved signed delta d. Shared f=1/ml, t, M, Q are in [mask rules](shared/mask-input.md). Initialize grid G to zero.

## 3. Overall calculation

Decide availability; generate horizontal and vertical neighbor events; retain each cell's maximum quantized contribution; [resample](shared/grid-resampling.md). Unavailable fields fill scval.

## 4. Step-by-step calculation

Horizontal constants are `hx=trunc(16t/(sx·p))`, `ax=fl32(fl32(80f)/fl32(sx·p))`; vertical uses hy, ay.

For neighbors bx, by and bx+1, by, let o=vx_left−vx_right. No event for o≤0. Otherwise k=trunc(o·hx/4096), with inclusive target columns:

$$[l,r]=\begin{cases}[\max(0,b_x+1-k),b_x+1]&d>0\\[b_x,\min(b_x+1-k,N_x-1)]&d\le0.\end{cases}$$

If l>r, discard the empty event without swapping endpoints. For a nonempty event, with a=ax:

$$L=\begin{cases}fl32(fl32(M\,fl32(o))a)&\gamma=1\\fl32(M\,pow32(fl32(fl32(o)a),\gamma))&\gamma\ne1.\end{cases}$$

Update every target cell by `G=max(G,Q(L))`, retaining the special multiplication grouping for gamma=1.

Vertical events use upper/lower vy differences and by, Ny, hy, ay to write row intervals in one column. Horizontal neighbors never cross a row boundary, nor vertical neighbors a column bottom. The last row can still contain horizontal events and the last column vertical events.

## 5. A complete numerical example

GRAY8, two nonoverlapping horizontal `4×4` blocks, p=1, d=−1, X vectors `[4,0]`, Y/SAD zero, sufficient padding. Set ml=80, gamma=1, time=100.

f=1/80, t=256, hx=1024, o=4, k=1, interval `[0,0]`. ax=0.25, L=255, so grid `[255,0]` expands to each row `[255,255,223,159,96,32,0,0]`.

Changing d to positive gives interval `[0,1]`, all 255. Keeping d negative but o=8 gives k=2 and empty `[0,-1]`, all zero. Larger convergence does not necessarily cover more cells.

## 6. Parameters and their calculation steps

| Parameter | Default | Role |
| --- | --- | --- |
| `vectors` | Required | Neighbor differences and delta direction |
| `ml` | 100.0, finite positive | Event strength |
| `gamma` | 1.0, finite nonnegative | Power |
| `time` | 100.0, 0–100 | Interval length |
| `scval` | 0.0 | Unavailable-field fill |
| `thscd1,thscd2` | 400, 51.0 | Scene decision |
| `prefix` | `MVUtensils` | Analysis names |

## 7. Boundaries, missing data, and errors

time=0 makes k=0 but does not eliminate events: positive d writes the right neighbor; nonpositive d can write both current/right cells. d=0 uses the nonpositive branch without requesting a reference image. gamma=0 gives M only to actual nonempty events; untouched cells remain zero.

Nonfinite contributions and complete malformed data fail; missing/unavailable fields fill scval. Output properties are only `_Range=[1]`.

## 8. Precision and determinism

Interval products/differences use wide integers. Quantize each contribution before maximum combination. Horizontal/vertical events are not added; crossing full-strength events do not double intensity.

[Back to the English index](README.md)
