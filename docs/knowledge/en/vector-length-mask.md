# VectorLengthMask: motion length as grayscale

## 1. What the function computes

Each block's vector length undergoes scale normalization, a power transform, and quantization before expansion into a pixel mask. SAD affects prior scene availability only, not the length formula.

## 2. Objects and notation

Vector vx, vy uses pel units, p per pixel. M is the integer maximum or floating 1. Shared f=fl32(1/ml), and gamma is binary32.

## 3. Overall calculation

Establish output using [shared input rules](shared/mask-input.md), decide field availability, fill scval if unavailable, otherwise compute and [resample](shared/grid-resampling.md) the length grid.

## 4. Step-by-step calculation

At creation calculate binary32 `f2=f·f,g2=gamma/2`. Per block convert to luma-pixel displacement:

$$d_x=fl64(v_x/p),\quad d_y=fl64(v_y/p),\quad q=fl64(fl64(d_xd_x)+fl64(d_yd_y)).$$

Then `a=fl64(q·fl64(f2))`, `L=fl64(M·pow64(a,fl64(g2)))`, and grid value Q(L). Mathematically this resembles `M·(length/ml)^gamma`, but f2, g2 binary32 rounding is retained; replacing it with an initial square root can change results.

Block origin, padding, and delta sign do not affect length. Time is validated but has no numerical role.

## 5. A complete numerical example

Valid GRAY8 single `8×8` block, p=1, vector (4, 0), SAD=0, ml=8, gamma=2 gives `f=1/8,f2=1/64,g2=1,q=16,a=0.25,L=63.75`. Truncation gives 63; the single-cell grid expands to constant 63.

Float32 analysis instead gives 0.25; 10-bit gives `trunc(1023/4)=255`. Precision comes from analysis, not carrier pixels.

## 6. Parameters and their calculation steps

| Parameter | Default | Role |
| --- | --- | --- |
| `vectors` | Required | Analysis; carrier pixels ignored |
| `ml` | 100.0, finite positive | Length scale |
| `gamma` | 1.0, finite nonnegative | Power |
| `time` | 100.0, finite 0–100 | Validation only |
| `scval` | 0.0, finite | Unavailable-field fill |
| `thscd1,thscd2` | 400, 51.0 | Scene availability |
| `prefix` | `MVUtensils` | Property names |

## 7. Boundaries, missing data, and errors

Zero vectors give zero for positive gamma; gamma=0 gives M even for zero vectors. Unavailable fields still use scval. Nonfinite intermediates fail rather than clipping overflow to white. Other rules are in the [shared article](shared/mask-input.md).

## 8. Precision and determinism

Quantize blocks before expansion. Score truncation, interpolation rounding, and scval rounding are distinct. Output properties contain only `_Range=[1]`.

[Back to the English index](README.md)
