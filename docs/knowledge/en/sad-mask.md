# SADMask: project block errors into grayscale

## 1. What the function computes

`SADMask` uses a block's motion vector to select another block's stored error, then normalizes that error to grayscale. It reads no images and measures no new SAD at the projected position.

Block error here means the stored `AnalysisSAD`, which may contain SAD, SATD, or DCT luma error plus enabled chroma SAD. This function does not recompute pixel SAD or convert between metrics; its existing threshold and scaling formulas apply to the stored value. See [analysis data](shared/analysis-data.md).

## 2. Objects and notation

Steps are sx=Bx−Ox, sy=By−Oy, pel p, analysis precision ba. f, t, M, Q follow [shared mask rules](shared/mask-input.md).

## 3. Overall calculation

Decide availability, project error indices, scale selected errors by depth/area, apply gamma and quantization, then [resample](shared/grid-resampling.md). Unavailable fields fill scval.

## 4. Step-by-step calculation

At creation:

$$h_x=trunc\frac{(256-t)16}{s_xp},\quad h_y=trunc\frac{(256-t)16}{s_yp},\quad
a=fl32\left(\frac{fl32(4f)}{fl32(B_xB_y)}\right).$$

For block bx, by, use its own vector:

$$i=b_x-trunc(v_xh_x/4096),\quad j=b_y-trunc(v_yh_y/4096).$$

If either index exceeds the grid, reset both to the current block. This is not independent axis clamping: a legal i with j=−1 still reads the current block, not column i on the current row.

For selected raw error S, first reduce toward an 8-bit integer scale:

$$s=\left\lfloor S/2^{\min(16,b_a)-8}\right\rfloor,\quad z=fl32(fl32(s)a),$$
$$L=fl32(M\,pow32(z,\gamma)),\quad G(b_x,b_y)=Q(L).$$

Float32 analysis also shifts by eight bits because errors are integer-encoded. Chroma participation adds no divisor here; it affects scene thresholds only.

## 5. A complete numerical example

GRAY8 single `8×8` block, p=1, zero vector, SAD=8, ml=1, gamma=1, time=100, passing scene checks. t=256, so hx=hy=0 and the block selects itself. a=4/64=1/16, z=0.5, L=127.5 truncates to 127. Resampling the single cell produces constant 127.

For projection, use three horizontal nonoverlapping `4×4` blocks with errors `[0,4,8]`, middle vector (4, 0), others zero; ml=1, gamma=1, p=1. time=0 gives hx=1024, so the middle selects the left error, giving grid `[0,0,255]`. time=100 gives `[0,255,255]` without projection.

## 6. Parameters and their calculation steps

| Parameter | Default | Role |
| --- | --- | --- |
| `vectors` | Required | Projection vectors and stored errors |
| `ml` | 100.0, finite positive | Error normalization |
| `gamma` | 1.0, finite nonnegative | Power |
| `time` | 100.0, 0–100 | Projection distance through 256−t |
| `scval` | 0.0 | Unavailable-field fill |
| `thscd1,thscd2` | 400, 51.0 | Scene decision |
| `prefix` | `MVUtensils` | Analysis names |

## 7. Boundaries, missing data, and errors

Out-of-grid projection returns to the block itself, but malformed input is not hidden by that branch. Complete negative SAD fails; absent arrays fill scval. No reference-frame existence check occurs. Output properties are only `_Range=[1]`.

## 8. Precision and determinism

Projection divides toward zero; depth reduction floors; Q truncates again. vx=−1, hx=1024 produces zero projected offset, not −1. Pixels interpolate the already quantized grid, not unquantized scores.

[Back to the English index](README.md)
