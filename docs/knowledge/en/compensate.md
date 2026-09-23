# Compensate: select and sample a reference block

## 1. What the function computes

`Compensate` uses each block's raw error to choose reference Super or current Super, samples a complete block at the selected displacement, then combines blocks through overlap windows. It neither estimates new vectors nor continuously blends the two images according to SAD.

## 2. Objects and notation

Current frame n, saved reference offset d, vector vx, vy, and error s. t is time quantized to 0–256; T is the scaled block threshold; f is the vertical field shift.

## 3. Overall calculation

Validate geometry and current Super, decode vectors, and decide whole-frame reference availability. If available, obtain reference and required parity and validate actual footprints. Select and sample current/reference blocks, then [compose them](shared/block-rendering.md). If unavailable, return clip for the whole frame.

## 4. Step-by-step calculation

### 4.1 Time and threshold

$$t=trunc(fl64(fl64(time\cdot256)/100)),\quad T=S(thsad).$$

S is the [analysis-scale conversion](shared/block-rendering.md), using analysis depth rather than output depth.

### 4.2 Field shift

`fields=true` requires p>1. Only odd d needs current/reference parity. Explicit tff gives `top(k)=bool(tff) XOR (k is odd)`; otherwise `_Field` element 0 is read, with nonzero meaning top.

Current top/reference bottom gives f=p/2; the reverse gives −p/2; otherwise zero. Even d, disabled fields, or an unavailable whole-frame reference needs no parity. The first two cases set f=0.

### 4.3 Selecting the image and position

If s<T, use `Super[n+d]` with:

$$d_x=trunc(v_xt/256),\qquad d_y=trunc(v_yt/256)+f.$$

Otherwise use `Super[n]` at displacement (0, f). Equality selects current Super. Field shift is added after time scaling and is not multiplied by t.

Thus time=0 can still read the reference image with zero vector displacement. A high-error block can still have a half-pixel field shift in current Super. Neither case is generally a copy of `clip[n]`.

### 4.4 Sampling and composition

Use [block sampling with floored total coordinates](shared/block-rendering.md) on every actual rendering plane. Luma-only analysis does not skip chroma rendering. Generate complete blocks, apply overlap composition, and crop to the visible image. All output properties come from clip; vector and scene properties are not rewritten.

## 5. A complete numerical example

One nonoverlapping GRAY8 `8×8` block, `p=1,pad=4,d=1`, current Super constant 10, reference constant 80, clip constant 5. Let `thsad=100`, no analysis chroma, and passing scene checks: T=100.

Error 99 selects the reference and zero motion gives 80. Error 100 selects current Super and gives 10. Missing vector arrays cause whole-frame fallback to clip's 5. All three inherit clip properties.

Changing time to zero in the first case still gives 80. Time changes displacement, not the identity of the reference image.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,super,vectors` | Required nodes | Output description, samples, analysis |
| `thsad` | 10000; nonnegative int64, scaled result representable | Binary block choice |
| `fields` | false; requires pel>1 when enabled | Field shift |
| `time` | 100; finite binary64, 0–100 | Displacement coefficient |
| `thscd1,thscd2` | 400, 51.0 | [Scene decision](sc-detection.md) |
| `tff` | Omitted | Parity source |
| `prefix` | `MVUtensils` | Super/Analysis names |

## 7. Boundaries, missing data, and errors

Invalid metadata, absent arrays, failed scene checks, or an out-of-range `n+d` cause whole-frame fallback. Complete malformed arrays, changed valid descriptors, or invalid current Super still fail. d=0 is valid and can displace current Super.

Creation checks the complete public vector domain after time scaling and all possible current-field shifts. Per frame, every actual reference vector with actual f must have a valid full footprint, even if SAD will ultimately select the current block. Failure is not fallback.

## 8. Precision and determinism

Time scaling truncates toward zero; chroma block total coordinates floor. For example, `vx=-3,time=50` gives dx=−1. Threshold, field shift, and window quantization remain separate; the process is not a linear image blend.

[Back to the English index](README.md)
