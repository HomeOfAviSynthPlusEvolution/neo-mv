# DepanStabilise: from cumulative motion to a smoothed sampling path

## 1. What the function computes

`DepanStabilise` reads adjacent-frame global motion, accumulates a path, then derives a correction between the actual and smoothed paths. It resamples images through that correction, optionally filling positions unavailable in the current layer from previous/next image layers.

Method 0 uses an inertial predictor/corrector recurrence; method 1 uses symmetric weighted temporal averaging. Their boundary, zoom, and limiting behavior differ.

Dimensions, format, count, and rate remain unchanged. Data supplies [Depan motion properties](shared/global-motion.md), not pixels. No new correction-motion properties are exported.

## 2. Objects and notation

Current n is in F frames. mj describes frame j relative to its predecessor. The first frame of [b, h] is the cumulative base; Cj is the actual path, Sj the smoothed path, Q the final output-to-input sampling map:

$$X=t_x+ux+vy,\qquad Y=t_y+wx+hy.$$

Correction dx=+2 reads two pixels right in the source. Initial sampling scale is the reciprocal z0=1/initzoom.

## 3. Overall calculation

1. Derive coefficients and temporal weights from rate and parameters.
2. Scan motion validity to select this frame's interval.
3. Compose adjacent motions into cumulative maps.
4. Smooth through the inertial/window method; inertial mode also tapers and limits corrections.
5. Select previous/next images and maps from the correction.
6. Render previous, next, current layers; inherit current properties and optionally draw diagnostics.

## 4. Step-by-step calculation

### 4.1 Parameters and valid intervals

Full coefficient formulas are in [temporal smoothing and limits](shared/stabilisation-smoothing.md). Effective aspect is pixaspect/(fields?2:1). Here fields changes geometry only: no `_Field` reads and no tff argument.

Motion index 0 always uses synthetic invalid `(0,0,0,1,0)` without reading data[0] properties. Every other visited index fully decodes all five keys, even if goodmotion is false.

Method 0 first computes t=(10fps)/cutoff. Unless t<5fps, replace it by 5fps. This initial t may overflow to positive infinity, but the cap must be finite. Set:

$$b_0=\max(0,\operatorname{trunc}(\operatorname{fl32}(n)-t)),\quad h=n.$$

Scan backward n…b0, stopping at the first invalid motion and setting b to its index; otherwise b=b0. At 25fps, cutoff=1, n=140, b0=15. If nearest invalid motion is 120, accumulate from 120 and do not inspect 119.

When b=n, directly use Q=Z(z0), skipping smoothing, taper, limits. Layer selection, sampling, and diagnostics still run.

Method 1 begins b0=max(0, n−r), h0=min(F−1, n+r). Scan backward to obtain b1. Independently scan forward n+1…h0; the first invalid j gives h1=max(j−1, n), otherwise h1=h0. Forward scanning still runs if current motion is invalid. Symmetrize:

$$k=\min(n-b_1,h_1-n),\qquad b=n-k,\quad h=n+k.$$

Already visited data must be validly decoded even if subsequently cropped away. An interval reduced to [n, n] still runs window smoothing and the motion/map round trip.

### 4.2 From adjacent motion to correction

Let M convert motion to a sampling map. Start Cb=I, then Cj=M(mj)∘C(j−1). The base tuple is not composed, allowing an interval to begin at invalid motion.

Method 0 initializes its first two unzoomed path entries to identity. Later entries use one predictor and one corrector for translation/rotation and a recursive diagonal average. Optional adaptive zoom runs independently after the unzoomed sequence completes.

Method 1 averages translation/rotation using cosine weights; diagonal averaging uses only the immediate three-frame neighborhood. Optional zoom has separate weights. Both form:

$$Q=S\circ Inv(C_n).$$

Inv is the specific similarity-model inverse, not an arbitrary affine inverse. [Detailed smoothing](shared/stabilisation-smoothing.md) expands that inverse, both recurrences, and adaptive zoom.

Method 0 converts Q to motion, tapers near the end, applies ordered dx, dy, zoom, rotation limits, and converts back. A limit may reduce a component or reset the whole correction and set b=n. Method 1 skips limits but still performs map→motion→map.

### 4.3 Layers and sampling

Previous/next selection independently starts from Q and scores candidate maps by translation plus temporal distance. Only strict improvement changes the image; it is paired with the final accumulated map. Next selection fully decodes its whole range before validity stops the selection loop.

Pixels are overwritten in previous, next, current order. Neighbors use nearest; current uses subpixel. First-layer borders may reflect or write background; later background branches preserve earlier pixels. See [layer selection/composition](shared/stabilisation-layers.md) and [sampling formulas](shared/depan-sampling.md).

Even identity correction does not directly return the original image: sampler boundaries and earlier layers can affect edge pixels.

### 4.4 Properties and diagnostics

All properties inherit from clip[n], including its existing five Depan keys. Correction motion does not replace them.

With info=true, convert final full-resolution Q before plane conversion/clamping back to motion and write DepanStabilise_info, for example:

```text
frame=4 base =0 dx=2.00 dy=0.00 rot=-0.000 zoom=1.00000
```

For b=n, use prefix `frame=4 BASE!=4`. Displacements have two decimal places, rotation three, zoom five, nearest-even with signed zero, capped at 127 bytes. The host renders only this property. Neighbor maps do not enter diagnostics; info=false preserves an inherited same-named value.

## 5. A complete numerical example

Use `48×48`, 24fps, cutoff=3, method=1, initzoom=1, addzoom=false, aspect=1, prev=next=0, subpixel=0. Request frame 1 in a sequence of at least three frames. Motion 1 is dx=+1 and motion 2 dx=0, both valid.

r=24/(4×3)=2. Backward scanning reaches synthetic invalid index 0; symmetrization gives [0, 2]. Cumulative horizontal translations are `[0,1,1]`, with unit diagonals and zero other components.

Let c=w(1)≈0.70710677, giving weights `[c,1,c]`:

$$t_{x,S}=(0c+1+1c)/((c+1)+c)\approx0.70710677.$$

Current cumulative translation is 1, so raw correction is approximately −0.29289323. After the required motion round trip, nearest horizontal coordinate is x+floor(tx+0.5)=x, with unchanged Y. Pixels therefore sample the same positions of the current image in this example.

Bilinear sampling would use that fractional correction. Unchanged nearest-neighbor pixels do not mean the correction calculation produced no displacement.

## 6. Parameters and their calculation steps

| Parameter | Default | Role |
| --- | --- | --- |
| `clip,data` | Required | Images and adjacent global motion |
| `cutoff` | 1 | Positive; radius, lookback, inertial response |
| `damping` | 0.9 | Inertial damping; finite negatives allowed |
| `initzoom` | 1 | Positive; reciprocal initial sampling scale |
| `addzoom` | false | Method-specific adaptive zoom |
| `prev,next` | 0, 0 | Nonnegative fill search extents |
| `mirror,blur` | 0, 0 | First-layer reflection bits 0–15; nonnegative horizontal averaging extent |
| `dxmax,dymax` | 60, 30 | Inertial nonlinear coefficients and limits |
| `zoommax,rotmax` | 1.05, 1 | Inertial limits; negatives can reset the whole correction |
| `subpixel` | 2 | Current nearest 0, bilinear 1, bicubic 2 |
| `pixaspect,fields` | 1, false | Positive aspect; fields halves it |
| `fitlast` | 0 | Positive end taper toward initial correction |
| `tzoom` | 3 | Nonnegative zoom response/window parameter |
| `method` | 0 | Inertial 0, symmetric window 1 |
| `info` | false | Diagnostic text |

Method 1 still validates all public types/domains and shared normalization, but does not use inertial coefficients, limits, or fitlast.

## 7. Boundaries, missing data, and errors

Supports constant 8–16-bit integer GRAY/YUV420/422/444 with known positive rate. Bicubic needs every plane height≥2; width 1 is allowed. Data has at least F frames but need not match dimensions, format, or rate.

Interval scanning and layer selection define which properties are visited. Unvisited malformed data cannot fail this output, and visited malformed data cannot be skipped. Invalid motion marks an interval boundary; malformed properties are errors.

tzoom=0 can pass creation but makes window adaptive normalization zero. Inertial mode fails only when the zoom reciprocal is needed. Float32 conversion of large frame numbers can put lookback bounds after n; this fails rather than silently repairing the bound.

Inertial overflow can recover within [its defined boundary](shared/stabilisation-smoothing.md). Other illegal arithmetic, missing frames, allocation failures, and text-rendering errors do not become unmodified output.

## 8. Precision and determinism

Paths and recurrences use separately rounded binary32 operations. Composed diagonals may differ slightly; inverse, window averaging, and final conversion each handle them explicitly rather than forcing symmetry everywhere.

Every output is defined from its own input interval. Request order, repeated requests, and other instances do not provide hidden recurrence state; recovery does not seed later outputs. Computation is repeatable in a fixed environment; text pixels also depend on the host renderer.

[Back to the English index](README.md)
