# DepanCompensate: compose global motion over time and sample

## 1. What the function computes

`DepanCompensate` selects a source frame from offset, converts and composes each intervening Depan motion into a sampling map, then interpolates that source image. It consumes neither block vectors nor Super.

## 2. Objects and notation

For output n and finite binary32 offset, I=ceil(offset) if positive, otherwise floor(offset). Source s=n−I. Aspect `a=pixaspect/(fields?2:1)`; center is half clip width/height.

## 3. Overall calculation

First decide direct current-frame bypass. Otherwise read motions in ascending order, compose a map, optionally match target field parity, then render clip[s] with per-plane maps/interpolation. Properties come from the selected source image.

## 4. Step-by-step calculation

I=0 or out-of-range s returns clip[n] without motion/parity reads. This is the bypass; a later identity map does not reenter it.

In the rendering branch, forward=(I>0) and every segment uses:

$$f=fl32(fl32(offset+(forward?1:-1))-fl32(I)).$$

Start from identity; read data[k] in ascending order from min(s, n)+1 through max(s, n). Decode using [shared properties](shared/global-motion.md). The first validly decoded good=false tuple resets the entire map to identity and stops; later tuples are not dependencies. Earlier malformed tuples still fail.

Convert each valid tuple to Tk using the same f, forward, aspect, center, then set T=Tk∘T. Every segment uses f, not just the last segment.

With fields=true and matchfields=true, add −0.5 to final ty for target top, +0.5 for bottom, even after invalid motion reset. Parity comes from clip[n] or tff; source parity is unnecessary. Disabling matchfields does not undo fields' halving of aspect.

Render clip[s] with [Depan sampling](shared/depan-sampling.md). Background is zero luma, half-range chroma. Properties come from clip[s]; composed motion does not overwrite Depan properties. info=true derives `DepanCompensate_info` from the final map before plane conversion, then invokes text rendering.

## 5. A complete numerical example

For `16×16`, n=5, offset=1.5: I=2, s=3, f=0.5; read data[4], data[5]. If both motions are dx=2, dy=rot=0, zoom=1, aspect=1 without fields, each becomes translation 1 and composition gives 2.

With subpixel=0, output (0, 0) reads clip[3] at (2, 0). It neither selects clip[4] nor produces total displacement 3. Properties also come from clip[3].

If the first tuple is invalid, reset to identity and skip the second, but still render clip[3], not clip[5].

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,data` | Required; data at least clip length | Images and motion tuples |
| `offset` | 0.0; finite binary32 −10…10 | Source frame and segment fraction |
| `subpixel` | 2; int32 saturation then 0–2 | Nearest, bilinear, bicubic |
| `pixaspect` | 1.0; finite positive | Geometry aspect |
| `matchfields` | true | Target parity matching with fields |
| `mirror` | 0; 0–15 | Top/bottom/left/right bits |
| `blur` | 0; nonnegative effective int32 | Horizontal reflected-border averaging |
| `info` | false | Diagnostic text |
| `fields,tff` | false, omitted | Half aspect and parity source |

## 7. Boundaries, missing data, and errors

Only 8–16-bit integer GRAY/YUV420/422/444 is supported. Mode 2 requires each plane height≥2; width 1 is allowed. Offset zero still validates arguments/format but reads no data properties. Creation does not require data[0] motion keys.

The info renderer still runs on bypass, without creating a new diagnostic key, so it may display an inherited one. Failure of required source or visited motion frames is an error, not bypass.

## 8. Precision and determinism

All segments keep six-coefficient composition without forcing equal diagonals. Both offset signs compose in ascending data-index order. Identity maps still use the selected sampler's edge rules, such as bilinear right-edge fill without right reflection; there is no generic copy shortcut.

[Back to the English index](README.md)
