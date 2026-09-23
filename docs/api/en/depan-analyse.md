# DepanAnalyse

Fit adjacent-frame global motion from existing block vectors.

## Calling the function

VapourSynth: `core.neomv.DepanAnalyse`. AviSynth: `neo_mv_DepanAnalyse`. Parameter order:

```text
DepanAnalyse(clip, vectors [, mask, zoom, rot, pixaspect, error, info, wrong, zerow, thscd1, thscd2, fields, tff])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `vectors` | Clip | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `mask` | Clip | Omitted | Optional integer8 mask; same dimensions as clip and at least its frame count. The first plane supplies fitting weights. |
| `zoom` | Boolean | `true` | Allow zoom in the global-motion fit. |
| `rot` | Boolean | `true` | Allow rotation in the global-motion fit. |
| `pixaspect` | Float | `1.0` | Finite positive pixel aspect ratio; field mode adjusts the effective vertical aspect. |
| `error` | Float | `15.0` | Finite fitting-error control used for initialization and acceptance. |
| `info` | Boolean | `false` | Write diagnostic text and draw it using the host renderer. AviSynth uses propShow. |
| `wrong` | Float | `10.0` | Finite threshold for rejecting motion inconsistent with neighboring observations. |
| `zerow` | Float | `0.05` | Finite weight multiplier for zero-displacement observations; not restricted to 0–1. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |
| `tff` | Boolean | Omitted | Explicit first-frame top-field flag; parity alternates with frame index. Omitted: read required `_Field` properties. |

## Input requirements

clip is a fixed video carrier and may be RGB/float because its pixels are not fitted. vectors must have valid default-prefix (`MVUtensils`) metadata, delta ±1 and at least clip’s frame count. With +1, output n reads vectors[max(0,n−1)]; with −1 it reads vectors[n]. An optional mask must match clip size and provide enough integer8 frames.

## Return value and properties

Writes `Depan_dx`, `Depan_dy` (pixel displacement), `Depan_rot` (degrees), `Depan_zoom` (scale) and `Depan_goodmotion` (0/1). These names have no configurable prefix. Goodmotion=0 denotes an unusable estimate, not zero motion. The visible clip/timeline is retained except when display options draw on it.

With info=true, the diagnostic property is `DepanAnalyse_info`; text drawing may modify visible pixels.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neomv.Super(clip, blksize=8, overlap=4, pad=32)
v = core.neomv.AnalyseMany(s, radius=1, badrange=0)
result = core.neomv.DepanAnalyse(clip, v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_DepanAnalyse(clip, v[0])
return result
```

## Restrictions and common errors

No prefix parameter is provided. Nonfinite controls, invalid aspect, unavailable required parity, corrupt vectors, bad mask geometry or numerical failure are errors. An unavailable field can yield invalid motion, but does not hide missing required mask frames.

## Computation

See [DepanAnalyse: computation](../../knowledge/en/depan-analyse.md) for formulas, rounding and numerical examples.

[API index](README.md)
