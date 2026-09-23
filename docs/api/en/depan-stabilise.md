# DepanStabilise

Smooth global motion and render the corrected image, optionally filling borders from neighboring frames.

## Calling the function

VapourSynth: `core.neomv.DepanStabilise`. AviSynth: `neo_mv_DepanStabilise`. Parameter order:

```text
DepanStabilise(clip, data [, cutoff, damping, initzoom, addzoom, prev, next, mirror, blur, dxmax, dymax, zoommax, rotmax, subpixel, pixaspect, fitlast, tzoom, info, method, fields])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `data` | Clip | Required | Clip carrying Depan motion properties, with at least as many frames as clip. Dimensions/format/rate need not match. |
| `cutoff` | Float | `1.0` | Finite positive smoothing cutoff in Hz; controls temporal support and response. |
| `damping` | Float | `0.9` | Finite inertial damping for method=0; negative values are accepted. |
| `initzoom` | Float | `1.0` | Finite positive initial zoom; sampling uses its reciprocal. |
| `addzoom` | Boolean | `false` | Enable adaptive zoom for the selected smoothing method. |
| `prev` | Integer | `0` | Nonnegative number of earlier frames eligible for filling uncovered areas. |
| `next` | Integer | `0` | Nonnegative number of later frames eligible for filling uncovered areas. |
| `mirror` | Integer | `0` | Reflection bit mask 0–15: top=1, bottom=2, left=4, right=8; add bits to combine borders. |
| `blur` | Integer | `0` | Nonnegative horizontal averaging extent for reflected borders; not temporal motion blur. |
| `dxmax` | Float | `60.0` | Finite horizontal inertial correction control/limit, in pixels. Negative limits can reset the whole correction. |
| `dymax` | Float | `30.0` | Finite vertical inertial correction control/limit, in pixels. Negative limits can reset the whole correction. |
| `zoommax` | Float | `1.05` | Finite inertial zoom limit; negative values can reset the whole correction. |
| `rotmax` | Float | `1.0` | Finite inertial rotation limit in degrees; negative values can reset the whole correction. |
| `subpixel` | Integer | `2` | Resampling: 0 nearest, 1 bilinear, 2 bicubic. |
| `pixaspect` | Float | `1.0` | Finite positive pixel aspect ratio; field mode adjusts the effective vertical aspect. |
| `fitlast` | Integer | `0` | A positive value tapers the last frames toward the initial correction; nonpositive disables this taper. Used by method=0. |
| `tzoom` | Float | `3.0` | Finite nonnegative adaptive-zoom time parameter. Zero can cause invalid normalization when adaptive zoom is used. |
| `info` | Boolean | `false` | Write diagnostic text and draw it using the host renderer. AviSynth uses propShow. |
| `method` | Integer | `0` | Smoothing: 0 inertial, 1 symmetric window. Method 1 does not use inertial limits, damping, or fitlast. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |

## Input requirements

clip accepts fixed integer 8–16-bit GRAY/YUV420/422/444. data carries the five Depan motion properties, normally from DepanEstimate or DepanAnalyse; it needs at least clip’s frame count. Float processing is not supported. Bicubic sampling needs at least two rows in every plane. clip must also have a known positive frame rate.

## Return value and properties

Returns one clip with the same dimensions, format, frame count and frame rate as `clip`. Source frame properties are retained. AviSynth forwards audio and parity from the first input.

With info=true, the diagnostic property is `DepanStabilise_info`; text drawing may modify visible pixels.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
data = core.neomv.DepanEstimate(clip, winx=32, winy=32)
result = core.neomv.DepanStabilise(clip, data)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
data = neo_mv_DepanEstimate(clip, winx=32, winy=32)
result = neo_mv_DepanStabilise(clip, data)
return result
```

## Restrictions and common errors

All floating controls must remain finite after float32 conversion. cutoff/initzoom/pixaspect must be positive; tzoom, prev, next and blur nonnegative. method is 0 or 1. Visited malformed motion properties are errors; goodmotion=0 marks an interval boundary. There are no tff or prefix parameters.

## Computation

See [DepanStabilise: computation](../../knowledge/en/depan-stabilise.md) for formulas, rounding and numerical examples.

[API index](README.md)
