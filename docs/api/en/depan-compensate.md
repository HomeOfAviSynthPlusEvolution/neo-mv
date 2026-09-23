# DepanCompensate

Apply global-motion compensation at an integer or fractional frame offset.

## Calling the function

VapourSynth: `core.neo_mv.DepanCompensate`. AviSynth: `neo_mv_DepanCompensate`. Parameter order:

```text
DepanCompensate(clip, data [, offset, subpixel, pixaspect, matchfields, mirror, blur, info, fields, tff])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `data` | Clip | Required | Clip carrying Depan motion properties, with at least as many frames as clip. Dimensions/format/rate need not match. |
| `offset` | Float | `0.0` | Finite float32 −10 to 10. Positive selects earlier source frames; negative selects later ones. Zero bypasses compensation. |
| `subpixel` | Integer | `2` | Resampling: 0 nearest, 1 bilinear, 2 bicubic. |
| `pixaspect` | Float | `1.0` | Finite positive pixel aspect ratio; field mode adjusts the effective vertical aspect. |
| `matchfields` | Boolean | `true` | Match target field parity when fields=true. |
| `mirror` | Integer | `0` | Reflection bit mask 0–15: top=1, bottom=2, left=4, right=8; add bits to combine borders. |
| `blur` | Integer | `0` | Nonnegative horizontal averaging extent for reflected borders; not temporal motion blur. |
| `info` | Boolean | `false` | Write diagnostic text and draw it using the host renderer. AviSynth uses propShow. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |
| `tff` | Boolean | Omitted | Explicit first-frame top-field flag; parity alternates with frame index. Omitted: read required `_Field` properties. |

## Input requirements

clip accepts fixed integer 8–16-bit GRAY/YUV420/422/444. data carries the five Depan motion properties, normally from DepanEstimate or DepanAnalyse; it needs at least clip’s frame count. Float processing is not supported. Bicubic sampling needs at least two rows in every plane.

## Return value and properties

Returns one clip with the same dimensions, format, frame count and frame rate as `clip`. Source frame properties are retained. AviSynth forwards audio and parity from the first input.

With info=true, the diagnostic property is `DepanCompensate_info`; text drawing may modify visible pixels.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
data = core.neo_mv.DepanEstimate(clip, winx=32, winy=32)
result = core.neo_mv.DepanCompensate(clip, data, offset=1)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
data = neo_mv_DepanEstimate(clip, winx=32, winy=32)
result = neo_mv_DepanCompensate(clip, data, offset=1)
return result
```

## Restrictions and common errors

offset=0 and out-of-range source selection bypass motion sampling. Invalid motion causes bypass; missing/malformed properties on visited frames and dependency failures are errors. offset=0 still validates arguments and format. fields plus matchfields require target parity; there is no prefix parameter.

## Computation

See [DepanCompensate: computation](../../knowledge/en/depan-compensate.md) for formulas, rounding and numerical examples.

[API index](README.md)
