# Compensate

Render block motion compensation from one vector clip.

## Calling the function

VapourSynth: `core.neomv.Compensate`. AviSynth: `neo_mv_Compensate`. Parameter order:

```text
Compensate(clip, super, vectors [, thsad, fields, time, thscd1, thscd2, tff, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `super` | Clip | Required | Super clip carrying auxiliary images; use the same prefix as the producer. |
| `vectors` | Clip | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `thsad` | Integer | `10000` | Nonnegative int64 block threshold. A reference block is used only when its error is strictly below the scaled threshold. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |
| `time` | Float | `100.0` | Finite 0–100 percent of reference displacement; not an image blending percentage. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `tff` | Boolean | Omitted | Explicit first-frame top-field flag; parity alternates with frame index. Omitted: read required `_Field` properties. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

## Input requirements

`clip` and Super must share visible dimensions, sample type/depth, color family, chroma ratios and frame count. Each vector carrier needs at least that many frames; its visible pixels are not used. Supported processing samples are integer 8–16-bit or float32 GRAY/YUV, subject to host format support.

Analysis real/working dimensions, padding and pel must match Super, and its block grid must cover the visible image without exceeding the working image. Processed chroma planes require aligned blocks and overlap. Analysis precision may differ from rendering precision; scene and error thresholds still use analysis precision. Preserve Super/Analysis properties and use a matching prefix. Auxiliary frame rates need not match clip.

## Return value and properties

Returns one clip with the same dimensions, format, frame count and frame rate as `clip`. Source frame properties are retained. AviSynth forwards audio and parity from the first input. Unavailable/rejected references copy clip for the whole frame. Available blocks select current or reference samples by thsad, then compose overlaps.

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
result = core.neomv.Compensate(clip, s, v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_Compensate(clip, s, v[0])
return result
```

## Restrictions and common errors

fields=true requires pel>1 and required parity. All possible vectors/time/field shifts must fit valid sampling support. Actual illegal samples or complete corrupt arrays are errors, not fallback. A zero delta is accepted.

## Computation

See [Compensate: computation](../../knowledge/en/compensate.md) for formulas, rounding and numerical examples.

[API index](README.md)
