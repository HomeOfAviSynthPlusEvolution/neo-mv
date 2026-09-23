# FlowInter

Interpolate between frame n and n+d without changing the timeline.

## Calling the function

VapourSynth: `core.neomv.FlowInter`. AviSynth: `neo_mv_FlowInter`. Parameter order:

```text
FlowInter(clip, super, vectors [, time, ml, blend, thscd1, thscd2, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `super` | Clip | Required | Super clip carrying auxiliary images; use the same prefix as the producer. |
| `vectors` | Clip array | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `time` | Float | `50.0` | Finite 0–100 percent between frame n and n+d. |
| `ml` | Float | `100.0` | Finite positive motion/occlusion scale. |
| `blend` | Boolean | `true` | Blend original endpoint images when interpolation motion is unavailable; false copies the left image. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

## Input requirements

`clip` and Super must share visible dimensions, sample type/depth, color family, chroma ratios and frame count. Each vector carrier needs at least that many frames; its visible pixels are not used. Supported processing samples are integer 8–16-bit or float32 GRAY/YUV, subject to host format support.

Analysis real/working dimensions, padding and pel must match Super, and its block grid must cover the visible image without exceeding the working image. Processed chroma planes require aligned blocks and overlap. Analysis precision may differ from rendering precision; scene and error thresholds still use analysis precision. Preserve Super/Analysis properties and use a matching prefix. Auxiliary frame rates need not match clip. vectors is exactly `[bw, fw]`, with deltas `+d, -d` for the same positive d. Super has the original clip frame count, and both vector carriers have at least that many frames; descriptors must agree except direction and allowed level differences.

## Return value and properties

Returns one clip with the same dimensions, format, frame count and frame rate as `clip`. Source frame properties are retained. AviSynth forwards audio and parity from the first input. If motion is unavailable, blend the clamped original endpoints when blend=true, otherwise copy the left image.

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
result = core.neomv.FlowInter(clip, s, v)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_FlowInter(clip, s, v)
return result
```

## Restrictions and common errors

time must be 0–100 and ml positive. Both main fields are validated even if one is unavailable. Missing extra observations select ordinary interpolation; corrupt observations or invalid sampling are errors, not a blend fallback.

## Computation

See [FlowInter: computation](../../knowledge/en/flow-inter.md) for formulas, rounding and numerical examples.

[API index](README.md)
