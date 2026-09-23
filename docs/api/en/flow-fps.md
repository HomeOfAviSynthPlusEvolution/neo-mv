# FlowFPS

Resample the timeline at a target frame rate using motion interpolation.

## Calling the function

VapourSynth: `core.neomv.FlowFPS`. AviSynth: `neo_mv_FlowFPS`. Parameter order:

```text
FlowFPS(clip, super, vectors [, num, den, extramask, ml, blend, thscd1, thscd2, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `super` | Clip | Required | Super clip carrying auxiliary images; use the same prefix as the producer. |
| `vectors` | Clip array | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `num` | Integer | `25` | Nonnegative target frame-rate numerator. If either num or den is zero, double the source rate. |
| `den` | Integer | `1` | Nonnegative target frame-rate denominator. With both positive, output rate is num/den. |
| `extramask` | Boolean | `true` | Use additional observations from the same vector clips for extra occlusion handling; false removes those dependencies. |
| `ml` | Float | `100.0` | Finite positive motion/occlusion scale. |
| `blend` | Boolean | `true` | Blend original endpoint images when interpolation motion is unavailable; false copies the left image. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

## Input requirements

`clip` and Super must share visible dimensions, sample type/depth, color family, chroma ratios and frame count. Each vector carrier needs at least that many frames; its visible pixels are not used. Supported processing samples are integer 8–16-bit or float32 GRAY/YUV, subject to host format support.

Analysis real/working dimensions, padding and pel must match Super, and its block grid must cover the visible image without exceeding the working image. Processed chroma planes require aligned blocks and overlap. Analysis precision may differ from rendering precision; scene and error thresholds still use analysis precision. Preserve Super/Analysis properties and use a matching prefix. Auxiliary frame rates need not match clip. vectors is exactly `[bw, fw]`, with deltas `+d, -d` for the same positive d. Super has the original clip frame count, and both vector carriers have at least that many frames; descriptors must agree except direction and allowed level differences.

## Return value and properties

Preserves dimensions/format, changes rate to the reduced target fraction and frame count to floor(input_count × target_rate / source_rate). Properties follow the selected left/endpoint source frame. Endpoint positions copy the corresponding original image; unavailable interpolation blends or copies according to blend. AviSynth forwards input audio without retiming it.

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
result = core.neomv.FlowFPS(clip, s, v, num=30, den=1)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_FlowFPS(clip, s, v, num=30, den=1)
return result
```

## Restrictions and common errors

Requires a known positive source rate and nonzero representable output count. Negative num/den is invalid even when the other is zero. Auxiliary inputs retain the source frame count, not the output count. extramask=false eliminates extra observation requests.

## Computation

See [FlowFPS: computation](../../knowledge/en/flow-fps.md) for formulas, rounding and numerical examples.

[API index](README.md)
