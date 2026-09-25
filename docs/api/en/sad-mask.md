# SADMask

Visualize stored block errors after vector-based projection.

Block error here means the stored `AnalysisSAD`, which may contain SAD, SATD, or DCT luma error plus enabled chroma SAD. This function does not recompute pixel SAD or convert between metrics; its existing threshold and scaling formulas apply to the stored value. See [analysis data](../../knowledge/en/shared/analysis-data.md).

## Calling the function

VapourSynth: `core.neo_mv.SADMask`. AviSynth: `neo_mv_SADMask`. Parameter order:

```text
SADMask(vectors [, ml, gamma, time, scval, thscd1, thscd2, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `vectors` | Clip | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `ml` | Float | `100.0` | Finite positive normalization scale; motion length, stored error, or occlusion strength depending on the function. |
| `gamma` | Float | `1.0` | Finite nonnegative exponent applied to the normalized block score. |
| `time` | Float | `100.0` | Finite 0–100. SADMask uses projection time; OcclusionMask uses interval length; VectorLengthMask validates but does not use it. |
| `scval` | Float | `0.0` | Finite fill value for unavailable/rejected fields. Integer output rounds by trunc(scval+0.5) and must be in sample range; float output uses it directly. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

## Input requirements

Only vectors properties are read; carrier pixels and reference images are not inputs to the calculation. Creation requires valid analysis metadata; preserve the selected prefix.

## Return value and properties

Returns a GRAY clip sized by AnalysisRealWidth/Height, with precision from AnalysisBitsPerSample and frame count/rate from vectors. Properties contain only `_Range=1`; input properties are not copied. Unavailable fields fill scval. AviSynth forwards the first input’s audio.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neo_mv.Super(clip, blksize=8, overlap=4, pad=32)
v = core.neo_mv.AnalyseMany(s, radius=1, badrange=0)
result = core.neo_mv.SADMask(v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_SADMask(v[0])
return result
```

## Restrictions and common errors

Invalid parameters, complete corrupt arrays or changed valid descriptors fail. There is no check that n+delta exists: complete external fields can generate masks at boundaries. VectorLengthMask gamma=0 also lights zero vectors; OcclusionMask gamma=0 only affects actual events.

## Computation

See [SADMask: computation](../../knowledge/en/sad-mask.md) for formulas, rounding and numerical examples.

[API index](README.md)
