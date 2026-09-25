# SCDetection

Attach scene-change flags using stored block errors.

Block error here means the stored `AnalysisSAD`, which may contain SAD, SATD, DCT, or mixed luma error plus enabled chroma SAD. This function does not recompute pixel SAD or convert between metrics; its existing threshold and scaling formulas apply to the stored value. See [analysis data](../../knowledge/en/shared/analysis-data.md).

## Calling the function

VapourSynth: `core.neo_mv.SCDetection`. AviSynth: `neo_mv_SCDetection`. Parameter order:

```text
SCDetection(clip, vectors [, thscd1, thscd2, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `vectors` | Clip | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

## Input requirements

clip supplies visible output; vectors supplies analysis properties. Creation requires valid analysis metadata, but not populated arrays on frame zero. The vectors input must provide the requested frame indices.

## Return value and properties

Returns one clip with the same dimensions, format, frame count and frame rate as `clip`. Source frame properties are retained. AviSynth forwards audio and parity from the first input. Sets `_SceneChangePrev` for negative delta, `_SceneChangeNext` otherwise, to 0 or 1; the opposite key is preserved. Missing/unavailable data yields 1.

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
result = core.neo_mv.SCDetection(clip, v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_SCDetection(clip, v[0])
return result
```

## Restrictions and common errors

thscd1 and thscd2 must satisfy their ranges. Complete corrupt vectors/errors or changed valid geometry fail. No reference-frame existence test is performed; thscd2=100 does not suppress the missing-data flag.

## Computation

See [SCDetection: computation](../../knowledge/en/sc-detection.md) for formulas, rounding and numerical examples.

[API index](README.md)
