# Degrain / Degrain1–Degrain25

Combine the current image with motion-compensated references weighted by matching error. The named variants fix the number of reference pairs.

Block error here means the stored `AnalysisSAD`, which may contain SAD, SATD, or DCT luma error plus enabled chroma SAD. This function does not recompute pixel SAD or convert between metrics; its existing threshold and scaling formulas apply to the stored value. See [analysis data](../../knowledge/en/shared/analysis-data.md).

## Calling the function

VapourSynth: `core.neo_mv.Degrain`. AviSynth: `neo_mv_Degrain`. Parameter order:

```text
Degrain(clip, super, vectors [, thsad, thsad2, planes, limit, thscd1, thscd2, weights, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Visible source clip; fixed dimensions, format and nonzero frame count. |
| `super` | Clip | Required | Super clip carrying auxiliary images; use the same prefix as the producer. |
| `vectors` | Clip array | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `thsad` | Integer array | `[400, 400]` | Nonnegative near-pair thresholds [luma, chroma], scaled for samples and block geometry. |
| `thsad2` | Integer array | `thsad` | Nonnegative farthest-pair thresholds [luma, chroma]; defaults to effective thsad. |
| `planes` | Integer array | All planes | Plane indices 0, 1, 2; omitted/empty processes every actual plane. Duplicates are errors; absent valid indices are ignored. |
| `limit` | Float array | `[+inf, +inf]` | Maximum change from clip in native sample units [luma, chroma]. Finite values must be positive after float32 conversion; nonfinite values disable limiting. |
| `thscd1` | Integer | `400` | Per-block scene threshold, integer 0–16320 inclusive, scaled for block area, precision and chroma. |
| `thscd2` | Float | `51.0` | Bad-block percentage, finite 0–100 inclusive. A scene is rejected when the bad count strictly exceeds this percentage. |
| `weights` | Integer array | 2R+1 ones | User coefficients in far-first-side … near-first-side, center, near-second-side … far-second-side order. Exact length 2R+1; explicit empty is invalid. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

### Pair order, weights and named variants

Each adjacent pair must have opposite nonzero deltas with equal magnitude. Magnitudes increase strictly from pair to pair; the function does not sort inputs. `AnalyseMany` produces the usual `[+1,-1,+2,-2,…]` order. Either sign may appear first in an individual pair.

`thsad`, `thsad2` and `limit` accept a scalar/one element for both luma and chroma, or two elements [luma, chroma]; empty arrays use defaults. U and V share the chroma setting. Thresholds must remain representable after scaling. Limits use actual pixel units, not an 8-bit-normalized scale.

For R=2, `weights=[a,b,c,d,e]` assigns c to center, b/d to vector members 0/1, and a/e to members 2/3. Each coefficient, after int32 saturation, must be in `0..floor(2147483646 / (256*(2R+1)))`. Do not put the center coefficient first.

`Degrain` infers R from the vector count. `DegrainR` requires exactly 2R members and otherwise has the same parameters:

| Entry | Vector members |
| --- | --- |
| `Degrain1` / `neo_mv_Degrain1` | 2 |
| `Degrain2` / `neo_mv_Degrain2` | 4 |
| `Degrain3` / `neo_mv_Degrain3` | 6 |
| `Degrain4` / `neo_mv_Degrain4` | 8 |
| `Degrain5` / `neo_mv_Degrain5` | 10 |
| `Degrain6` / `neo_mv_Degrain6` | 12 |
| `Degrain7` / `neo_mv_Degrain7` | 14 |
| `Degrain8` / `neo_mv_Degrain8` | 16 |
| `Degrain9` / `neo_mv_Degrain9` | 18 |
| `Degrain10` / `neo_mv_Degrain10` | 20 |
| `Degrain11` / `neo_mv_Degrain11` | 22 |
| `Degrain12` / `neo_mv_Degrain12` | 24 |
| `Degrain13` / `neo_mv_Degrain13` | 26 |
| `Degrain14` / `neo_mv_Degrain14` | 28 |
| `Degrain15` / `neo_mv_Degrain15` | 30 |
| `Degrain16` / `neo_mv_Degrain16` | 32 |
| `Degrain17` / `neo_mv_Degrain17` | 34 |
| `Degrain18` / `neo_mv_Degrain18` | 36 |
| `Degrain19` / `neo_mv_Degrain19` | 38 |
| `Degrain20` / `neo_mv_Degrain20` | 40 |
| `Degrain21` / `neo_mv_Degrain21` | 42 |
| `Degrain22` / `neo_mv_Degrain22` | 44 |
| `Degrain23` / `neo_mv_Degrain23` | 46 |
| `Degrain24` / `neo_mv_Degrain24` | 48 |
| `Degrain25` / `neo_mv_Degrain25` | 50 |

## Input requirements

`clip` and Super must share visible dimensions, sample type/depth, color family, chroma ratios and frame count. Each vector carrier needs at least that many frames; its visible pixels are not used. Supported processing samples are integer 8–16-bit or float32 GRAY/YUV, subject to host format support.

Analysis real/working dimensions, padding and pel must match Super, and its block grid must cover the visible image without exceeding the working image. Processed chroma planes require aligned blocks and overlap. Analysis precision may differ from rendering precision; scene and error thresholds still use analysis precision. Preserve Super/Analysis properties and use a matching prefix. Auxiliary frame rates need not match clip.

## Return value and properties

Returns one clip with the same dimensions, format, frame count and frame rate as `clip`. Source frame properties are retained. AviSynth forwards audio and parity from the first input. Unselected planes copy clip. Unavailable/scene-rejected references receive zero effective weight; the current frame receives the remaining weight.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neo_mv.Super(clip, blksize=8, overlap=4, pad=32)
v = core.neo_mv.AnalyseMany(s, radius=2, badrange=0)
result = core.neo_mv.Degrain2(clip, s, v)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=2, badrange=0)
result = neo_mv_Degrain2(clip, s, v)
return result
```

## Restrictions and common errors

Requires 1–25 pairs (2–50 vector clips). Every member is validated even when its user weight is zero. Invalid descriptors, malformed complete arrays or insufficient Super sampling support fail. There are no fields, time, search or metric parameters.

## Computation

See [Degrain: computation](../../knowledge/en/degrain.md) for formulas, rounding and numerical examples.

[API index](README.md)
