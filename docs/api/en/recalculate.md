# Recalculate

Map existing vectors onto a target grid and optionally refine them using fresh sample errors.

## Calling the function

VapourSynth: `core.neo_mv.Recalculate`. AviSynth: `neo_mv_Recalculate`. Parameter order:

```text
Recalculate(super, vectors [, thsad, smooth, blksize, search, searchparam, mvlambda, chroma, pnew, overlap, meander, fields, tff, satd, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `super` | Clip | Required | Super clip carrying auxiliary images; use the same prefix as the producer. |
| `vectors` | Clip array | Required | Motion-analysis clip carrying analysis properties; see input requirements below. |
| `thsad` | Integer | `200` | Signed int32 threshold: search when freshly measured mapped-vector error exceeds the scaled threshold. Negative values are allowed. |
| `smooth` | Boolean | `true` | Interpolate the old vector grid; false selects a nearest old block. |
| `blksize` | Integer array | Super value | Target [width, height], inherited from Super when omitted/empty. Supported pairs and alignment follow Super. |
| `search` | Integer | `2` | Search mode 0–5; see the mode table below. |
| `searchparam` | Integer | `2` | Search parameter at coarse levels (the single level for Recalculate); values below 1 act as 1. Not a universal maximum displacement. |
| `mvlambda` | Integer | `1000` | Nonnegative distance penalty relative to the predictor; zero disables this penalty. |
| `chroma` | Boolean | `true` | Include chroma in matching when available; forced off for GRAY. |
| `pnew` | Integer | `25` | New-candidate error penalty, 0–256 inclusive. |
| `overlap` | Integer array | Super value | Target [horizontal, vertical] overlap; omitted/empty inherits Super. Each axis must be 0 through half its block and chroma aligned. |
| `meander` | Boolean | `true` | Alternate horizontal block traversal on successive rows. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |
| `tff` | Boolean | Omitted | Explicit first-frame top-field flag; parity alternates with frame index. Omitted: read required `_Field` properties. |
| `satd` | Boolean | `false` | Use SATD for luma; chroma remains SAD. Not supported for 6×6 and 16×2 blocks. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |

### Search modes

| Value | Candidate pattern |
| --- | --- |
| 0 | Decreasing steps with directional hints. |
| 1 | Concentric square rings around a fixed start. |
| 2 | Hexagon search and local refinement. |
| 3 | Cross, sparse pattern and hexagon refinement. |
| 4 | Horizontal-only search around a fixed start. |
| 5 | Vertical-only search around a fixed start. |

Search parameters control traversal; they do not universally impose a displacement radius. `fields=true` uses parity information; explicit `tff` describes the first field, not a constant parity for all frames.

## Input requirements

Use a Super with matching prefix and supported GRAY/YUV sample format. Axis arrays accept a scalar or one value for both axes, or two values [horizontal, vertical]; empty arrays inherit Super and more than two values fail. Block sizes and alignment follow [Super](super.md). Supply a nonempty vector clip array. Input sample precision must match Super; old block sizes, pel and grids may differ.

## Return value and properties

Returns a clip array with one result per input member, in input order, even for one member. Each result preserves Super’s visible image/timeline and writes recalculated Analysis properties with the member’s original delta. Out-of-range references are clamped to the nearest source frame.

AviSynth forwards audio and parity from the first input, Super.

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
result = core.neo_mv.Recalculate(s, v, blksize=8, overlap=4)
result[0].set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_Recalculate(s, v, blksize=8, overlap=4)
return result[0]
```

`result` is an array; the example outputs only member 0. Index another member or pass the full array to a compatible consumer.

## Restrictions and common errors

Empty vectors fail. Metadata must be valid, but missing/wrong-count old arrays use zero old vectors; complete corrupt arrays fail. fields=true requires old pel>1 and required parity. The target grid must have valid sampling support; later incompatible descriptor changes fail.

## Computation

See [Recalculate: computation](../../knowledge/en/recalculate.md) for formulas, rounding and numerical examples.

[API index](README.md)
