# Analyse

Estimate block motion from each frame to a reference at n+delta.

## Calling the function

VapourSynth: `core.neomv.Analyse`. AviSynth: `neo_mv_Analyse`. Parameter order:

```text
Analyse(super [, blksize, levels, search, searchparam, pelsearch, mvlambda, chroma, delta, lsad, plevel, globalmv, pnew, pzero, pglobal, overlap, badsad, badrange, meander, trymany, fields, tff, satd, prefix])
```

Brackets here mark optional arguments, not a literal array. Use named optional arguments. Booleans are `True`/`False` in Python and `true`/`false` in AviSynth.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `super` | Clip | Required | Super clip carrying auxiliary images; use the same prefix as the producer. |
| `blksize` | Integer array | Super value | Target [width, height], inherited from Super when omitted/empty. Supported pairs and alignment follow Super. |
| `levels` | Integer | `0` | 0 selects the automatic count; positive requests a count, negative subtracts from the automatic count. Capped by available Super levels; at least one must remain. |
| `search` | Integer | `2` | Search mode 0–5; see the mode table below. |
| `searchparam` | Integer | `2` | Search parameter at coarse levels (the single level for Recalculate); values below 1 act as 1. Not a universal maximum displacement. |
| `pelsearch` | Integer | `SuperPel` | Positive finest-level search parameter in subpixel vector units; default is Super's pel. |
| `mvlambda` | Integer | `1000` | Nonnegative distance penalty relative to the predictor; zero disables this penalty. |
| `chroma` | Boolean | `true` | Include chroma in matching when available; forced off for GRAY. |
| `delta` | Integer | `1` | Signed nonzero reference offset: output frame n analyzes source frame n+delta. Positive means a later reference. |
| `lsad` | Integer | `400` | Predictor-error threshold controlling distance-penalty adaptation; negative values are accepted. |
| `plevel` | Integer | `1` | Pyramid-level penalty scaling: 0, 1 or 2. |
| `globalmv` | Boolean | `true` | Enable the global-motion predictor. |
| `pnew` | Integer | `25` | New-candidate error penalty, 0–256 inclusive. |
| `pzero` | Integer | `pnew` | Zero-vector seed penalty, 0–256; defaults to effective pnew. |
| `pglobal` | Integer | `0` | Global-seed penalty, 0–256; effectively pzero when globalmv=false. |
| `overlap` | Integer array | Super value | Target [horizontal, vertical] overlap; omitted/empty inherits Super. Each axis must be 0 through half its block and chroma aligned. |
| `badsad` | Integer | `10000` | Raw-error threshold for bad-block search extension; negatives are accepted. |
| `badrange` | Integer | `24` | Positive: multiscale expansion; negative: expanding rings; zero: no expansion of those types. Subpixel local refinement may still run. |
| `meander` | Boolean | `true` | Alternate horizontal block traversal on successive rows. |
| `trymany` | Integer | `0` | 0 searches from the selected seed; 1 tries multiple seeds on coarse levels; 2 does so on all levels. |
| `fields` | Boolean | `false` | Enable field-aware calculations. This does not separate interlaced frames into fields. |
| `tff` | Boolean | Omitted | Explicit first-frame top-field flag; parity alternates with frame index. Omitted: read required `_Field` properties. |
| `satd` | Boolean | `false` | Use SATD for luma; chroma remains SAD. Not supported for 16×2 blocks. |
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

Use a Super with matching prefix and supported GRAY/YUV sample format. Axis arrays accept a scalar or one value for both axes, or two values [horizontal, vertical]; empty arrays inherit Super and more than two values fail. Block sizes and alignment follow [Super](super.md).

## Return value and properties

Preserves the visible Super image, dimensions, format, frame count, rate and unrelated properties. Adds selected-prefix `Analysis*` metadata plus `AnalysisVectors` and `AnalysisSAD` when a reference is available. For frame n with n+delta outside the clip, emits metadata but removes those two arrays. Carrier pixels are not a visualization of vectors.

AviSynth forwards audio and parity from the first input, Super.

## Minimal examples

Replace the plugin path. These blank-source scripts need no external video reader; the selected output can be evaluated directly.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neomv.Super(clip, blksize=8, overlap=4, pad=32)
result = core.neomv.Analyse(s, delta=1, badrange=0)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
result = neo_mv_Analyse(s, delta=1, badrange=0)
return result
```

## Restrictions and common errors

Missing Super data, invalid block geometry/enums, delta=0, nonpositive pelsearch, or no usable levels fail at creation. The complete search sampling domain must fit valid Super support. Field mode requires pel>1 and required parity; SATD rejects 16×2. Changed frame metadata or nonfinite calculations fail at evaluation.

## Computation

See [Analyse: computation](../../knowledge/en/analyse.md) for formulas, rounding and numerical examples.

[API index](README.md)
