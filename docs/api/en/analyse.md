# Analyse

Estimate block motion from each frame to a reference at n+delta.

## Calling the function

VapourSynth: `core.neo_mv.Analyse`. AviSynth: `neo_mv_Analyse`. Parameter order:

```text
Analyse(super [, blksize, levels, search, searchparam, pelsearch, mvlambda, chroma, delta, lsad, plevel, globalmv, pnew, pzero, pglobal, overlap, badsad, badrange, meander, trymany, fields, tff, metric, prefix, metric_weight, metric_threshold])
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
| `metric` | String | `"sad"` | Luma matching metric: pure SAD/SATD/DCT or local/global mixtures; chroma remains SAD. See restrictions below. |
| `prefix` | String | `"MVUtensils"` | Property-name prefix. Match all producers and consumers. Empty is allowed; NUL is not. |
| `metric_weight` | Float | `0.5` | Transform contribution after a local trigger; [0,1], precision 1/65536. |
| `metric_threshold` | Float | `0.03125` | Local relative luma-sum change threshold; [0,1], precision 1/65536. |

### Matching metric

`metric` accepts case-sensitive strings and defaults to `"sad"`, replacing the former `satd` Boolean parameter.

| Value | Luma error | Formats and block sizes |
| --- | --- | --- |
| `"sad"` | Sum of absolute pixel differences. | 8–16-bit integer and float32; all valid block sizes. |
| `"satd"` | Absolute differences based on 4×4 Hadamard transforms. | 8–16-bit integer and float32; both dimensions must be divisible by 4, excluding 6×6 and 16×2. |
| `"dct"` | DCT-II and coefficient quantization of each source/reference block, followed by coefficient absolute differences with DC weighting and block-size scaling. | 8–16-bit integer only; all valid block sizes, including 6×6 and 16×2. |

With `chroma=true`, chroma uses SAD in all modes; `metric` changes only the luma metric. DCT uses float32 arithmetic for all supported sizes and integer bit depths, including 8-bit 8×8. AC coefficients round the computed float32 value to the nearest even integer; DC is truncated from the exact integer sample sum. Small numerical differences from double or historical integer DCT implementations are accepted and can change a search decision between nearly tied candidates. This is not the direct distance between unquantized DCT coefficients.

The output property remains named `AnalysisSAD`, but contains block error from the selected metric (including chroma SAD when enabled), not necessarily pixel SAD. Changing metrics changes the error distribution; error thresholds are not automatically converted to equivalent SAD thresholds.


#### Mixed modes

These five new modes accept only 8–16-bit integer samples. Every SATD-family mode requires both block dimensions divisible by 4, even at zero weight; the DCT family supports all valid block shapes. The base distances follow the definitions above.

| `metric` | Luma error policy |
| --- | --- |
| `sad_dct_global` | Mix SAD and DCT using a frame-pair weight; DC weight is 4. |
| `sad_dct_local` | Check brightness change per candidate, then mix with `metric_weight` if triggered; DC weight is 1. |
| `sad_satd_global` | Mix SAD and SATD using a frame-pair weight. |
| `sad_satd_local` | Check brightness change per candidate, then mix with `metric_weight` if triggered. |
| `sad_satd_global_half` | Halve the global SATD weight, limiting it to 50%. |

Only the two local modes accept explicit `metric_weight` and `metric_threshold`. Both must be finite numbers in [0,1], defaulting to 0.5 and 0.03125. At creation, each is rounded to the nearest 1/65536 grid point, with exact ties rounded up. Supplying either parameter to another mode is an error, even at its default value. Both parameters are appended to the original signature.

Let Ls and Lr be source and candidate luma sums. Trigger only when `abs(Ls-Lr)*65536 > (Ls+Lr)*H`, where H is the quantized threshold. The triggered luma error is `(S*(65536-W)+X*W)/65536`, where W is the quantized weight and X is the transform error; truncate once at the end. Otherwise use SAD. Two zero sums or equality at the threshold do not trigger. weight=0 or threshold=1 always uses SAD; weight=1 uses pure transform error only after triggering.

Global modes average signed zero-displacement brightness differences across the actual coarsest block grid, then normalize this to an integer weight from 0 to 16. Opposite changes can cancel; overlaps are counted repeatedly. The coarsest level itself uses SAD; finer levels reuse the same weight. With just one level, global modes are SAD. Each AnalyseMany member computes its own statistics. Chroma SAD is added exactly once, and thresholds are not automatically converted between metrics.

#### Migrating from MVTools dct

This table maps policies; it does not guarantee bitwise equivalence between DCT backends. Omit new parameters marked with a dash.

| Original `dct` | New `metric` | `metric_weight` | `metric_threshold` |
| ---: | --- | ---: | ---: |
| 0 | `sad` | — | — |
| 1 | `dct` | — | — |
| 2 | `sad_dct_global` | — | — |
| 3 | `sad_dct_local` | 0.5 | 0.03125 |
| 4 | `sad_dct_local` | 0.75 | 0.03125 |
| 5 | `satd` | — | — |
| 6 | `sad_satd_global` | — | — |
| 7 | `sad_satd_local` | 0.5 | 0.03125 |
| 8 | `sad_satd_local` | 0.75 | 0.03125 |
| 9 | `sad_satd_global_half` | — | — |
| 10 | `sad_satd_local` | 0.25 | 0.0625 |

Custom proportions are supported, for example `metric="sad_satd_local", metric_weight=0.6, metric_threshold=0.04`. Weight and threshold are independent. Given identical base errors and trigger decisions, one final truncation can exceed upstream's separately truncated terms by 0–1 for 50%, or 0–2 for 25%/75%. This does not bound final vector differences: candidate rankings can change.


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
s = core.neo_mv.Super(clip, blksize=8, overlap=4, pad=32)
result = core.neo_mv.Analyse(s, delta=1, badrange=0, metric="dct")
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
result = neo_mv_Analyse(s, delta=1, badrange=0, metric="dct")
return result
```

## Restrictions and common errors

Missing Super data, invalid block geometry/enums, delta=0, nonpositive pelsearch, or no usable levels fail at creation. The complete search sampling domain must fit valid Super support. Field mode requires pel>1 and required parity; SATD rejects 6×6 and 16×2; DCT and mixed modes reject float32, and unknown or wrongly cased metric strings are errors. Changed frame metadata or nonfinite calculations fail at evaluation.

## Computation

See [Analyse: computation](../../knowledge/en/analyse.md) for formulas, rounding and numerical examples.

[API index](README.md)
