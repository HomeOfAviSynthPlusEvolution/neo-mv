# Experimental staggered and hexagonal motion analysis

This branch adds `layout` to `Analyse` and `AnalyseMany`. Default `0` keeps the
existing rectangular analysis and interpolation algorithms.

| layout | Analysis support | Block centers | FlowFPS reconstruction |
| --- | --- | --- | --- |
| 0 | Rectangle | Existing grid and overlap | Existing resampling and masks |
| 1 | Rectangle | Staggered rows | Triangles and compression mask |
| 2 | Hexagon | Same staggered rows as 1 | Same as 1 |

Compare 1 against 2 to isolate the SAD support shape. Comparing 0 against 2 also
changes the grid density, predictors, interpolation, boundary behavior and masks;
it is not a shape-only comparison. This is an algorithm experiment, not a claim
of better quality or speed. It is not hexagonal pixel resampling: all images and
Super planes retain their ordinary rectangular pixel lattice.

## Geometry and analysis

For block dimensions W,H, center spacing is W horizontally and 3H/4 vertically.
Odd rows shift right by W/2. The hexagon has vertices (W/2,0), (W,H/4),
(W,3H/4), (W/2,H), (0,3H/4), (0,H/4) within its bounding rectangle. Pixel
centers inside or on the boundary contribute to SAD. The same normalized shape
is applied independently on each chroma plane. Each plane's sum is scaled by
bounding rectangle area / actual included sample count before combining planes.
Integer scaling rounds to nearest; floating SAD is normalized before error encoding.
Existing lambda and scene thresholds therefore retain their rectangular-area scale.

Spatial predictors use the preceding block and the two actual neighbors above.
Parent predictions sample a triangular surface at the child's physical center,
then scale displacement for the pyramid level and pel. Both staggered modes use
scalar search and SAD, including in a Highway build. Existing rectangular SIMD
kernels are unchanged. FlowFPS pixel sampling and blending still use their
normal selected backend; the new motion interpolation and masks are scalar.

Blocks remain wholly inside each Super working image. Each row has the same
number of entries, reserving room for the odd-row offset. The finite grid can
leave a border strip without a block center. Triangle vertices outside this grid
repeat the nearest boundary entry; motion is extended rather than extrapolated.
This boundary behavior is part of the experiment. Coarse levels too small for
one staggered row are omitted.

## FlowFPS and restrictions

Vector components are clipped and converted to chroma units as in the existing
dense-flow path, then interpolated with barycentric weights on triangles between
staggered centers. The experimental mask measures local compression of the
triangular motion surface: the negative minimum eigenvalue of its symmetric
spatial gradient, multiplied by time/256 and 80/ml, clipped to [0,1]. Gradients
use centered differences one luma pixel apart. Uniform translation gives zero;
converging motion raises the mask. This is a heuristic occlusion indicator, not
an exact visibility calculation, and differs from the legacy rectangular mask.

- Supported chain: `Super -> Analyse/AnalyseMany -> FlowFPS` in both hosts.
- `layout=1/2` requires SAD, progressive frames, W divisible by 4, H divisible by
  8, and an admitted existing block-size pair. Start with 16x16.
- Working width must be at least 3W/2 and height at least H.
- Staggered spacing replaces Analyse's overlap setting with [0,H/4]. Super's
  overlap hint still affects its working-image extent; keep Super identical for A/B.
- pel 1/2/4, integer and float samples, and existing chroma ratios are supported.
- Other vector consumers, including Recalculate, Compensate, Degrain, Mask,
  Flow, FlowInter and FlowBlur, reject staggered vectors in this branch.
- Vectors carry the additional `AnalysisLayout` property under the chosen prefix.
  Missing means 0 for old rectangular vectors. Do not feed experimental vectors
  into older plugins that do not understand this property.

## VapourSynth

```python
s = core.neo_mv.Super(clip, blksize=[16], overlap=[0], pad=[32], pel=2)
v = core.neo_mv.AnalyseMany(s, radius=1, layout=2, metric="sad")
result = core.neo_mv.FlowFPS(clip, s, v, num=60000, den=1001)
```

## AviSynth

```avs
s = neo_mv_Super(clip, blksize=16, overlap=0, pad=32, pel=2)
v = neo_mv_AnalyseMany(s, radius=1, layout=2, metric="sad")
result = neo_mv_FlowFPS(clip, s, v, num=60000, den=1001)
```

Change only `layout` to compare modes. There is no new six-direction motion-search
pattern: the existing candidate search is reused with the new support and predictors.
CI builds one Windows x64 clang-cl DLL and uploads it with this note.
