# Experimental parent predictors in Analyse

`Analyse` and `AnalyseMany` accept `parentpredict` (boolean, default `false`) in both AviSynth+ and VapourSynth.

At motion boundaries, averaging coarse-level vectors can produce a fine-level starting point far from either real motion. Enabling this option retains the existing interpolated predictor and adds up to four individual vectors from its surrounding parent-level stencil. Each candidate is scaled to the current level and pel precision, receives the field shift when applicable, and is clipped to the block's legal search domain. Duplicate candidates are skipped. Surrounding parents are offered even when their interpolation weight is zero, which can occur with overlap.

Candidates are evaluated against the current block using the selected metric. They receive the same distance penalty as spatial predictors, anchored to the original interpolated predictor. The existing adaptive lambda and penalty controls are unchanged. Additional candidates come after the existing seeds, so equal costs retain the earlier seed.

The existing `trymany` setting controls refinement:

- `trymany=0`: choose the lowest-cost seed, then refine it.
- `trymany=1`: refine each seed on non-finest levels; select then refine on the finest level.
- `trymany=2`: refine each seed on every level and select the lowest-cost result.

The coarsest level has no parent stencil. With only one analysis level, the option has no effect. The default preserves existing behavior. Super and motion-vector formats are unchanged, and downstream filters need no new options.

## VapourSynth

```python
s = core.neo_mv.Super(clip, blksize=[8], overlap=[4], pel=2)
v = core.neo_mv.Analyse(s, delta=1, parentpredict=True)
vectors = core.neo_mv.AnalyseMany(s, radius=1, parentpredict=True)
```

## AviSynth+

```avs
s = neo_mv_Super(clip, blksize=8, overlap=4, pel=2)
v = neo_mv_Analyse(s, delta=1, parentpredict=true)
vectors = neo_mv_AnalyseMany(s, radius=1, parentpredict=true)
```

To compare, reuse the same Super clip and settings with `parentpredict=false` and `true`. Optionally compare `trymany=2`, which spends more time refining seeds rather than selecting one before refinement. This feature adds up to four unique seed evaluations per block below the coarsest level, plus additional searches where `trymany` refines all seeds. It does not enlarge the search radius or guarantee better visual results.

Tests cover a synthetic two-level motion boundary, pel scaling, negative vectors, field shifts, clipping, duplicate candidates, tie ordering, penalties, and refinement modes. Real-clip quality and performance remain experimental.

CI produces a Windows x64 clang-cl DLL supporting both hosts. Download this branch's artifact and load `neo-mv.dll` explicitly. This branch is independent of the Recalculate multi-predictor and chroma subpixel experiments.
