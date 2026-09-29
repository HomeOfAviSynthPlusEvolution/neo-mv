# Recalculate multi-predictor experiment

This branch adds `multipredict` (boolean, default `false`) to Recalculate.
The Windows x64 DLL supports both AviSynth and VapourSynth.

```python
# VapourSynth: use your existing Super and vector list.
refined = core.neo_mv.Recalculate(super, vectors, multipredict=True)
```

```avs
# AviSynth: use your existing Super and vector array.
refined = neo_mv_Recalculate(super, vectors, multipredict=true)
```

When enabled, each target block evaluates both the existing linear (`smooth=true`)
and nearest-neighbour (`smooth=false`) predictions using the requested metric.
Identical positions are evaluated once. The smaller raw error wins; ties keep
the prediction selected by `smooth`. The selected prediction is then refined
only if its error exceeds `thsad`, using the existing search and penalty rules.

This selects a starting point **before** refinement; it does not run two full
Recalculate searches and compare their final results. Vector output format is
unchanged. With `multipredict=false`, existing behaviour is preserved.

For feedback, compare the same script with this option off/on and report your
Recalculate parameters, input sample, affected frames and processing speed.
This experimental feature does not promise to eliminate interpolation stripes.

Downloads: open this branch's successful **CI** run in GitHub Actions and download
the `neo-mv-windows-x64-clang-cl-<commit>` artifact. The archive contains
`neo-mv.dll`, this usage note and the license.
