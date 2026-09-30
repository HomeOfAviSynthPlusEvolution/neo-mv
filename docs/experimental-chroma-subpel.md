# Experimental chroma subpixel compensation

`Compensate` and `Degrain` accept `chroma_subpel` (boolean, default `false`) in both AviSynth+ and VapourSynth. Enable it to preserve odd motion-vector components when sampling subsampled chroma.

With 4:2:0, the effective chroma sampling step becomes 1/(2*pel) pixels on both axes. With 4:2:2, only the horizontal axis needs refinement. `pel=1`, `2`, and `4` are supported, including negative vectors. Luma and 4:4:4 sampling are unchanged. Integer samples round to nearest with ties upward; float samples retain fractional values.

This experiment linearly interpolates adjacent existing Super phases. It does **not** generate new direct Wiener phases, change motion estimation, or extend Flow-family filters. Existing Super clips and vectors can be reused. At logical phase boundaries, including cropped pel=4 phases, additional taps replicate the nearest available sample.

The default retains the previous output. The enabled path allocates temporary chroma blocks and performs scalar interpolation before the existing scalar/SIMD composition kernels. It uses additional CPU time and memory, especially with overlapping blocks and many Degrain references. Quality and performance on real clips remain to be evaluated.

## VapourSynth

```python
s = core.neo_mv.Super(clip, pel=2)
v = core.neo_mv.AnalyseMany(s, radius=1)
compensated = core.neo_mv.Compensate(clip, s, v[0], chroma_subpel=True)
denoised = core.neo_mv.Degrain(clip, s, v, chroma_subpel=True)
```

## AviSynth+

```avs
s = neo_mv_Super(clip, pel=2)
v = neo_mv_AnalyseMany(s, radius=1)
compensated = neo_mv_Compensate(clip, s, v[0], chroma_subpel=true)
denoised = neo_mv_Degrain(clip, s, v, chroma_subpel=true)
```

CI builds a Windows x64 clang-cl DLL for both hosts. Download the artifact for this branch and load `neo-mv.dll` explicitly. This branch is independent of the Recalculate multi-predictor experiment.
