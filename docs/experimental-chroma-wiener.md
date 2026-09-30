# Experimental runtime Wiener chroma sampling

`Compensate` and `Degrain` accept `chroma_wiener` (boolean, default `false`) in both AviSynth+ and VapourSynth. This branch is independent of the other experiments.

When enabled, subsampled chroma is sampled directly from the integer (phase-zero) Super plane with a separable six-tap filter. Motion-vector precision is retained: 4:2:0 uses steps of 1/(2*pel) chroma pixels on both axes, reaching 1/8 at `pel=4`; 4:2:2 uses that horizontal precision and 1/pel vertically. Negative displacements use floor coordinates. Every fractional position in these chroma planes uses the direct filter, including positions already representable by the old Super phases. Integer positions borrow the original samples. Luma, 4:4:4, and grayscale sampling are unchanged.

## Kernel definition

The half-pixel kernel is the existing Wiener six-tap filter `[1, -5, 20, 20, -5, 1] / 32`. Other positions linearly interpolate **coefficients** between integer and half-pixel positions. With tap offsets -2 through +3, every row below is divided by 128:

| Phase | -2 | -1 | 0 | +1 | +2 | +3 |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 128 | 0 | 0 | 0 |
| 1/8 | 1 | -5 | 116 | 20 | -5 | 1 |
| 2/8 | 2 | -10 | 104 | 40 | -10 | 2 |
| 3/8 | 3 | -15 | 92 | 60 | -15 | 3 |
| 4/8 | 4 | -20 | 80 | 80 | -20 | 4 |
| 5/8 | 3 | -15 | 60 | 92 | -15 | 3 |
| 6/8 | 2 | -10 | 40 | 104 | -10 | 2 |
| 7/8 | 1 | -5 | 20 | 116 | -5 | 1 |

The quarter/half rows agree with the coefficient ratios in [DTL's referenced implementation](https://github.com/DTL2020/mvtools/blob/9eedb9d0850f638fc43212fb515cf048f9b9a58f/Sources/MVPlane.cpp#L126). Its commented eighth-phase table is unfinished; the odd eighth rows above explicitly extend the same coefficient interpolation rule. They are not independently optimized Wiener kernels.

The earlier `chroma_subpel` experiment interpolated already-generated Super phases. This version evaluates the filter directly from phase zero, retains signed intermediate values, and rounds/clips only after both axes. Integer output is rounded to nearest with ties upward and clipped to the declared bit depth. Float uses double intermediates, preserves finite overshoot, and rejects non-finite/unrepresentable results. Taps outside the logical integer plane replicate its nearest edge sample.

In exact arithmetic, away from edges and without intermediate clipping, these coefficients describe the same interpolant as blending the corresponding Wiener half-pixel samples. The main differences are intermediate precision/clipping, edge handling, and direct block sampling. No general visual-quality improvement is claimed.

## Usage

VapourSynth:

```python
s = core.neo_mv.Super(clip, blksize=[8], overlap=[4], pel=4, sharp=2)
v = core.neo_mv.AnalyseMany(s, radius=1)
compensated = core.neo_mv.Compensate(clip, s, v[0], chroma_wiener=True)
denoised = core.neo_mv.Degrain(clip, s, v, chroma_wiener=True)
```

AviSynth+:

```avs
s = neo_mv_Super(clip, blksize=8, overlap=4, pel=4, sharp=2)
v = neo_mv_AnalyseMany(s, radius=1)
compensated = neo_mv_Compensate(clip, s, v[0], chroma_wiener=true)
denoised = neo_mv_Degrain(clip, s, v, chroma_wiener=true)
```

Super's `sharp` and external `pelclip` fractional samples do not control the enabled chroma path: it always uses the fixed kernel above on phase zero. Motion estimation and luma compensation continue using the original Super phases. For comparisons with the earlier linear experiment, use `sharp=2` and no external `pelclip` so the intended interpolation kernels correspond.

This experiment supports 8/16-bit integer storage and float. It does not change motion estimation, Flow filters, Super storage, or the vector format. Super still builds its usual phases, so this version does not reduce Super memory. Runtime filtering is scalar, followed by the existing scalar/SIMD composition kernels, and allocates temporary blocks. CPU and memory costs, especially with overlap and many Degrain references, still need practical evaluation.

CI builds a Windows x64 clang-cl DLL for both hosts. Download this branch's artifact and load `neo-mv.dll` explicitly.
