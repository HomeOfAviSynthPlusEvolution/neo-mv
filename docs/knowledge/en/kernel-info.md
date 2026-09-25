# KernelInfo: query the selected computation backends

`fft` and `fft_lanes` describe the PocketFFT backend used by `DepanEstimate`. They do not describe the DCT block metric, which uses neo-mv's own scalar/Highway transform and strict coefficient quantization.
## 1. What the function computes

`KernelInfo()` reports the selected general computation backend, instruction target, and FFT configuration. It has no video input and computes neither pixels, motion, nor performance scores.

## 2. Objects and notation

In VapourSynth, `core.neo_mv.KernelInfo()` returns four fields:

| Field | Meaning |
| --- | --- |
| `backend` | `scalar` or `highway` |
| `target` | Selected general-kernel target name; `scalar` for scalar execution |
| `fft` | `pocketfft-scalar` or `pocketfft-native` under the public selection path |
| `fft_lanes` | Number of single-precision vector lanes in the FFT backend; 1 for scalar |

Names are string data; lane count is an integer. Lanes do not represent threads, CPU cores, or a performance multiplier.

In AviSynth, `neo_mv_KernelInfo()` returns a native array in the order `[backend, target, fft, fft_lanes]`; indices 0–3 correspond to the fields above.

## 3. Overall calculation

1. Determine the general kernel selection.
2. Query its target name.
3. Select scalar or native FFT from that backend.
4. Return FFT name and lane count alongside the kernel fields.

## 4. Step-by-step calculation

General kernel selection reads `NEO_MV_KERNEL`. `scalar` selects scalar; `highway` requires a build containing Highway. When unset, builds with Highway choose it by default; other builds choose scalar. Other strings, including empty strings, fail. Names are case-sensitive.

The first successful selection is stored in process-static state. Later queries do not reread the environment to switch backends. `target` reports the selected backend's target name.

Scalar kernels select scalar FFT. Highway requests native FFT. On x86 builds containing those targets, selection tries supported AVX-512, AVX2, then SSE2 FFT, with each candidate also requiring an actual multilane implementation. Other targets use the built native implementation. If none is available, FFT falls back to scalar.

Native FFT reports `pocketfft-native` if its resulting lane count exceeds 1, otherwise `pocketfft-scalar`. Thus backend=highway can coexist with fft=pocketfft-scalar. The general-kernel target name is not necessarily an exact FFT instruction-set label.

## 5. A complete numerical example

Set `NEO_MV_KERNEL=scalar` before the first backend initialization, then query:

```text
backend   = scalar
target    = scalar
fft       = pocketfft-scalar
fft_lanes = 1
```

Both general kernels and FFT use scalar configuration. This does not limit the host to one frame request or one worker thread.

## 6. Parameters and their calculation steps

There are no public arguments. `NEO_MV_KERNEL` is process configuration, not a function parameter. Querying cannot replace the loaded build or enable a backend absent from it.

## 7. Boundaries, missing data, and errors

Invalid environment values and explicitly requested unavailable Highway fail. No video frames or image FFT analysis are needed. Failure to write return fields is also an error.

## 8. Precision and determinism

The function queries names and integers, without floating image arithmetic. It helps identify the configuration used by [DepanEstimate](depan-estimate.md) and other functions, but does not establish bitwise equality between configurations. A fixed build/runtime environment reports its selected state.

[Back to the English index](README.md)
