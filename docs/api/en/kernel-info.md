# KernelInfo

Query the selected general-kernel and FFT configuration. This function takes no clip and does not process video.

`fft` and `fft_lanes` describe the PocketFFT backend used by `DepanEstimate`. They do not describe the DCT block metric, which uses neo-mv's own scalar/Highway transform and strict coefficient quantization.
## Calling the function

```text
core.neo_mv.KernelInfo()
neo_mv_KernelInfo()
```

## Parameters

None.

## Return value

VapourSynth returns a mapping with the following keys (string data may be exposed as Python bytes). AviSynth returns the native array `[backend, target, fft, fft_lanes]`.

| Key | AVS index | Meaning |
| --- | --- | --- |
| `backend` | 0 | `scalar` or `highway`. |
| `target` | 1 | General-kernel target name; `scalar` for scalar execution. |
| `fft` | 2 | `pocketfft-scalar` or `pocketfft-native`. |
| `fft_lanes` | 3 | Integer count of single-precision FFT lanes; scalar is 1. |

The FFT target may differ from the general-kernel target. Lane count is neither thread count nor an estimated speedup.

## Backend selection and errors

Set `NEO_MV_KERNEL` before the first backend initialization. Exact lowercase `scalar` selects scalar; `highway` requires a build with Highway. When unset, the build chooses Highway if available, otherwise scalar. Other values, including an empty string, are errors.

The first successful selection is cached in the process. Changing the environment later does not switch backends. This query cannot enable a backend absent from the loaded build.

## Minimal examples

Replace the plugin path. The blank clip only gives the script a video output.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
print(core.neo_mv.KernelInfo())
core.std.BlankClip(width=64, height=48, length=12, format=vs.YUV420P8).set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
k = neo_mv_KernelInfo()
Assert(k[3] >= 1)
return BlankClip(width=64, height=48, length=12, pixel_type="YV12")
```

## Computation

See [KernelInfo: backend selection](../../knowledge/en/kernel-info.md).

[API index](README.md)
