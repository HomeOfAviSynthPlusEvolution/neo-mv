# neo-mv

**English** | [简体中文](README.zh-CN.md) | [日本語](README.ja.md)

neo-mv is a motion-processing plugin for VapourSynth and AviSynth. It provides block motion estimation, motion compensation, temporal blending, motion masks, frame interpolation, and global motion estimation and stabilization, with an API based on MVUtensils.

The implementation uses C++17, with scalar kernels, cross-platform SIMD through Google Highway, and PocketFFT for frequency-domain motion estimation. DualSynth2 connects the computation core to both hosts. VapourSynth uses `core.neomv`; AviSynth uses functions prefixed with `neo_mv_`.

## Design

neo-mv separates motion calculations from host frame management. The core operates on image planes, motion fields, and explicit geometry; the host layer manages clips, frame requests, properties, and output allocation. The core can be built and tested independently.

The implementation was developed from behavioral specifications, with explicit rules for search order, rounding, boundaries, and unavailable motion. Scalar implementations provide the arithmetic reference, while SIMD paths are checked against their numerical contracts. Familiar function names do not guarantee identical output to every historical MVTools or MVUtensils build.

## Supported operations

| Area | Functions |
|---|---|
| Working images | `Super`: image pyramids, extended borders, and subpixel phases. |
| Block motion | `Analyse`, `AnalyseMany`, `Recalculate`, `SCDetection`: motion search, multiple temporal distances, refinement, and scene flags. |
| Block rendering | `Compensate`, `Degrain`, `Degrain1`–`Degrain25`: motion-compensated sampling and weighted temporal composition. |
| Motion masks | `VectorLengthMask`, `SADMask`, `OcclusionMask`: vector magnitude, block error, and occlusion maps. |
| Per-pixel motion | `Flow`, `FlowInter`, `FlowFPS`, `FlowBlur`: dense displacement, interpolation, frame-rate conversion, and trajectory averaging. |
| Global motion | `DepanAnalyse`, `DepanEstimate`, `DepanCompensate`, `DepanStabilise`: motion fitting, FFT correlation, geometric compensation, and stabilization. |
| Diagnostics | `KernelInfo`: selected computation and FFT backends. |

Block-motion and Flow operations support planar GRAY/YUV with 8–16-bit integer or 32-bit floating-point samples. Format and subsampling restrictions vary by function. `DepanAnalyse`, `DepanCompensate`, and `DepanStabilise` use integer images; `DepanEstimate` also accepts float32. RGB is not supported.

Motion data is carried in frame properties. The default property prefix is `MVUtensils`, independently of the `neomv` plugin namespace. Super's auxiliary images belong to the implementation that created them; generate Super with neo-mv for use by neo-mv consumers.

## Documentation and use

The computation knowledge base explains how each function turns inputs into outputs: representations, formulas, operation order, parameter effects, numerical examples, boundaries, and precision.

- [English knowledge base](docs/knowledge/en/README.md)

Load the built plugin explicitly, or place it in VapourSynth's plugin autoload directory. The example below uses the Windows filename; substitute the actual plugin path for your platform.

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")

print(core.neomv.KernelInfo())
clip = core.std.BlankClip(width=640, height=360, format=vs.YUV420P8, length=24)
super_clip = core.neomv.Super(clip, blksize=16, overlap=8, pad=32, pel=2)
vectors = core.neomv.Analyse(super_clip, delta=1)
output = core.neomv.Compensate(clip, super_clip, vectors)
output.set_output()
```

This minimal example uses a synthetic clip to show the calling sequence. `Super` requires explicit block size and overlap. Positive `delta` refers to a later frame; negative `delta` refers to an earlier frame. See the function articles for full parameter and calculation details.

The same plugin also provides an AviSynth C interface. Use `LoadCPlugin` with AviSynth+ 3.7.4 or later (interface 11), or a compatible AviSynthMinus runtime:

```avs
LoadCPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=640, height=360, length=24, pixel_type="YV12")
super_clip = neo_mv_Super(clip, blksize=16, overlap=8, pad=32, pel=2)
vectors = neo_mv_AnalyseMany(super_clip, radius=1)
return neo_mv_Degrain1(clip, super_clip, vectors)
```

Function names and parameter order follow the knowledge base, with the `neo_mv_` prefix. Array parameters accept native arrays such as `[16, 8]`; a scalar is shorthand for one element. `AnalyseMany` and `Recalculate` return clip arrays, including a one-element array for a single Recalculate input. `neo_mv_KernelInfo()` returns `[backend, target, fft, fft_lanes]`. Boolean parameters use `true`/`false`. Audio and parity are forwarded from the first input clip; field calculations use `_Field` properties or an explicit `tff`. Depan's `info=true` uses AviSynth's `propShow` to draw its diagnostic property.

## SIMD and CPU selection

SIMD builds select a compiled Highway target supported by the running CPU. Scalar fallback remains available. Set `NEO_MV_KERNEL=scalar` before the first backend initialization to use scalar kernels and scalar FFT; `NEO_MV_KERNEL=highway` explicitly requires a SIMD-enabled build. With the variable unset, the build selects its default backend.

Selection is cached after the first successful initialization. Changing the environment afterward does not switch existing or new filter instances to another backend. `core.neomv.KernelInfo()` reports `backend`, `target`, `fft`, and `fft_lanes`; the FFT target can differ from the general-kernel target. Lane count is not a thread count or a speedup estimate.

FFT profiles and permitted floating-point differences can affect results. Wider SIMD does not guarantee higher throughput. See [KernelInfo](docs/knowledge/en/kernel-info.md) and each function's precision section.

## Building and testing

Requires CMake 3.24 or later, Git, and a C++17 compiler. CMake retrieves pinned DualSynth2 and PocketFFT sources, plus Highway 1.4.0 when SIMD is enabled. Both host SDKs are discovered locally or fetched automatically. Running VapourSynth tests requires a matching runtime and `vspipe`; AviSynth tests require a matching runtime library.

```sh
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/release --config Release --parallel 4
ctest --test-dir build/release -C Release --output-on-failure
```

| Option | Purpose |
|---|---|
| `NEO_MV_BUILD_VAPOURSYNTH=OFF` | Disable the VapourSynth entry point. |
| `NEO_MV_BUILD_AVISYNTH=OFF` | Disable the AviSynth entry point. Set both host options to OFF for a core-only build. |
| `NEO_MV_ENABLE_SIMD=OFF` | Disable SIMD kernels and vectorized FFT profiles. |
| `BUILD_TESTING=OFF` | Build without tests. |
| `NEO_MV_TEST_VAPOURSYNTH=OFF` | Skip host tests while retaining core tests and the plugin. |
| `NEO_MV_VSPIPE_EXECUTABLE=/path/to/vspipe` | Select the runtime used for host tests. |
| `NEO_MV_VS_SDK=/path/to/sdk` | Select a local VapourSynth SDK. |
| `NEO_MV_AVS_SDK=/path/to/sdk` | Select a local AviSynth SDK. |
| `NEO_MV_TEST_AVISYNTH=ON` | Enable AviSynth host tests (off by default). |
| `NEO_MV_AVISYNTH_RUNTIME=/path/to/avisynth.dll` | Select the runtime library for AviSynth host tests. |
| `FETCHCONTENT_SOURCE_DIR_DUALSYNTH2=/path/to/dualsynth2` | Use a local DualSynth2 checkout instead of the pinned download. |
| `NEO_MV_BUILD_BENCHMARKS=ON` | Build manually invoked kernel benchmarks; requires SIMD. |

The plugin target retains the name `neo_mv_vs`, with output basename `neo-mv`; it contains both entry points by default. Tests cover arithmetic, motion search, image sampling, property validation, scalar/SIMD comparisons, boundaries, and host behavior. Separate black-box tests compare public behavior with MVUtensils; they require an external reference installation.

CI configures Windows x64, Linux x64, macOS ARM64, and Linux ASan/UBSan checks. The release workflow builds Windows, Linux, and macOS on x64 and ARM64; VapourSynth host tests in that workflow currently run on Windows x64; AviSynth runtime tests are enabled separately with the options above.

## Performance

Existing measurements give approximately **0.62–1.83× MVUtensils R9 throughput** across the measured filter paths. The ratio is **neo-mv throughput / MVUtensils throughput**, equivalently MVUtensils time / neo-mv time; **above 1 means neo-mv is faster**. Each range below spans 8-/16-bit and AVX2/AVX-512 configurations, not a confidence interval.

| Function | Relative throughput |
|---|---:|
| Super | 1.11–1.44× |
| Analyse | 0.71–0.81× |
| Recalculate | 0.75–0.84× |
| Compensate | 0.80–1.03× |
| Degrain1 | 0.88–1.19× |
| Degrain2 | 0.94–1.38× |
| VectorLengthMask | 1.17–1.83× |
| SADMask | 0.84–1.56× |
| OcclusionMask | 0.81–1.08× |
| Flow | 1.26–1.59× |
| FlowInter | 1.07–1.58× |
| FlowFPS | 1.03–1.67× |
| FlowBlur | 1.30–1.35× |
| DepanAnalyse | 0.73–0.93× |
| DepanCompensate (bilinear) | 0.79–0.94× |
| DepanCompensate (bicubic) | 1.15–1.49× |
| DepanEstimate | 0.62–0.81× |
| DepanStabilise | 1.06–1.42× |

The ranges summarize VapourSynth measurements of individual filters on fixed 1080p YUV420P8/P16 inputs with one host thread. Upstream images are precomputed, so these figures do not represent whole-pipeline throughput. Results vary with inputs, parameters, hardware, and thread count.

## Development and contributions

The maintainer directs development, reviews changes, and is responsible for releases. Bug reports, suggestions, and contributions are welcome. Discuss numerical semantics, public interface changes, and substantial architectural changes before implementation.

This project uses AI-assisted implementation, tests, and review. Contributions should explain the problem, approach, validation, and how AI was involved. Reports should include the version, OS, CPU, compiler, build options, input/output formats, and a minimal reproducer; performance reports should also describe dimensions, backend selection, and the measurement method.

## Acknowledgments and license

Thanks to the authors and contributors of MVTools and MVUtensils for their work on motion processing, and to the developers and users who contribute tests, reports, and improvements. neo-mv uses DualSynth2 for host integration, Google Highway for SIMD, and PocketFFT for FFT computation.

Thanks to [SB.SB](https://sb.sb) for sponsoring the LLM subscription used in this project's development.

neo-mv is licensed under the GNU General Public License, version 2 or later (`GPL-2.0-or-later`). See [LICENSE](LICENSE). Third-party components retain their own copyright notices and license terms.
