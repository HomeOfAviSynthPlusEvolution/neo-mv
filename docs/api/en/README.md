# API reference

Signatures, defaults, parameter ranges, input/output requirements and runnable VapourSynth/AviSynth examples for all 44 public entries. The 25 numbered Degrain variants share one page with Degrain.

Use `core.neo_mv.Name` in VapourSynth and `neo_mv_Name` in AviSynth. Load the plugin with `core.std.LoadPlugin` or `LoadPlugin`, respectively. Examples use a blank source and a placeholder plugin path; replace that path before running.

Array parameters accept one scalar as a one-element array. In Python, use lists such as `[16, 8]`; AviSynth also uses native arrays such as `[16, 8]`. A missing argument and an explicit empty array are not always equivalent; each page documents this. `AnalyseMany` and `Recalculate` return arrays of clips, even when Recalculate has one input.

| Function | Purpose |
| --- | --- |
| [Super](super.md) | Build multilevel and subpixel reference images. |
| [Analyse](analyse.md) | Estimate block motion from each frame to a reference at n+delta. |
| [AnalyseMany](analyse-many.md) | Create an ordered array of forward/backward analysis clips. |
| [Recalculate](recalculate.md) | Map existing vectors onto a target grid and optionally refine them using fresh sample errors. |
| [SCDetection](sc-detection.md) | Attach scene-change flags using stored block errors. |
| [Compensate](compensate.md) | Render block motion compensation from one vector clip. |
| [Degrain / Degrain1–Degrain25](degrain.md) | Combine the current image with motion-compensated references weighted by matching error. The named variants fix the number of reference pairs. |
| [Flow](flow.md) | Render per-pixel motion compensation after expanding the block vector grid. |
| [FlowInter](flow-inter.md) | Interpolate between frame n and n+d without changing the timeline. |
| [FlowFPS](flow-fps.md) | Resample the timeline at a target frame rate using motion interpolation. |
| [FlowBlur](flow-blur.md) | Average samples along motion trajectories around the current frame. |
| [VectorLengthMask](vector-length-mask.md) | Visualize block-vector magnitude. |
| [SADMask](sad-mask.md) | Visualize stored block errors after vector-based projection. |
| [OcclusionMask](occlusion-mask.md) | Build an occlusion mask from differences between neighboring block vectors. |
| [DepanAnalyse](depan-analyse.md) | Fit adjacent-frame global motion from existing block vectors. |
| [DepanCompensate](depan-compensate.md) | Apply global-motion compensation at an integer or fractional frame offset. |
| [DepanEstimate](depan-estimate.md) | Estimate adjacent-frame translation and optional zoom from first-plane FFT correlation. |
| [DepanStabilise](depan-stabilise.md) | Smooth global motion and render the corrected image, optionally filling borders from neighboring frames. |
| [KernelInfo](kernel-info.md) | Query the selected computation and FFT backends. |

For the calculation behind each function, see the [computation knowledge base](../../knowledge/en/README.md).
