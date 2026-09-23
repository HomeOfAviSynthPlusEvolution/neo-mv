# neo-mv computation knowledge base

These articles explain how each public neo-mv function turns inputs into outputs: data representations, operation order, formulas, where parameters enter the calculation, and how boundaries and rounding affect results. Each function includes a numerical example; shared operations have separate articles.

Function names in these articles omit the host prefix. Use `core.neo_mv.Super(...)` in VapourSynth and `neo_mv_Super(...)` in AviSynth; the same rule applies to the other functions. Computations and parameter names are shared. AviSynth accepts native arrays such as `[8, 8]`; `AnalyseMany` and `Recalculate` return arrays of clips, including when `Recalculate` produces one clip. `KernelInfo` returns an array in the order `[backend, target, fft, fft_lanes]`.

## Image levels and block motion

| Function | Calculation |
| --- | --- |
| [Super](super.md) | Border extension, reduction pyramids, and subpixel phases |
| [Analyse](analyse.md) | Predictors, multilevel search, block errors, and vector selection |
| [AnalyseMany](analyse-many.md) | Independent temporal analyses and output ordering |
| [Recalculate](recalculate.md) | Mapping old vectors to a new grid and searching locally |
| [SCDetection](sc-detection.md) | Error thresholds, bad-block counts, and scene flags |

## Images from block motion

| Function | Calculation |
| --- | --- |
| [Compensate](compensate.md) | Reference sampling, current-block fallback, and overlap composition |
| [Degrain](degrain.md) | Reference reliability, integer weight normalization, overlap, and limiting |
| [Degrain1–Degrain25](degrain.md#named-entry-points) | Fixed pair-count entry points; the table lists each vector count |

## Motion masks and per-pixel displacement

| Function | Calculation |
| --- | --- |
| [VectorLengthMask](vector-length-mask.md) | Vector lengths mapped to grayscale |
| [SADMask](sad-mask.md) | Error projection, normalization, powers, and quantization |
| [OcclusionMask](occlusion-mask.md) | Convergence events from neighboring vectors |
| [Flow](flow.md) | Dense displacement fields and reference-phase sampling |
| [FlowInter](flow-inter.md) | Bidirectional interpolation at a fixed time with occlusion handling |
| [FlowFPS](flow-fps.md) | Rational timeline mapping and bidirectional interpolation |
| [FlowBlur](flow-blur.md) | Sampling and averaging along forward/backward trajectories |

## Global motion and geometry

| Function | Calculation |
| --- | --- |
| [DepanAnalyse](depan-analyse.md) | Iterative translation, rotation, and scale fitting from block motion |
| [DepanEstimate](depan-estimate.md) | Periodic correlation peaks and two-window scale estimation |
| [DepanCompensate](depan-compensate.md) | Composing global motions and resampling an image |
| [DepanStabilise](depan-stabilise.md) | Cumulative paths, temporal smoothing, correction limits, and layers |
| [KernelInfo](kernel-info.md) | General-kernel and FFT configuration queries |

## Shared calculations

| Article | Contents |
| --- | --- |
| [Analysis data](shared/analysis-data.md) | Metadata, vector encoding, and availability states |
| [Block rendering](shared/block-rendering.md) | Sample coordinates, planes, overlap windows, and integer sums |
| [Grid resampling](shared/grid-resampling.md) | Block-to-pixel coordinates, coefficients, and rounding |
| [Mask inputs](shared/mask-input.md) | Shared inputs, unavailable-field fill, and output properties |
| [Bidirectional interpolation](shared/bidirectional-interpolation.md) | Time scaling, occlusion, extra vectors, and pixel composition |
| [Global motion](shared/global-motion.md) | Depan properties, map conversions, composition, and inverse |
| [Depan sampling](shared/depan-sampling.md) | Coordinate classes, nearest/bilinear/bicubic sampling, and edges |
| [Stabilization smoothing](shared/stabilisation-smoothing.md) | Inertial recurrence, window averaging, adaptive zoom, and recovery |
| [Stabilization layers](shared/stabilisation-layers.md) | Neighbor images, map accumulation, and ordered overwriting |
