# DepanEstimate

Estimates global motion from adjacent image windows. Returns one video node under `clip`, preserving the input format, dimensions, frame rate and frame count. Interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| clip | Required node | Constant GRAY or YUV, integer 8..16 or float32, host-supported format |
| trust | 4.0 | Finite binary32 in [0,100], local and temporal confidence threshold |
| winx | 0 | Saturated int32; nonnegative even pre-halving width, zero automatic |
| winy | 0 | Saturated int32; nonnegative height, zero automatic |
| wleft | -1 | Saturated int32; negative automatic horizontal placement |
| wtop | -1 | Saturated int32; negative automatic vertical placement |
| dxmax | -1 | Saturated int32; negative automatic horizontal search limit |
| dymax | -1 | Saturated int32; negative automatic vertical search limit |
| zoommax | 1.0 | Finite binary32; exactly 1 selects one window, any other value selects two |
| stab | 1.0 | Finite binary32 displacement penalty coefficient; negative values are permitted |
| pixaspect | 1.0 | Finite positive binary32 aspect used for vertical motion |
| info | false | Diagnostic property followed by host text rendering |
| show | false | Display the current correlation surfaces in the first plane |
| fields | false | Field-parity-dependent vertical correction |
| tff | Omitted | Optional boolean parity override |

There is no public prefix, FFT backend or thread-count argument in this interface. A build/runtime FFT configuration must not silently add required script parameters. RGB is unsupported. YUV chroma subsampling is irrelevant to estimation because only the full-resolution first plane is analyzed; do not reject an otherwise host-supported planar YUV format merely because its subsampling differs from 420/422/444. No automatic depth/color conversion is performed.

## Creation and evaluation

Require frame count F in [1,2147483647] and positive constant dimensions. Validate all supplied arguments and [window geometry](kernel-window-geometry.md). Check shape/stride/allocation representability without narrow integer overflow. Establish the [FFT profile](kernel-fft-execution.md). No pixel or parity data is required at creation. info=true must establish an available valid renderer invocation.

For output n, obtain the basic observations required by [temporal validity](kernel-temporal-validity.md). For each basic index j, compare clip[max(0,j-1)] to clip[j] using the same admitted rectangles. With fields=true, require clip[j]'s parity or explicit tff even if that record will be rejected, including j=0. With fields=false, `_Field` and tff do not affect estimation.

Generate [correlation](kernel-fft-correlation.md), find [peak/confidence](kernel-peak-confidence.md), and [refine motion](kernel-subpixel-motion.md) for each used rectangle. Apply [window combination](kernel-two-window-motion.md), including the first-frame override. Apply the temporal validity rule to determine final motion. Missing required frames, invalid used samples or arithmetic/FFT failures are frame errors, not scene changes.

Start output pixels and properties from clip[n]. With show=false, preserve all samples exactly, including unused float NaNs. With show=true, apply [correlation display](kernel-correlation-display.md) only for observation n; invalid motion does not suppress display. Do not display a neighboring surface. All untouched samples remain exact copies. The input video and its properties are immutable.

## Output properties

Replace `Depan_dx`, `Depan_dy`, `Depan_rot`, `Depan_zoom`, `Depan_goodmotion` according to [motion transport](../../phase-5/depan-analyse/kernel-motion-properties.md), regardless of inherited types/counts. No producer magnitude clamps are applied. Remove every occurrence of the following keys, including an inherited key of another type or count:

- `DepanEstimateFFT`
- `DepanEstimateFFT2`
- `DepanEstimateX`
- `DepanEstimateY`
- `DepanEstimateZoom`
- `DepanEstimateGood`
- `DepanEstimateTrust`

This is an output property-removal contract only; these names need not exist as intermediate properties. Their inherited contents are never trusted as spectra, confidence or motion input. Do not remove other similarly named keys. No range, matrix, chroma-location or scene-change property is implicitly rewritten. info=false preserves an inherited `DepanEstimate_info`; info=true replaces it and applies [diagnostics](kernel-diagnostics.md).

## Examples

- With show=false,info=false, every output image is a bit-preserving copy of clip[n], while the five motion properties can change. The first frame always exports (0,0,0,1,0).
- A three-frame 4x4 GRAY8 clip can use explicit winx=winy=4,dxmax=dymax=1,stab=0,zoommax=1. Let each frame have one sample 1 and all others zero, at (2,0),(1,0),(0,0) respectively. The ideal pair surfaces at n=1,2 peak at (1,0) with value 16. They produce dx=1,dy=0,zoom=1 and confidence approximately 80.80808; neither temporal check rejects at trust=4. Frame zero remains invalid. This example fixes sign and normalization; the concrete FFT profile supplies its ordinary numerical approximation.
- In a one-frame clip, source frame 0 and its windows/parity are still required; final motion is invalid. show=true can still fail on a constant correlation surface.
- fields=true with a missing required `_Field` fails even when trust would reject that observation. Explicit tff removes the frame-property requirement and alternates parity by each basic observation's index.
- A stale inherited `DepanEstimateFFT` byte string is removed, without decoding it. An inherited `DepanEstimate_info` remains unchanged when info=false.
