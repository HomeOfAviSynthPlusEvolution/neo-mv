# Shared analysis data: vectors, errors, and availability

## Properties carry calculation data

Analysis nodes carry geometry, vectors, and errors in frame properties. The visible image's dimensions or format do not automatically define those data. Other producers can supply valid data too; consumers do not depend on original node identity.

Property names concatenate `prefix` and suffix directly, for example `MVUtensilsAnalysisPel`. Scalars are host int64 elements. Vectors and errors are int64 arrays, not serialized native structures.

## Descriptor fields

| Suffix after `Analysis` | Meaning |
| --- | --- |
| `Width,Height` | Working dimensions without padding; positive and no smaller than real dimensions |
| `RealWidth,RealHeight` | Positive real-image dimensions |
| `HPad,VPad` | Nonnegative luma padding on each side |
| `Pel` | Vector units per pixel: 1, 2, 4 |
| `Levels` | At least 1; analysis level count, not multiple exported grids |
| `Chroma` | Whether chroma contributes to error; zero/nonzero |
| `XRatioUV,YRatioUV` | Each 1 or 2; GRAY uses 1, 1 |
| `BlkSizeX,BlkSizeY` | Each at least 2; functions add restrictions |
| `OverlapX,OverlapY` | Each 0 through half a block |
| `NBlkX,NBlkY` | Positive grid dimensions |
| `DeltaFrame` | d in reference frame `n+d`; shared format permits 0, though Analyse does not generate it |
| `BitsPerSample` | Integer 8–16, or 32 for floating analysis |

Read scalar element 0; missing, empty, or wrongly typed scalars initially become zero. Ignore later elements. Except for `Chroma`, which uses full int64 zero/nonzero truth, saturate to int32 before validation. Missing `Chroma` can give valid false; missing `Pel` gives invalid zero.

## Decoding arrays

`AnalysisSAD` stores the selected luma metric (SAD, SATD, or DCT), plus chroma SAD when enabled. There is no metric identifier in the descriptor. Consumers use these stored errors directly for thresholds, weights, and masks; they do not recompute pixel SAD or convert the errors to a common metric. Recalculate chooses its own metric for fresh measurements.

Count `N=NBlkX·NBlkY`, row-major index `i=by·NBlkX+bx`. `AnalysisVectors` stores signed X in the low 32 bits and signed Y in the high 32 bits. `AnalysisSAD` stores nonnegative raw error. See [Analyse](../analyse.md) for packing and units.

For padded block origin `x0=hp+bx(Bx-Ox),y0=vp+by(By-Oy)`, vectors must satisfy:

$$-px_0\le v_x<p(W+2h_p-x_0-B_x),\quad
-py_0\le v_y<p(H+2v_p-y_0-B_y).$$

Upper bounds are exclusive. Shared range validation does not replace renderer-specific footprint checks.

## Three states and malformed data

1. Invalid scalar values give **InvalidMetadata**, without reading arrays.
2. Valid metadata with either array absent or not of count N gives **MetadataOnly**. Counts are checked before types or elements.
3. With both counts correct, validate integer types, nonnegative SAD, and vector bounds. Success gives **Complete**. Malformed elements here are errors, not missing data.

Metadata-only operations stop after scalars and do not certify arrays. Complete zero vectors and errors are valid measurements, distinct from missing arrays.

For N=4, vector count 4 and error count 3 give MetadataOnly even if the short array is floating-point. Both counts 4 with floating-point errors instead fail.

## Scene detection and reference availability

[SCDetection](../sc-detection.md) explains scaling error thresholds by area, chroma, and depth, then counting bad blocks. Many consumers reuse it: invalid metadata, missing arrays, or a scene change can make a reference unavailable; complete malformed data remain errors.

Functions respond differently. Analyse omits arrays for out-of-range references; Recalculate may start from zero vectors when old arrays are absent and measure again; renderers may fall back to the center image. The shared decoder does not impose universal zero filling.

[Back to the English index](../README.md)
