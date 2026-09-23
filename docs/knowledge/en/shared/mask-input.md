# Shared inputs and numerical rules for the three masks

## Analysis metadata defines the output

VectorLengthMask, SADMask, and OcclusionMask read vector-frame properties only, not carrier pixels, Super, or `n+delta` images. Output is GRAY at the analysis real dimensions, with frame count/rate from vectors. Precision 8–16 produces matching integer output; 32 produces float32. Integer maximum is M=2^b−1; floating score maximum is M=1.

Creation permits metadata-only data and requires complete visible block coverage within working dimensions. Per-frame invalid metadata, absent arrays, or a scene rejection selects scval. Complete malformed arrays and changed valid descriptors fail. All valid scalars, including delta, stay fixed except positive Levels. See [analysis data](analysis-data.md).

## Parameter conversion and quantization

Convert ml, gamma, scval to binary32; require finite values, ml>0, gamma≥0. Time is finite binary64 in 0–100. Constants are:

$$f=fl32(1/ml),\quad t=trunc(fl64(fl64(time\cdot256)/100)).$$

Function-specific derived constants are also checked at creation. Underflow to zero is permitted, but subnormals must not be deliberately flushed. Require finite score L before:

$$Q(L)=\begin{cases}trunc(\min(L,M))&\text{integer}\\fl32(\min(L,M))&\text{float}.\end{cases}$$

Integer scores truncate without +0.5. The fallback uses a different rule: `trunc(fl32(scval+0.5))`, whose mathematical integer must be in [0, M]. Floating fallback uses scval directly, even outside [0, 1]. Integer scval=12.5 gives 13, while score L=12.5 gives 12.

## Powers

For nonnegative bases/exponents, `pow(0,0)=1,pow(x,0)=1,pow(0,g>0)=0,pow(x,1)=x`. Other powers use the stated binary32/64 precision, admitting a correctly rounded result or an adjacent finite nonnegative representable value, stable for repeated inputs. Subsequent truncation uses that actual value; there is no extra final-pixel tolerance.

Gamma zero does not waive earlier finite-intermediate checks. OcclusionMask computes contributions only for actual events with nonempty destination intervals; absent events do not turn white through pow(0, 0).

## Pixels and properties

Available fields first produce quantized block values, then [resample to pixels](grid-resampling.md). Unavailable fields directly fill converted scval, without grid calculations.

Output properties contain only `_Range=[1]`, this interface's full-range marker. Analysis, Super, scene, and color properties are not inherited. Do not substitute the older `_ColorRange`, whose numeric convention differs. The range marker does not rescale pixels.

There is no temporal boundary test: valid complete external vectors at the last frame can generate a mask even with delta=1. Scene defaults are thscd1=400, thscd2=51.0; see [SCDetection](../sc-detection.md) for domains and calculation.

[Back to the English index](../README.md)
