# Limiting the final change from the centre image

Inputs: the composed visible plane q, current Super[n] integer-phase visible samples C, render sample type/depth, and a user limit converted to binary32. Output: the final processed visible plane. Apply this once after block composition, never separately to each reference or block contribution.

Validate both luma and chroma limit arguments at creation even if a plane is not processed. Any non-finite limit disables limiting. A finite limit must be strictly positive; +0, -0 and negative finite values are errors. This parameter-specific rule for non-finite values does not permit non-finite image samples.

## Integer output

Let M=2^b-1. If limit>=M, limiting is disabled. Otherwise define L=trunc(fl32(limit+0.5)), which can be zero for a positive argument. For every pixel,

$$out=\min(\min(M,C+L),\ \max(\max(0,C-L),q)).$$

Use wide integer arithmetic for C+L and C-L. This clamps to the centre interval intersected with the legal sample range. L=0 returns C. The input q is already a valid sample after composition.

## Float output

Every finite positive limit is active, including values larger than 1. Define lo=fl32(C-limit), hi=fl32(C+limit) and out=min(hi,max(lo,q)). Require finite endpoints and intermediates; otherwise return a controlled frame error. Do not clamp the result to [0,1]. When q lies within [lo,hi], preserve q, including its zero sign; when below/above the interval return the corresponding endpoint. Disabled limiting copies q exactly.

Each pixel is independent; input planes are immutable and the output must not alias them. The centre image is Super[n], not clip[n]. No property or vector data changes.

## Examples

- GRAY8, C=100,q=107,limit=2.2f gives L=2 and out=102. limit=0 is a creation error; limit=0.1f is valid, L=0 and out=100.
- 10-bit C=1020,q=1023,limit=2 gives out=1022. The sample maximum is 1023, not 65535.
- Float C=0.25,q=0.9,limit=0.1f gives out=fl32(0.25+0.1f). Float limit=2 remains active; it does not imply a normalized-video bypass.
- Positive infinity, negative infinity and NaN each disable this operator. A finite positive argument whose binary32 conversion becomes +0 fails the finite-positive check.
