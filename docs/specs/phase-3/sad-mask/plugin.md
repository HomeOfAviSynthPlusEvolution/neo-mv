# SADMask

Returns a grayscale visualization of projected block SAD. The interface, in order, is

`SADMask(vectors, ml=100.0, gamma=1.0, time=100.0, scval=0.0, thscd1=400, thscd2=51.0, prefix="MVUtensils") -> clip`.

Argument types, conversions, domains, metadata requirements, scene eligibility, fallback, output video description and exact output-property rule are the same as [VectorLengthMask](../vector-length-mask/plugin.md) and its [shared mask input contract](../vector-length-mask/kernel-mask-input.md). Here ml scales SAD per block area and time controls the projection in [projected SAD](kernel-sad-projection.md); it is not ignored. There is no Super argument or reference-image request.

At creation validate that kernel's normalization and projection constants. At each eligible frame generate its SAD mask grid and [resize it](../vector-length-mask/kernel-grid-resampling.md). On ineligible frames fill the entire visible output with the converted scval. Evaluation errors do not become scval fallback. No inherited property is retained; the result has only `_Range=[1]`.

For GRAY8 analysis with actual/working dimensions 8x8, one 8x8 block, overlap zero, pad=4,p=1, vector (0,0), SAD=8, ml=1,gamma=1,time=100 and default scene thresholds, output is an 8x8 GRAY8 image of 127. With a missing SAD array and scval=9, output is all 9. A complete negative SAD fails instead. Frame count and rate always come from vectors, not from a referenced frame index.
