# Ordered trajectory sample average

Inputs are samples from the complete prevalidated [trajectory list](kernel-trajectory.md). Output is one render sample. Let c=1+mF+mB. The list always begins with the center sample C, followed by ascending F samples, then ascending B samples. No weights other than multiplicity are used.

For c=1, copy C's stored representation exactly. Do not perform a float conversion, normalization or division. For c>1 and integer render samples, require every sample in [0,M], sum exactly in a sufficiently wide integer and return floor(sum/c). No rounding bias or saturation is added. With 16-bit samples and c<=65537, sum<=4294967295, so signed64 accumulation is sufficient.

For c>1 and float32 samples, require every used value finite, convert C exactly to binary64 as the initial accumulator, then add each F sample and each B sample in list order with a separate binary64 rounding at each addition. Divide the resulting binary64 sum by the exact integer count represented as binary64, round to binary64, then round once to binary32. Require finite intermediate and final values. Do not use a binary32 accumulator, pairwise/reordered summation or a reassociated division. There is no [0,1] clamp.

Repeated positions count separately even when their sample values are equal. The kernel does not look at vector SAD, scene metadata or temporal frame indices. It never averages multiple video frames.

## Examples

- Ordered samples [10,20,30,0,1] sum to61,count5. Integer output is12; float output is fl32(12.2).
- [1,2] gives integer1 or float1.5. A constant integer input stays constant for any legal trajectory count.
- Float samples [2^60,1,-2^60] in that order give binary64 sum0 and output0; rearranging to [2^60,-2^60,1] gives fl32(1/3). This illustrates why the given list order is normative even with a binary64 accumulator.
- With no additional taps, a float center's signed zero or other stored bit pattern is copied without arithmetic. With additional taps, a NaN or infinity in any used sample is an error, even if an attempted alternative averaging algorithm would cancel or ignore it.
