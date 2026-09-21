# Correlation peak and confidence

Input is a finite binary32 correlation surface C, dimensions wx,wy, admitted search limits mx,my, finite stab, and trust limit t in [0,100]. Output is peak indices (imax,jmax), signed integer peak displacement (dx,dy), binary32 confidence T and a local-valid flag. Arithmetic follows the [common scalar rules](../README.md#common-contracts).

## Ordered search

Visit rows in order [0,...,my,wy-my,...,wy-1]. Within each row visit columns [0,...,mx,wx-mx,...,wx-1]. The second interval is empty when the corresponding limit is zero. The geometry prevents overlap. There are J=(2mx+1)(2my+1) visited samples.

Initialize the maximum to C(0,0), its indices to (0,0), and the sum to +0. For every visited sample, add it to the sum with one binary32 addition. Replace the maximum and indices only when the new value is strictly greater. Ties retain the first visited location. Do not search the unvisited middle of the surface for this operator; it may still be used by subpixel neighbors or display.

Let K=wx*wy, P=fl32(Cmax/fl32(K)) and M=fl32(fl32(sum/fl32(J))/fl32(K)). Set dx=imax if 2imax<wx, otherwise dx=imax-wx; define dy similarly. These comparisons/products are exact integers.

## Confidence and admission

Use the following separately rounded operations, with Ax=fl32(mx+1), Ay=fl32(my+1):

$$T_0=((P-M)100)/(P+0.1),$$
$$d_x=A_x+stab\,|dx|,\qquad d_y=A_y+stab\,|dy|,$$
$$s_x=A_x/d_x,\qquad s_{xy}=s_x A_y,\qquad s=s_{xy}/d_y,\qquad T=T_0s.$$

Preserve this grouping; two independently evaluated axis ratios need not round identically. Every displayed operation is binary32; integer abs values convert before multiplication by stab. Require nonzero divisors and finite results, even when a later confidence test would reject the motion. Negative finite stab is supported and may raise confidence or produce a controlled arithmetic error. Confidence is not clamped to [0,100].

The window is locally valid precisely when T>=t. Equality passes. Failure of this comparison is ordinary unreliable motion, distinct from a data/arithmetic error. A locally invalid window supplies dx'=dy'=+0 and does not run subpixel refinement; retain T for later combination and temporal checks.

## Examples

- For a 4x4 surface zero except C(1,0)=160, mx=my=1,stab=0: J=9,P=10,M=1.1111111640930176,T=88.00879669189453. The peak displacement is (1,0); t=4 passes. With stab=1, T=58.67253112792969.
- If equal highest samples occur at (1,0) and (3,0), the first wins and dx=+1, not -1. A still larger C(2,0) is outside that search and is not a candidate.
- mx=my=0 searches only (0,0); P=M and T=0 when its denominator is nonzero. t=0 admits the window; t=4 rejects it.
- A 4x4 constant surface C=1024 with mx=my=1 has confidence zero because its accumulated sum and normalizations are exact. A general constant binary32 value can accumulate rounding error: the same geometry with C=fl32(0.1),stab=0 has a slightly negative confidence and can fail t=0. Do not replace computed confidence with zero merely because every sample is equal.
- A surface giving P=-fl32(0.1) makes P+0.1 zero and fails, regardless of trust limit. With mx=1,dx=1,stab=-2 the horizontal penalty divisor is zero and fails.
