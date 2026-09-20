# Per-plane thresholds and reference weights

Inputs: number of temporal pairs R in [1,25], near/far nonnegative int64 threshold pairs for luma/chroma, common analysis descriptor, per-reference availability and block SAD values, and 2R+1 user coefficients. Output: one threshold per pair/plane and nonnegative integer weights Vc,V0,...,V(2R-1) summing exactly to 256 for each block and processed plane. No image samples are read.

## Threshold scaling and pair position

Scale each near/far luma/chroma argument with [S](../compensate/kernel-reference-availability.md#thresholds). Each result must be in [0,2147483646]; reject larger values at creation, including an unused far threshold when R=1. U and V use the same chroma threshold. Analysis precision and AnalysisChroma determine S, independently of the rendered plane and video precision.

For pair index j=1,...,R, Tnear and Tfar are the scaled values for this plane. If R=1 or j=1, Tj=Tnear. Otherwise use independently rounded binary64 operations:

$$\theta_j=\operatorname{fl64}(\operatorname{fl64}((j-1)\pi_{64})/(R-1)),\quad
a_j=\operatorname{fl64}(\operatorname{fl64}(1-\operatorname{fl64}(\cos\theta_j))/2),$$
$$T_j=\left\lfloor\operatorname{fl64}(\operatorname{fl64}(T_{near}+\operatorname{fl64}(a_j(T_{far}-T_{near})))+0.5)\right\rfloor.$$

pi64 is the nearest binary64 value to pi. Convert exact integer differences to binary64 before multiplication. The [cosine tolerance](../README.md#common-contracts) applies. This is interpolation by pair index, not by the absolute frame distance. The two members of a pair use the same threshold even if their user coefficients differ. Tfar may be less than Tnear.

## Raw reliability

For an available reference with block SAD s>=0 and this plane's threshold T, define

$$w(s,T)=\begin{cases}0&s\ge T,\\
\operatorname{trunc}\left(\frac{256(1-(s/T)^2)}{1+(s/T)^2}\right)&s<T.
\end{cases}$$

For the second branch compute r=fl64(fl64(s)/fl64(T)), z=fl64(r*r), numerator=fl64(256*fl64(1-z)), denominator=fl64(1+z), then truncate fl64(numerator/denominator). T=0 always selects the first branch without division. An unavailable reference has w=0. For all processed planes use the field's stored total block SAD; there is no separate chroma SAD array.

## User coefficient mapping and normalization

Let the vectors input order be [a1,b1,a2,b2,...,aR,bR]. User weights have order [aR,...,a1,centre,b1,...,bR], independent of the signs of each pair's deltas. With zero-based input array U, define

$$u_c=U[R],\quad u_{a_j}=U[R-j],\quad u_{b_j}=U[R+j].$$

Each coefficient is an integer in [0,floor(2147483646/(256(2R+1)))]. All-ones is the default. Form exact integer products and sum

$$Z=256u_c+1+\sum_{r=0}^{2R-1}w_ru_r,\quad g=\operatorname{fl64}(256/Z),$$
$$V_r=\operatorname{trunc}(\operatorname{fl64}(\operatorname{fl64}(w_ru_r)g)),\qquad
V_c=256-\sum_{r=0}^{2R-1}V_r.$$

Compute the division as binary64, with Z converted to that precision. The +1 is part of the definition. Vc receives the remaining weight; it is not an independently normalized centre coefficient. Bounds ensure Z is positive. No division by zero, negative final weights or modulo arithmetic is permitted. Inputs remain immutable and each block/plane result is independent of other blocks.

## Examples

- T=100: s=0,50,100 give w=256,153,0. T=0 gives w=0 even at s=0.
- R=1, both references available with w=256, all user coefficients 1: Z=769, V0=V1=85,Vc=86.
- R=1, first reference w=256 and second unavailable, all coefficients 1: Z=513,V0=127,V1=0,Vc=129.
- All user coefficients zero give Z=1 and Vc=256, all reference weights zero. With centre coefficient zero and both reference coefficients 1 at w=256, the references receive 127 each and Vc=2.
- R=3, near=100, far=300 after scaling gives thresholds 100,200,300 for pairs at distances 1,4,10 just as for distances 1,2,3.
- R=2, U=[10,20,30,40,50] gives uc=30, ua1=20, ub1=40, ua2=10, ub2=50.
