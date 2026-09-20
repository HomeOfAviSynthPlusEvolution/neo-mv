# Pyramid reduction

Inputs: the previous level's padded integer-phase plane S, expressed in working coordinates; target working size w,h; rfilter=0,1,2. Output: target working samples T, followed by [border extension](kernel-border-extension.md). S(0,0) denotes the working image origin, not its padding origin.

Use R, A and D from the [common numerical contract](../README.md). Target dimensions come from [geometry](kernel-geometry.md). Every addressed S coordinate must be defined in the previous level's working image or padding. Output and intermediate planes must not alias S.

## rfilter=0

$$T(x,y)=D(S(2x,2y),S(2x+1,2y),S(2x+1,2y+1),S(2x,2y+1)).$$

There is one rounding operation for integer samples; two successive rounded averages are not equivalent.

## rfilter=1

First define sample-precision V for 0<=x<2w, 0<=y<h. For y=0 or y=h-1,

$$V(x,y)=A(S(x,2y),S(x,2y+1)).$$

For all other rows,

$$V(x,y)=R_3((S(x,2y-1)+3(S(x,2y)+S(x,2y+1)))+S(x,2y+2)).$$

For x=0 or x=w-1,

$$T(x,y)=A(V(2x,y),V(2x+1,y));$$

otherwise,

$$T(x,y)=R_3((V(2x-1,y)+3(V(2x,y)+V(2x+1,y)))+V(2x+2,y)).$$

V is rounded to sample precision before horizontal filtering. A single row/column is both an initial and final boundary and uses the two-point rule.

## rfilter=2

Use the same two-point boundary rules. At interior positions replace the four-tap expression with

$$C_6(a,b,c,d,e,f)=R_5(a+((f+10(c+d))+5(b+e))).$$

The six inputs have offsets -2,-1,0,1,2,3 from index 2y or 2x. Apply vertically to S, round/store V, then horizontally to V. All coefficients are positive; do not apply the signed subpixel interpolation coefficients here. No clipping is required on valid integer input.

The logical intermediate extent is 2w by h, regardless of whether an implementation stores it. Float operations use the indicated grouping and binary32 rounding; R becomes unbiased division by its power of two.

## Examples

- A 2x2 patch [0,0;0,1] gives 0 for rfilter=0. Averaging rows and then those results would incorrectly give 1.
- In a 32x32 GRAY8 plane with only S(4,4)=10, output (2,2) is 2 for rfilter=1: V(4,2)=R3(30)=4, then R3(12)=2. For rfilter=2 it is 1: V(4,2)=R5(100)=3, then R5(30)=1.
- The same impulse as float32 produces 1.40625 and 0.9765625 respectively.

Consumer: Super levels above zero. The result is independent of internal row buffering or parallel partitioning.
