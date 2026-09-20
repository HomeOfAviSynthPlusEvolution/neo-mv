# Frame-rate and time mapping

Inputs are source frame count Nv, a known positive source frame rate Ri=I/J, user num/den, paired distance d>=1, and an output index n. Outputs are reduced output rate P/Q, output count No, source indices l,r and coefficient t. This operator has no image or vector-array dependency.

## Creation

Require Nv in [1,2147483647] and positive int64 I,J. Reduce Ri exactly. num and den are int64, each nonnegative. Their defaults are independently num=25,den=1. Define

$$R_o=\begin{cases}num/den&num>0\land den>0,\\2R_i&num=0\lor den=0.\end{cases}$$

Reduce Ro to coprime positive P,Q. Reduce Ri/Ro to coprime positive a,b. P,Q,a,b must each fit positive int64. Perform reduction using exact arithmetic; overflowing an unreduced intermediate must not cause rejection if the required reduced values fit. The result's frame count is

$$N_o=\left\lfloor\frac{N_v b}{a}\right\rfloor.$$

Require 1<=No<=2147483647 and host support for the resulting count/rate. Zero-frame results are creation errors, not unspecified-length clips. Do not round up, add an endpoint frame or assume a zero num/den means the default. Frame-rate arithmetic never uses floating point to choose P,Q,a,b,No or l.

## Output position

For 0<=n<No compute exact integers l=floor(n*a/b),r=l+d. Indices use sufficiently wide arithmetic before boundary checks; r may exceed the source frame count or signed32 range. Define binary64 temporaries

$$u=\operatorname{fl64}(\operatorname{fl64}(\operatorname{fl64}(n)\operatorname{fl64}(a))/\operatorname{fl64}(b)),$$
$$e=\operatorname{fl64}(u-\operatorname{fl64}(l)),\qquad t_0=\operatorname{trunc}(\operatorname{fl64}(\operatorname{fl64}(e\,256)+0.5)),$$
$$t=\begin{cases}\operatorname{trunc}(t_0/d)&d>1,\\t_0&d=1.\end{cases}$$

Require finite temporaries and mathematical t0 in [0,256] before integer conversion; a violation is a frame error, not a clamp. All multiplications and additions shown round separately. l remains the exact integer result even when u rounds to an integer. A valid t0=256 is retained. The division by d happens after time quantization. Do not replace this rule with exact rational fractional-time rounding or divide the entire source-frame position by d.

The endpoint selector uses final t: t=0 selects clip[min(l,Nv-1)], t=256 selects clip[min(r,Nv-1)]. Other t values use interpolation/fallback as the plugin specifies. No vector-data or Super evaluation is needed by either endpoint branch. Creation requirements still apply. With the admitted exact count formula l is nonnegative and below Nv; the explicit upper clamp also defines the selected source expression.

## Output duration

Every result frame, including copies and fallback frames, replaces `_DurationNum` with a one-element integer array [Q] and `_DurationDen` with [P]. Replacement applies regardless of previous types, values or element counts. It does not mutate the source property map. Other properties come from the source selected by the plugin.

## Examples

- Ri=24/1,Nv=10,num=0,den=1 gives Ro=48/1,No=20,a/b=1/2. At n=3,l=1,t0=128: d=1 gives r=2,t=128; d=2 gives r=3,t=64. At n=2,t=0 selects clip[1]. Every result has Duration 1/48.
- Omitting num/den for the same source gives 25/1 and No=10. Setting num=60,den=0 still means 48/1.
- Ri=24/1,Ro=30/1,Nv=10 gives a/b=4/5,No=12. At n=1,l=0,t0=205. For d=1,t=205; for d=2,t=102.
- Ri=1/1,Ro=1000/1,Nv=2,d=1,n=999 gives l=0,r=1,t0=t=256 and selects clip[1]. Do not clamp this coefficient to 255. At n=1999 the same endpoint selects clip[1] through the right-index clamp.
- Ri=24/1,Nv=1,num=1,den=1 gives No=0 and fails creation. num=-1 fails even if den=0. Input I=J=9223372036854775807 with num=0 reduces to Ro=2/1; the unreduced doubling need not fit int64 and must not itself cause rejection.
