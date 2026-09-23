# FlowFPS: map the output timeline before bidirectional interpolation

## 1. What the function computes

`FlowFPS` converts each output index to a source position, then either copies an endpoint or performs [bidirectional interpolation](shared/bidirectional-interpolation.md). It changes rate, frame count, and frame duration.

## 2. Objects and notation

Source rate Ri=I/J, count Nv; reduced target rate P/Q. Reduced source/target rate ratio is a/b. Pair `[bw,fw]` has distance d>0. Output n determines left l, right r, time t.

## 3. Overall calculation

Reduce rates and derive output count at creation. Per frame, compute exact l and quantize fractional time; copy endpoints or choose interpolation/original-image fallback. Every successful branch replaces duration properties.

## 4. Step-by-step calculation

### 4.1 Rate and count

num, den independently default to 25, 1 and must be nonnegative. When both are positive, target is num/den; if either is zero, target is 2Ri, not the default rate.

Reduce P/Q and a/b using exact integers; reduced components must fit positive int64. Large unreduced products alone must not cause failure. Count is:

$$N_o=\lfloor N_vb/a\rfloor.$$

Do not round upward or append another endpoint. Result must be 1–2147483647.

### 4.2 Source position and time coefficient

Compute l=floor(n·a/b) exactly, r=l+d. Quantize using these binary64 steps:

$$u=fl64(fl64(fl64(n)fl64(a))/fl64(b)),\quad e=fl64(u-fl64(l)),$$
$$t_0=trunc(fl64(fl64(e\cdot256)+0.5)),\quad
t=\begin{cases}trunc(t_0/d)&d>1\\t_0&d=1.\end{cases}$$

Floating rounding of u does not alter l. t0 must be 0–256, without clamping invalid values; legal 256 remains. Division by d follows quantization rather than dividing the full source position or using a different rational rounding rule.

### 4.3 Branch order

t=0 copies clip[min(l, Nv−1)]; t=256 copies clip[min(r, Nv−1)]. Endpoints do not read current vector arrays or Super; blend/extramask do not change that choice, although creation geometry checks still apply.

Other t selects main bw[l], fw[r], and tries extras only when extramask=true. Available fields use shared interpolation. Out-of-range/unavailable main pairs blend clamped original images when blend=true, otherwise copy the left image.

Nonendpoint properties come from left clip. Every branch replaces `_DurationNum=[Q],_DurationDen=[P]`; other properties, including scene flags, remain.

## 5. A complete numerical example

Three source frames at 24fps, num=48, den=1, d=1 give a/b=1/2, No=6.

n=1 gives l=0, r=1, u=0.5, t=128. Available zero-motion main fields, missing extras, and constant Super values 10, 21 give integer output 15. Properties come from clip[0], with duration 1/48.

n=2 gives l=1, t=0 and directly copies clip[1], without reading different Super pixels or malformed current vector arrays. n=5 gives l=2, r=3, t=128: right is out of range, so fallback copies/blends the final original frame. All cases have duration 1/48.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,super,vectors` | Required; match original Nv | Timeline, samples, pair |
| `num,den` | 25, 1; nonnegative int64 | Target rate; either zero doubles source rate |
| `extramask` | true | Extra fields |
| `ml` | 100.0; finite positive binary32 | Occlusion strength |
| `blend` | true | Nonendpoint fallback |
| `thscd1,thscd2` | 400, 51.0 | Scene decision |
| `prefix` | `MVUtensils` | Data names |

## 7. Boundaries, missing data, and errors

Source rate must be known and positive. Zero output count is a creation error. num=−1 remains invalid even with den=0. Auxiliary lengths match source Nv, not output No. extramask=false removes extra-field dependencies entirely. Unlike FlowInter, endpoint branches perform no interpolation validation.

## 8. Precision and determinism

Rate reduction, count, and l use exact integers; fractional time uses specified binary64 operations. Source 24fps, target 30fps, n=1 gives t0=205; d=2 then truncates to 102. Source 1fps, target 1000fps, n=999 gives t=256 and must copy the right endpoint, not clamp to 255.

[Back to the English index](README.md)
