# Basic motion from one or two windows

Inputs are one or two locally decoded window results (dx',dy',local-valid,T), finite zoommax, the two rectangle origins when applicable, and basic observation index n. Output is a basic tuple (dx,dy,z,good,T). Rotation is not estimated. This operator uses the [common binary32 arithmetic](../README.md#common-contracts).

For zoommax=1, copy the first window's dx',dy',validity and T, and set z=1. For zoommax!=1, let D=left2-left>0 and compute

$$z_* = 1+(dx'_2-dx'_1)/\operatorname{fl32}(D).$$

The output is good only if both windows are locally valid and abs(z*-1)<fl32(zoommax-1), strictly. If good, set dx=(dx'1+dx'2)/2,dy=(dy'1+dy'2)/2,z=z*. Otherwise set dx=dy=+0,z=1,good=false. T is min(T1,T2) in either case; at a numeric tie retain T1, including its zero sign. Compute z* before applying the validity tests; averaging is evaluated only on success. Do not estimate rotation from vertical disagreement. Window dy' values have already undergone field correction and aspect division; do not repeat either transformation.

Finite zoommax below 1 still enables two-window geometry but can never satisfy the strict scale test. It is not an alias for disabled zoom. Do not add a positive-scale admission test or apply the motion consumer's magnitude clamps to producer output. In particular, at very large dimensions binary32 displacement conversion can make z*=0 even when exact-integer peak bounds alone would suggest a positive result; the displayed comparison still determines acceptance.

After computing the above result, n=0 forces good=false and T=+0. Its dx,dy,z are irrelevant to final output because [temporal validity](kernel-temporal-validity.md) resets invalid motion. Correlation, local decoding, required parity and their errors are not bypassed on frame zero. No synthetic perfect-confidence result is substituted for its autocorrelation.

## Examples

- Two locally valid windows with dx'1=-1,dx'2=1,dy'1=dy'2=0,D=100 produce z*=1.0199999809265137. zoommax converted from 1.02 equals z*, so the strict test fails. zoommax=1.1 admits it, with dx=dy=0. If T1=80,T2=60, resulting T is 60 in both cases.
- A locally invalid second window resets the combined motion even when the first is strong; the confidence is still the smaller T, not automatically zero.
- zoommax=0.9 uses two windows and produces bad motion even for identical windows. zoommax=1 uses one window and can accept zero motion.
- Frame zero always has basic confidence 0 and good=false, including a highly textured identical-image pair.
