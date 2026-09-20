# Subpixel phase samples

Inputs: level-0 padded plane B; actual plane dimensions w,h; working dimensions ww>=w,hh>=h; padding hp,vp>=0; sample type; pel in {1,2,4}; sharp in {0,1,2}; and optionally an external pel plane. The padded dimensions are a=ww+2hp,c=hh+2vp. Output: logical phases P(ax,ay), 0<=ax,ay<pel. Each phase coordinate (u,v) represents the sample at (u-hp+ax/pel,v-vp+ay/pel) in original plane coordinates.

P(0,0)=B exactly. pel=1 requires no interpolation. For built-in pel>1 require a,c>=2 for sharp=0, >=4 for sharp=1 and >=6 for sharp=2. These bounds apply per plane, including chroma. Operations act on the full padded plane. Use [R, K, A and D](../README.md); intermediates must retain negative values until explicitly clipped. Output phases and scratch must not overwrite B or phases still needed by the equations.

## Half-sample operator

For a one-dimensional sequence z of length m, the final output at i=m-1 equals z(i). At other positions:

| sharp | Interior value and domain | Remaining positions |
| --- | --- | --- |
| 0 | A(z(i),z(i+1)) for all i<m-1 | None |
| 1 | K(R4(9(z(i)+z(i+1))-z(i-1)-z(i+2))) for 1<=i<m-3 | A(z(i),z(i+1)) |
| 2 | K(R5(z(i-2)+z(i+3)+20(z(i)+z(i+1))-5(z(i-1)+z(i+2)))) for 2<=i<m-4 | A(z(i),z(i+1)) |

H(B) applies this horizontally, V(B) vertically. For sharp=1 float arithmetic, four consecutive taps a,b,c,d use horizontal numerator -(a+d)+9(b+c), vertical numerator ((-a)-d)+9(b+c). For sharp=2 float arithmetic, six taps use a+(f+5(4(c+d)-(b+e))). Round each operation to binary32. K is the identity for float samples.

For sharp=0 the diagonal half phase is J(u,v)=D(B(u,v),B(u+1,v),B(u,v+1),B(u+1,v+1)) at u<a-1,v<c-1, in that argument order. Its last column uses A(B(u,v),B(u,v+1)); its last row uses A(B(u,v),B(u+1,v)); the bottom-right sample copies B. For sharp=1/2, J=H(V(B)), retaining the vertical rounding and clipping before H.

pel=2 phases are P(0,0)=B, P(1,0)=H(B), P(0,1)=V(B), P(1,1)=J.

## Quarter phases

For pel=4 define P00=B, P20=H(B), P02=V(B), P22=J. Subscripts are ax,ay. A +x or +y shifts the indicated phase by one base-plane sample. Each A rounds separately.

| Phase | Equation | Defined domain |
| --- | --- | --- |
| P10 | A(P00,P20) | Entire plane |
| P12 | A(P02,P22) | Entire plane |
| P01 | A(P00,P02) | Entire plane |
| P21 | A(P20,P22) | Entire plane |
| P11 | A(P01,P21) | Entire plane |
| P30 | A(P00(+x),P20) | u<a-1 |
| P32 | A(P02(+x),P22) | u<a-1 |
| P31 | A(P01(+x),P21) | u<a-1 |
| P03 | A(P00(+y),P02) | v<c-1 |
| P23 | A(P20(+y),P22) | v<c-1 |
| P13 | A(P03,P23) | v<c-1 |
| P33 | A(P03(+x),P23) | u<a-1 and v<c-1 |

All domains also require 0<=u<a,0<=v<c. Locations outside these domains are not logical quarter-phase outputs and must not be used as semantic inputs. Phase-1 block samplers must enforce these domains for every addressed sample, including chroma. Unused allocation contents are not output values. Do not compute a defined sample by reading an undefined intermediate.

Analyse and Recalculate apply the [whole-domain sampling precondition](../analyse/kernel-block-error.md#sampling-admissibility-and-geometry-errors) before motion evaluation. If any required candidate or admitted seed footprint reaches an undefined edge, the consuming geometry is rejected; Super does not extend that edge to make it admissible. A successful Super output need only provide the logical domains defined here and does not certify every possible consumer geometry.

## External pel samples

For pel>1 an external plane E must have actual dimensions pel*w by pel*h, where w,h are the source plane's actual dimensions. For nonzero phases,

$$P_{ax,ay}(x+hp,y+vp)=E(pel\,x+ax,pel\,y+ay),\quad 0\le x<w,\ 0\le y<h.$$

Extend each sampled actual phase to its working/padded extent by [nearest-edge extension](kernel-border-extension.md). Thus external phases are defined over their whole extent. P00 still comes from the original source, never E. `sharp` has no effect on external phases. Reduced pyramid levels still use the original integer phase.

## Examples

- At an interior midpoint between 20 and 40 with consecutive taps [0,10,20,40,80,100], sharp 0/1/2 gives integer values 30,28,27; float values 30,28.125,26.5625.
- A sharp=0 base patch [10,14;18,22] gives pel=2 phases 10,12,14,16 at its top-left. For pel=4, P10=11, P01=12, P21=14 and P11=13.
- With source value 10 and external pel=2 cell [99,14;18,22], output phases are 10,14,18,22. The external 99 does not replace the source.

Consumers: Super and block-error sampling. No physical phase packing or SIMD over-read is part of this operator.
