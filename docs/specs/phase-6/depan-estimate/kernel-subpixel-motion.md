# Subpixel peak motion, fields and aspect

Inputs are a locally valid peak from [peak confidence](kernel-peak-confidence.md), the same finite surface, positive binary32 pixaspect a, fields and the current observation's field parity. Outputs are binary32 dx',dy'. A locally invalid window instead yields (0,0), with no refinement arithmetic. The plugin still validates parity for every required basic observation when fields=true, before this local branch.

For the horizontal axis, let c0=C(imax,jmax), cp=C((imax+1) mod wx,jmax), cm=C((imax-1) mod wx,jmax), using nonnegative periodic residues. In binary32 define

$$f_1=(c_p-c_m)/2,\qquad f_2=(c_p+c_m)-2c_0,$$
$$a_x=\begin{cases}+0&f_2=0,\\\min(1,\max(-1,-f_1/f_2))&f_2\ne0.\end{cases}$$

If abs(fl32(fl32(dx)+ax))>fl32(mx), replace ax by +0. Do not clamp the fractional result to the search boundary. For the vertical axis use C(imax,(jmax+1) mod wy) and C(imax,(jmax-1) mod wy), the same equations and my to obtain ay. Neighbors come from the whole periodic surface, even when they are outside the peak-search region. f2 need not be negative; the displayed formula and clamp still apply. Non-finite required arithmetic is a frame error.

Without fields, set dx'=fl32(fl32(dx)+ax) and dy'=fl32(fl32(fl32(dy)+ay)/a).

With fields, first set ay=fl32(ay+(top?0.5:-0.5)), then ay=fl32(ay*2), and replace integer dy by 2dy exactly. Finally use the same dx'/dy' equations. The horizontal displacement is unchanged. The parity is that of the basic observation's current source frame, using [the shared parity contract](../../phase-5/depan-analyse/kernel-motion-properties.md#field-parity). It is not determined by the window's top coordinate. Do not halve pixaspect in this kernel or apply the dymax bound again after field correction.

## Examples

- Peak c0=10 with left cm=6,right cp=8 gives f1=1,f2=-6 and ax=0.1666666716337204. At dx=0,mx=1 it is retained. At dx=1,mx=1 it is discarded, producing dx'=1, rather than a clipped fractional value.
- At the same positive search boundary, left=8,right=6 gives negative ax and moves inward, so the refinement remains admitted.
- A peak at imax=0 reads its left neighbor at wx-1. At wx=2 the plus/minus horizontal neighbor is the same sample; f1=0.
- With dx=dy=ax=ay=0,a=1, fields=false produces (0,0); top field produces (0,1), bottom produces (0,-1). At a=2 those vertical outputs are +0.5 and -0.5.
- If integer dy=1,ay=0.25,my>=2,fields=true,top=false,a=2: adjusted ay=(0.25-0.5)*2=-0.5, integer dy becomes 2, and dy'=0.75.
