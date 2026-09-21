# Correlation-surface display

Inputs are a copy of clip[n], its current one or two complete correlation surfaces, and their [rectangles](kernel-window-geometry.md). Output replaces only the first-plane samples inside those rectangles. This operator is used only for show=true. It displays the periodic surface directly, without centering its zero shift or cropping to the peak-search domain. Other planes and samples outside the rectangles retain their stored bits.

For each surface independently, find cmin and cmax over all wx*wy finite samples. Initialize both from C(0,0), scan row-major and replace only on strict smaller/larger comparisons. With M=2^b-1 for an integer clip, or M=1 for float32, compute in binary32

$$d=c_{max}-c_{min},\qquad norm=\operatorname{fl32}(M)/d,\qquad q(i,j)=(C(i,j)-c_{min})norm.$$

Require d>0 and finite norm and q. A constant surface when show=true is a controlled frame error, including an all-zero surface; no arbitrary black display is substituted. show=false does not perform these operations and may successfully return bad motion for the same input.

For integer output form the mathematical integer J=trunc(q), require J in [0,M], then store J. The range check is on J, not on the untruncated q; do not narrow to a machine integer before checking representability. No nearest rounding or extra clamp is applied. For float32 store q, with no additional [0,1] clamp. A representable last-bit excursion is retained. Write output (left+i,top+j); use left2 for the second surface. Admitted window geometry makes the two rectangles disjoint. Each surface uses its own min/max, even when motion was rejected or frame n=0 is forced bad. Display does not change the confidence or motion values, and never feeds subsequent observations.

## Examples

- A 4x4 surface zero except C(1,0)=160 displays its peak as M and every other sample as 0. If left=2,top=1, the bright output sample is at (3,1), not at the center of the rectangle.
- For two surfaces whose min/max pairs are (0,10) and (100,200), each independently maps its own extrema to approximately 0 and M. Do not normalize them jointly.
- On integer8, a surface with minimum 0,maximum 10 and a sample 1 produces norm=25.5,q=25.5 and stored sample 25. On float32 the same sample is approximately 0.1.
- On integer8 with cmin=0,cmax=336, the maximum sample produces q=255.00001525878906 under binary32 normalization. J=255 passes the integer range check and is stored. Rejecting the untruncated q would incorrectly fail this supported display.
- A constant first-plane image may have an admissible zero-motion result at trust=0 with show=false, but its exactly constant correlation surface fails this display operator. Chroma is unchanged even when show=true.
