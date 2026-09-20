# Mirror and horizontal border sampling

Inputs are an integer plane P of size W,H, integer coordinates i,j, mirror in [0,15], nonnegative integer blur limit b, and a border sample B in [0,M]. Output is a sample. Each sampler below specifies where it calls these rules. Mirror bits are top=1, bottom=2, left=4, right=8. Coordinates and sums use exact integers; source reads are confined to visible samples.

## One-pass reflection

For a coordinate q and axis length D, first replace q by -q if q<0 and the low-side bit is enabled. Then replace the resulting q by 2D-q-2 if q>=D and the high-side bit is enabled. Apply each condition once, in that order. The result may still be outside [0,D); this is not periodic reflection. General two-axis reflection applies the vertical rule and horizontal rule independently, then reads P only if both resulting coordinates are valid; otherwise output B. No blur or final coordinate clamp is applied by this general rule.

## Horizontal edge fill on a valid row

This rule is called only when 0<=j<H and the sampler has already excluded its directly sampled/interpolated columns. Let C(k)=min(max(k,0),W-1).

For i<0 with left enabled:

- b=0: output P(C(-i),j).
- b>0: let ell=min(b,-i). Output floor of the sum of P(C(k),j) over k=-i-ell+1 through -i, divided by ell.

For the right edge with right enabled, use a threshold R and parameter delta supplied by the caller. Require i>=R. Let q=2W-i-2.

- b=0: output P(C(q),j).
- b>0: let ell=min(b,i-W+delta). Output floor of the sum over k=q through q+ell-1 of P(C(k),j), divided by ell.

Ordinary horizontal fill uses R=W,delta=1. Bilinear interior-row fill uses R=W-1,delta=2. These calls guarantee ell>=1 when b>0. A disabled relevant side, or no applicable side, yields B. A request for single-sample horizontal fill means ordinary fill with b=0 regardless of the user's blur value. Each column is clamped separately before reading; do not shorten the count at a border. Blur is a spatial average of reflected columns, not temporal blur.

Phase 5 always supplies a concrete B. This interface has no negative-border sentinel or permission to leave output memory unchanged.

## Examples

- For W=5 and row [0,10,20,30,40], i=-2,left enabled gives 20 at b=0 and 15 at b=2.
- At i=5,right enabled,b=2, ordinary fill has ell=1 and returns 30. Bilinear fill has ell=2 and returns 35.
- At i=W-1=4, bilinear right fill with b=0 returns column 4. With the right bit disabled, it returns B, even though column 4 exists.
- With H=5,j=-10 and both vertical bits enabled: -10 becomes 10, then -2. It remains out of range and yields B. A horizontal edge-fill call may instead clamp its reflected column; the two operations are intentionally different.
