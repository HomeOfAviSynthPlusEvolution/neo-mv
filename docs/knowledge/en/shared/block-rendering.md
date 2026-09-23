# Block rendering: coordinates, overlap windows, and visible output

[Compensate](../compensate.md) selects one image source per block; [Degrain](../degrain.md) generates a weighted temporal sample per block. Both then use this block-composition process.

## 1. Matching image and vector geometry

Clip and Super must share real dimensions, sample type/depth, color family, chroma ratios, and positive frame count. Output rate comes from clip; auxiliary nodes need not share it. Vector carrier pixel format is irrelevant, but required frames must exist.

Analysis working/real dimensions, padding, and pel must match Super. Coverage requires `Wr≤Nx(Bx-Ox)+Ox≤W` and the vertical equivalent; uncovered visible areas cannot simply be filled black. Chroma processing requires blocks/overlap aligned to actual rendering ratios. Analysis precision may differ from rendering precision; thresholds still use analysis precision.

Current valid scalar metadata must match creation values except that positive Levels may vary. Complete malformed data fail before temporal bounds can hide them. A reference is available only after scene checks and if `n+d` exists; indices are not clamped. See [data states](analysis-data.md) and [scene thresholds](../sc-detection.md).

## 2. From vectors to block sampling coordinates

Luma block origin is x, y; displacement dx, dy is in luma pel units. For rendering-plane ratios rx, ry, floor the total coordinates:

$$a_x=\left\lfloor\frac{px+d_x}{r_x}\right\rfloor,\quad a_y=\left\lfloor\frac{py+d_y}{r_y}\right\rfloor.$$

Split `qx=floor(ax/p),αx=ax-pqx`, and similarly for Y. Read local sample i, j from Super phase `(αx,αy)` at `(hP+qx+i,vP+qy+j)`.

For `p=2,x=0,dx=-1,rx=2`, `ax=-1,qx=-1,αx=1`: half a chroma pixel left. This differs from Analyse's error sampling, which first truncates chroma motion components toward zero.

The entire generated block must lie in defined phase support, even portions later cropped away. Degrain validates the complete public vector domain and center displacement at creation. Compensate validates the time-scaled domain and allowed current-field shifts, then checks actual shifted reference footprints per frame. Zero weights or poor SAD do not waive geometry checks.

## 3. Overlap windows

For a plane's block length B and overlap O, an interior block uses:

$$w_m(i)=\begin{cases}
\cos^2\frac{\pi(i-O+1/2)}{2O}&i<O\\
1&O\le i<B-O\\
\cos^2\frac{\pi(i-B+O+1/2)}{2O}&i\ge B-O.
\end{cases}$$

Without overlap, all weights are 1. Replace the first block's left shoulder and last block's right shoulder by 1. A single block gets both replacements, making its whole window 1. This avoids attenuation at edges with no neighboring block.

Angles use binary32 π with separate multiplication/division rounding. Convert cos to binary32 before squaring. Multiply axis windows before quantizing:

$$W(i,j)=trunc(fl32(fl32(fl32(w_y(j)w_x(i))\,2048)+0.5)).$$

Do not quantize axis windows separately. `B=4,O=1` gives an interior axis approximately `[0.5,1,1,0.5]`; a single shoulder gives 2D weight 1024, intersecting shoulders 512.

## 4. Composing blocks

Without overlap, copy the uniquely covering block and crop right/bottom to visible dimensions. With overlap, visit contributions in block-row order. For integer samples:

$$a_k=\lfloor q_kW_k/64\rfloor,\quad
out=clip\left(\left\lfloor\frac{\sum a_k+16}{32}\right\rfloor,0,2^b-1\right).$$

Floor each contribution divided by 64 before summing and dividing by 32. Do not divide by the actual weight sum: the fixed scale is 2048, preserving window quantization effects.

Float32 starts at z=+0 and accumulates `fl32(fl32(qk·Wk)/64)` in block-row order, rounding each addition, then returns `fl32(z/32)`. No +16 bias or `[0,1]` clipping.

Two weights 1024 with block values 10, 21 give integer contributions 160, 336 and output `floor((496+16)/32)=16`; float32 gives 15.5. Degrain's temporal rounding has already happened inside each block and cannot be merged with this stage.

With no overlap, block width 8, three blocks, visible width 18, and working width 32, output 8, 8, 2 columns from the blocks. Extra working width does not move crop origins. Working samples outside grid coverage and outside the visible image are not output.

## 5. Shared threshold scale

Analysis area, chroma participation, and precision define:

$$F=\left(\frac{B_xB_y}{64}\,C_f\right)\frac{2^{\min(16,b_a)}-1}{255},\quad
C_f=\begin{cases}1+2/(r_xr_y)&\text{analysis includes chroma}\\1&\text{otherwise}\end{cases}.$$

Scale nonnegative z as `S(z)=trunc(fl64(fl64(fl64(z)F)+0.5))`, computing the factor in the shown binary64 order. For 10-bit, luma-only `8×8` analysis, `S(400)=1605`, even when the rendered video is 8-bit.

[Back to the English index](../README.md)
