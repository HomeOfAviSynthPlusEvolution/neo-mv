# Resampling block grids into per-pixel values

The three masks expand a small intensity grid into an image; Flow functions expand displacement grids into dense fields. They share coordinates but differ in integer/floating arithmetic.

## 1. Coordinates use complete block coverage

For target-plane blocks Bx, By, overlap Ox, Oy and grid Nx, Ny, coverage is `WB=Nx(Bx-Ox)+Ox,HB=Ny(By-Oy)+Oy`. Visible dimensions cannot exceed it. Cropping right/bottom does not stretch the remaining grid again.

For pixel x:

$$D_x=2W_B,\quad U_x=(2x+1)N_x-W_B,\quad i=\lfloor U_x/D_x\rfloor,\quad R_x=U_x-iD_x.$$

This is continuous grid coordinate `(x+0.5)Nx/WB-0.5`. Y similarly gives j, Ry. Clamp four neighbor indices independently to grid edges while retaining original interpolation fractions. This extends a small grid; it does not authorize out-of-bounds Super sampling.

## 2. Integers: rounding on each axis

Set Q=16384. Round exact fractions `Q·Rx/Dx,Q·Ry/Dy` to nearest-even integers a, b. Complementary coefficients are exactly Q−a, Q−b, not independently rounded.

One-dimensional interpolation is:

$$L(s_0,s_1,k)=\left\lfloor\frac{(Q-k)s_0+ks_1+Q/2}{Q}\right\rfloor.$$

Coefficients use nearest-even, but sample half-ties round toward positive infinity, including negative displacement values.

Use horizontal then vertical if `2WB·Ny<HB·Nx+WB·HB`; otherwise vertical then horizontal, including equality. The first axis's integer result feeds the second. The choice uses full coverage, not cropped dimensions.

For grid `[[0,1],[0,3]]`, nonoverlapping `3×3` blocks and `6×6` output, pixel (2, 2) has fractions 1/3 and coefficients 5461. Horizontal results 0, 1 then vertical give 0. Rounding the exact 2D value 5/9 once would give 1 instead.

## 3. Float32: horizontal then vertical

Use binary64 fractions a=Rx/Dx, b=Ry/Dy. Compute both rows `(1-a)Gleft+aGright`, then `(1-b)h0+bh1`, rounding every binary64 operation and finally converting once to binary32. There are no Q14 coefficients, axis-order choice, or range remapping.

## 4. A one-dimensional example

Grid `[0,100]`, two nonoverlapping width-4 blocks, width-8 output gives integer row `[0,0,13,38,63,88,100,100]`. Visible width 6 retains the first six entries.

Float32 grid `[0,1]` gives `[0,0,0.125,0.375,0.625,0.875,1,1]`. Signed integer grid `[0,4]` gives `[0,0,1,2,3,4,4,4]`.

Masks quantize grid values before resampling; expanding unquantized scores first changes the calculation. Geometry products use sufficiently wide exact integers. Large coverage does not require an equally large intermediate allocation.

[Back to the English index](../README.md)
