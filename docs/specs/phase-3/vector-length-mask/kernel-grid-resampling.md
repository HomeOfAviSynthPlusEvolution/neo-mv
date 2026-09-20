# Block grid to visible sample grid

Inputs are a row-major Nx by Ny grid G, block dimensions Bx,By, overlaps Ox,Oy and visible output dimensions Wr,Hr, all in the destination plane's pixel units. Output is Wr by Hr samples. Nx,Ny,Bx,By,Wr,Hr are positive; 0<=Ox<Bx and 0<=Oy<By. Define

$$W_B=N_x(B_x-O_x)+O_x,\qquad H_B=N_y(B_y-O_y)+O_y.$$

Require Wr<=WB and Hr<=HB. Do not stretch the grid to the cropped visible dimensions. This operator has no knowledge of pel, reference frames, SAD or properties. Plane-specific calls supply their own B, O and visible dimensions with the same block counts.

## Coordinates and edge extension

For each visible integer coordinate 0<=x<Wr, 0<=y<Hr, define exact integer quantities

$$D_x=2W_B,\quad U_x=(2x+1)N_x-W_B,\quad i=\lfloor U_x/D_x\rfloor,\quad R_x=U_x-iD_x,$$
$$D_y=2H_B,\quad U_y=(2y+1)N_y-H_B,\quad j=\lfloor U_y/D_y\rfloor,\quad R_y=U_y-jD_y.$$

Thus 0<=Rx<Dx and 0<=Ry<Dy. These are the center-aligned coordinates u=(x+0.5)Nx/WB-0.5 and v=(y+0.5)Ny/HB-0.5. For each of the four neighbors independently clamp its integer column to [0,Nx-1] and row to [0,Ny-1]; retain the original weights. Denote these samples G00,G10,G01,G11. This edge extension applies to the small grid only, never to a Super sampling domain.

## Integer samples

Unsigned mask samples and signed displacement samples use two one-dimensional passes. Both coefficient quantization and the integer result of the first pass are part of the output contract. Let Q=16384 and define

$$a=\operatorname{RN_{even}}(Q R_x/D_x),\qquad b=\operatorname{RN_{even}}(Q R_y/D_y),$$

where RN_even rounds the exact rational value to the nearest integer, choosing the even integer at a tie. Thus a,b are in [0,Q]. The complementary coefficients are exactly Q-a and Q-b; do not round them independently. Define an axis interpolation operator

$$L(s_0,s_1,k)=\left\lfloor\frac{(Q-k)s_0+k s_1+Q/2}{Q}\right\rfloor.$$

Each axis therefore rounds its sample result with halfway results toward positive infinity, including negative values. Coefficient rounding and sample rounding are different rules. Do not replace the quantized coefficients with exact spatial fractions, and do not postpone both sample roundings until the final two-dimensional result.

Choose the order once for the destination plane's supplied geometry. Use horizontal then vertical precisely when

$$2W_BN_y < H_BN_x+W_BH_B.$$

Otherwise use vertical then horizontal, including equality. This comparison uses exact integers and full covered dimensions WB,HB; it does not use cropped Wr,Hr. The integer block geometry in this operator guarantees WB>=Nx and HB>=Ny. The results are

$$G'(x,y)=\begin{cases}
L(L(G_{00},G_{10},a),L(G_{01},G_{11},a),b)&\text{horizontal first},\\
L(L(G_{00},G_{01},b),L(G_{10},G_{11},b),a)&\text{vertical first}.
\end{cases}$$

The same rule applies to each component of a displacement field and to an integer mask. If an axis is an identity map, evaluating or omitting its identity interpolation gives the same value. Tiling, SIMD batching and request order must not change coefficients, axis order or intermediate rounding.

The supported integer inputs here are mask samples in [0,2^ba-1] for ba=8..16, and signed displacement components in [-32768,32767]. Both produce results in the same respective range: each axis has nonnegative coefficients summing to Q, so no added clamp is needed. Integer masks arrive already quantized by their mask kernel. Resizing the unquantized mask scores would produce different pixels. Use sufficiently wide exact integer arithmetic or an equivalent exact rearrangement for geometry, quantization and axis operations; overflow of a chosen 64-bit intermediate is not a valid reason to reject otherwise representable geometry and storage. No WB by HB temporary allocation is required.

## Float samples

For finite binary32 samples use a=fl64(Rx/Dx), b=fl64(Ry/Dy), c=fl64(1-a), e=fl64(1-b), with integer operands converted to binary64 for these divisions. Define

$$h_0=\operatorname{fl64}(\operatorname{fl64}(cG_{00})+\operatorname{fl64}(aG_{10})),\quad h_1=\operatorname{fl64}(\operatorname{fl64}(cG_{01})+\operatorname{fl64}(aG_{11})),$$
$$G'(x,y)=\operatorname{fl32}(\operatorname{fl64}(\operatorname{fl64}(eh_0)+\operatorname{fl64}(bh_1))).$$

Require finite intermediates and result. No range remapping is performed. This float mode always uses the displayed horizontal-then-vertical binary64 arithmetic and a single final binary32 conversion. The integer mode's coefficient quantization, integer intermediate rounding and axis-order selection do not apply to this float mode.

## Examples and failures

- Nx=2,Ny=1,Bx=By=4,Ox=Oy=0,Wr=8,Hr=4, each row of the unsigned grid is [0,100]. Every output row is [0,0,13,38,63,88,100,100]. With Wr=6, keep its first six entries; do not compute an alternative six-pixel stretch.
- The same geometry with signed grid [-1,0] produces [-1,-1,-1,-1,0,0,0,0]. A grid [-1,1] at Nx=2,Bx=3,Wr=6 has x=1 weight 0 and x=2 weight 1/3: results -1 and 0 respectively.
- Nx=Ny=2, Bx=By=3, overlap zero, G=[[0,1],[0,3]], Wr=Hr=6. At x=y=2 both exact fractions are 1/3, so a=b=5461. The geometry selects horizontal first. Its rounded row values are 0 and 1, and the final value is 0. One final rounding of the unquantized two-dimensional value 5/9 would instead give 1 and is incorrect.
- Nx=2,Ny=1,Bx=3,By=4,overlap zero,Wr=6,Hr=4,G=[0,65535] gives each integer row [0,0,21844,43691,65535,65535]. Exact unquantized interpolation would instead give 21845 and 43690 at the two interior positions. Fourteen-bit coefficient quantization is observable even when only one axis varies.
- Nx=Ny=2,Bx=16,By=2,Ox=0,Oy=1,Wr=32,Hr=3 has WB=32,HB=3. Since 128<102 is false, use vertical first. For G=[[0,1170],[2340,8191]], at x=9,y=1 the coefficients are a=1536,b=8192 and the result is 1499. Horizontal first would give 1500. A library's arbitrary pass order is not interchangeable.
- A float grid [0,1] with the first geometry produces [0,0,0.125,0.375,0.625,0.875,1,1]. A constant grid remains constant, including Nx=Ny=1.
- Nx=Ny=1,Bx=By=2147483647,Ox=Oy=0,Wr=Hr=1,G=[255] produces one sample 255. Its covered dimensions do not require a correspondingly sized intermediate image. It is a supported call, not an allocation-size fallback.
- Wr=9 with WB=8 is an error. Invalid views, inadequate grid storage or non-finite float grid values are errors before accessing sample data outside the declared views.
