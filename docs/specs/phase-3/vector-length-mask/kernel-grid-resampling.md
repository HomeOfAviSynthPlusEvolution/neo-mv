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

For unsigned mask samples or signed displacement samples use the exact numerator

$$Z=(D_x-R_x)(D_y-R_y)G_{00}+R_x(D_y-R_y)G_{10}+(D_x-R_x)R_yG_{01}+R_xR_yG_{11},$$
$$G'(x,y)=\left\lfloor\frac{Z+(D_xD_y)/2}{D_xD_y}\right\rfloor.$$

There is one final rounding, with halfway results toward positive infinity, including negative values. Do not round a horizontal intermediate. DxDy is even. A convex weighted average and this rounding remain inside the input integer range; no added clamp is needed. Integer masks arrive already quantized by their mask kernel. Resizing the unquantized mask scores instead would produce different pixels.

The supported integer inputs here are mask samples in [0,2^ba-1] for ba=8..16, and signed displacement components in [-32768,32767]. Both produce results in the same respective range. Use sufficiently wide integer arithmetic or an equivalent exact rearrangement: overflow of a chosen 64-bit intermediate is not a valid reason to reject otherwise representable geometry and storage. The formula does not require allocating a WB by HB intermediate image.

## Float samples

For finite binary32 samples use a=fl64(Rx/Dx), b=fl64(Ry/Dy), c=fl64(1-a), e=fl64(1-b), with integer operands converted to binary64 for these divisions. Define

$$h_0=\operatorname{fl64}(\operatorname{fl64}(cG_{00})+\operatorname{fl64}(aG_{10})),\quad h_1=\operatorname{fl64}(\operatorname{fl64}(cG_{01})+\operatorname{fl64}(aG_{11})),$$
$$G'(x,y)=\operatorname{fl32}(\operatorname{fl64}(\operatorname{fl64}(eh_0)+\operatorname{fl64}(bh_1))).$$

Require finite intermediates and result. No range remapping is performed. Integer and float modes have different rounding contracts; neither mode uses a quantized integer approximation of a or b.

## Examples and failures

- Nx=2,Ny=1,Bx=By=4,Ox=Oy=0,Wr=8,Hr=4, each row of the unsigned grid is [0,100]. Every output row is [0,0,13,38,63,88,100,100]. With Wr=6, keep its first six entries; do not compute an alternative six-pixel stretch.
- The same geometry with signed grid [-1,0] produces [-1,-1,-1,-1,0,0,0,0]. A grid [-1,1] at Nx=2,Bx=3,Wr=6 has x=1 weight 0 and x=2 weight 1/3: results -1 and 0 respectively.
- Nx=Ny=2, Bx=By=3, overlap zero, G=[[0,1],[0,3]], Wr=Hr=6. At x=y=2 both fractions are 1/3; the exact weighted value is 5/9 and the integer result is 1. Rounding each horizontal row first would produce horizontal values 0 and 1, then a vertical value of 1/3 rounded to 0. Such intermediate rounding is forbidden.
- A float grid [0,1] with the first geometry produces [0,0,0.125,0.375,0.625,0.875,1,1]. A constant grid remains constant, including Nx=Ny=1.
- Nx=Ny=1,Bx=By=2147483647,Ox=Oy=0,Wr=Hr=1,G=[255] produces one sample 255. Its formal weighted numerator exceeds signed 64-bit range, but its geometry, input view and output view need no large allocation. It is a supported call, not an overflow fallback.
- Wr=9 with WB=8 is an error. Invalid views, inadequate grid storage or non-finite float grid values are errors before accessing sample data outside the declared views.
