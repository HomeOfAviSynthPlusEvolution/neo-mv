# Pyramid and block geometry

## Inputs and outputs

Inputs: actual luma dimensions W,H; block Bx,By; overlap Ox,Oy; luma padding px,py; global chroma ratios rx,ry; pel in {1,2,4}; and `onelevel`. Output: block-grid dimensions, working dimensions, level count, and each plane/level's working and padded dimensions.

W,H are positive and at least Bx,By. Bx,By must be one of the allowed positive block pairs in [Super](plugin.md); 0<=Ox<=floor(Bx/2) and likewise Y. Ratios are 1 or 2; GRAY uses 1,1. Dimensions, block sizes and overlaps must align with their corresponding chroma ratios. Padding is positive in luma units; per-plane padding may round to zero. A block has positive steps sx=Bx-Ox, sy=By-Oy.

$$N_x=\left\lceil\frac{W-O_x}{s_x}\right\rceil,\quad W_0=N_xs_x+O_x,$$
$$N_y=\left\lceil\frac{H-O_y}{s_y}\right\rceil,\quad H_0=N_ys_y+O_y.$$

For plane ratios ux,uy (1,1 for Y; rx,ry for U/V), actual dimensions are W/ux,H/uy, working dimensions W0/ux,H0/uy, and hp=floor(px/ux), vp=floor(py/uy).

## Levels

For integer d,r,p define

$$F(d,r,p)=r\left\lfloor\frac{\lfloor d/r\rfloor+\mathbf1_{p\ge r}}{2}\right\rfloor.$$

Starting with actual W,H, repeatedly apply F horizontally using rx,px and vertically using ry,py. Let t count consecutive reduced pairs still at least Bx,By before the first failing pair. L=max(1,t); do not add the initial image to t. `onelevel` forces L=1.

Plane level 0 has the working dimensions above. Each next level has w'=F(w,rx,hp), h'=F(h,ry,vp). Even for U/V, use global rx,ry in F, with that plane's padding. Padding stays constant across levels. Every resulting dimension must be positive. Padded dimensions are a=w+2hp, c=h+2vp. Level 0 has pel squared logical phases; all other levels have only the integer phase. This defines no physical stacking, row pitch, guard row or alignment.

For reduction to w',h', require 2w'<=w+hp and 2h'<=h+vp. The reduction operator gives the remaining readable-region requirements.

## Examples

- GRAY, W=18,H=10, block 8x8, overlap 4x4: Nx=4,Ny=2 and working size 20x12. Padding 2x2 gives a 24x16 integer phase.
- GRAY 64x64, block 8x8, padding 16: reduced candidates 32,16,8 pass and 4 fails. L=3; stored working levels are 64,32,16.
- YUV420 72x72, block 8x8, no overlap, padding 3: L=3. Y working levels are 72,36,18; U/V levels are 36,18,8. U/V use hp=vp=1 and global rx=ry=2.

Consumers: Super allocation, Analyse level/grid derivation. This kernel has no pixel input, scratch buffer or aliasing issue.
