# Projected SAD mask grid

Inputs are a Complete scene-eligible field, block geometry, pel p, analysis precision ba, maximum M and the shared f,gamma,t. Output is a quantized Nx by Ny mask grid. Vectors determine which block's SAD is visualized; no image samples are read and SAD is not recomputed.

Let sx=Bx-Ox and sy=By-Oy. At creation compute

$$h_x=\operatorname{trunc}\frac{(256-t)16}{s_xp},\qquad h_y=\operatorname{trunc}\frac{(256-t)16}{s_yp},$$
$$a=\operatorname{fl32}\left(\frac{\operatorname{fl32}(4f)}{\operatorname{fl32}(B_xB_y)}\right).$$

Require finite intermediate constants and a. At each block bx,by with its own vector vx,vy compute exact integer expressions

$$i=b_x-\operatorname{trunc}(v_xh_x/4096),\qquad j=b_y-\operatorname{trunc}(v_yh_y/4096).$$

If either coordinate is outside its grid range, replace both with i=bx,j=by. This is a self-block fallback, not independent coordinate clamping. Let S be the raw int64 SAD at the selected block. Define

$$s=\left\lfloor S/2^{\min(16,b_a)-8}\right\rfloor,\qquad z=\operatorname{fl32}(\operatorname{fl32}(s)a),$$
$$L=\operatorname{fl32}(M\operatorname{pow32}(z,\gamma)),\qquad G(b_x,b_y)=Q(L).$$

Require finite required intermediates and L. [Q and power](../vector-length-mask/kernel-mask-input.md) are shared with the other masks. AnalysisChroma does not add a divisor to this formula; its only additional effect is in scene classification. Use analysis depth for the SAD shift, even for a float mask (ba=32 gives a shift of eight). Resize the quantized grid with [grid resampling](../vector-length-mask/kernel-grid-resampling.md).

## Examples

- Integer8, 8x8 blocks, ml=1,gamma=1,time=100 gives a=1/16 and no projection. S=8 gives L=127.5 and block value 127; S=16 gives 255. A constant grid with S=8 produces visible value 127.
- With ba=10, the same raw S=8 first becomes s=2 and produces trunc(1023*2/16)=127. With ba=32, raw S=2048 becomes s=8 and produces float 0.5.
- At Nx=3,Ny=1,Bx=By=4,overlap=0,p=1,ml=1,gamma=1, SAD=[0,4,8], take the center vector (4,0), and zero vectors in the other blocks. At time=0, hx=1024: the center selects SAD[0], giving block grid [0,0,255]. At time=100 the grid is [0,255,255]. This is a kernel example with sufficient public vector padding and scene eligibility supplied by the caller.
- If a projection's i is valid but j=-1, use the block's own SAD, not the SAD at column i of its current row. With vx=-1,hx=1024, trunc(-1024/4096)=0; floor division here would incorrectly select a different column.
