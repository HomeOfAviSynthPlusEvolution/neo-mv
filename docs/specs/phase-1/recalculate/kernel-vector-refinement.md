# Vector-grid mapping and refinement

Inputs: old validated vector grid and metadata; new current/reference Super samples; target Bx,By,Ox,Oy,p; smooth, thsad, mvlambda, search,searchparam,pnew,chroma and satd. Output: a target row-major grid of freshly measured triples. Old analysis precision must equal Super precision. This operator reuses [block error](../analyse/kernel-block-error.md) and [motion search](../analyse/kernel-motion-search.md); pass chroma and satd to every fresh error evaluation, including the mapped predictor and all refinement candidates.

Old geometry and pel may differ from the target; old actual/working sizes do not scale coordinates automatically. The mapping below uses block centres directly in their stated coordinate systems. Inputs are immutable; no target block depends on another target block's result. Scratch and output are private and cannot alias old data.

## Target geometry and bounds

Compute the target grid by the Analyse finest-level ceil formula from actual Super dimensions, requiring coverage within the Super working dimensions W,H. For target origin x=bx(Bx-Ox),y=by(By-Oy), use

$$-p(x+hp)\le v_x<p(W+hp-x-B_x),$$
$$-p(y+vp)\le v_y<p(H+vp-y-B_y).$$

Omega is the integer rectangle defined by these inequalities. Require it to be nonempty and satisfy the [whole-domain sampling precondition](../analyse/kernel-block-error.md#sampling-admissibility-and-geometry-errors) for every target block. Every integer vector in Omega must be sampleable in all enabled planes, even if a mapped vector's error would avoid refinement. Reject failing target geometry at creation; do not shrink Omega or exclude individual in-Omega candidates. Clamping of the mapped vector still uses the unchanged upper-minus-one endpoints. Recalculate has no separately admitted seed outside Omega and adds no field shift.

## Old-grid prediction

Use suffix 0 for old block/overlap/grid/pel. Let sx0=Bx0-Ox0,sy0=By0-Oy0 and

$$c_x=\lfloor B_x/2\rfloor+(B_x-O_x)b_x,\quad
j_x=\operatorname{trunc}((c_x-\lfloor B_{x0}/2\rfloor)/s_{x0}),$$
$$d_x=\max(0,c_x-(\lfloor B_{x0}/2\rfloor+s_{x0}j_x)).$$

Define cy,jy,dy identically. A,B,C,D are old vectors at (jx,jy),(jx+1,jy),(jx,jy+1),(jx+1,jy+1), clamping each lookup to the old grid. Compute dx,dy before clamping indices. Half-block divisions are integer floor, even for an externally supplied odd block size; j division is trunc toward zero.

If smooth=false, select left when 2dx<sx0 and right otherwise; select top when 2dy<sy0 and bottom otherwise. Ties select right/bottom. If smooth=true, for vx and vy separately calculate

$$u=A s_{x0}+d_x(B-A),\quad v=C s_{x0}+d_x(D-C),$$
$$I=\operatorname{trunc}\left((u+\operatorname{trunc}(d_y(v-u)/s_{y0}))/s_{x0}\right).$$

Keep both truncations. Scale each selected/interpolated vector component z to floor(z*p/p0), then clamp its position to Omega. Old SAD values are validated as part of the input field but are not interpolated, area-scaled, or used for refinement decisions; every output error is measured on the new samples. If valid old metadata has unavailable/count-mismatched arrays, use an all-zero old vector grid before this mapping. Malformed complete arrays remain errors.

## Threshold and refinement

Use Qb and area/depth scaling from [analysis](../analyse/kernel-analysis.md):

$$\lambda_0=Q_b(\operatorname{trunc}(mvlambda\,B_xB_y/64)),\quad
T=\operatorname{trunc}(Q_b(thsad)\,B_xB_y/64).$$

When chroma is enabled replace T by T+2*trunc(T/(rx*ry)), with division before multiplication. thsad may be negative; preserve signed truncation. Per-block lambda is zero on the first block row and floor(lambda0/p^2) elsewhere. There is no LSAD adaptation, level multiplier, global predictor, multistart or bad-block expansion.

Let u be the mapped/clamped position. Recompute s(u) on new samples. If s(u)<=T, output (u,s(u)) without searching. Otherwise initialize (u,s(u),s(u)) and apply the requested search relation using fixed predictor u, lambda, q=pnew and range max(1,searchparam). Output its winning position and raw SAD. With T<0 every nonnegative s triggers refinement.

Field metadata may be validated by the plugin, but this mapping and error evaluation adds no extra field shift to the input vector. Meander does not change independent per-block mathematical results.

## Examples

- Old blocks 8x8 without overlap, target 16x8: first target centre is (8,4); jx=0,dx=4,sx0=8. Nearest selection chooses the right old block. Old left/right vx=0,4 yields 4, while smooth selection yields 2.
- Interpolated vx=-1,p0=2,p=1 scales to -1, not zero, before clamping.
- Old SAD=999,new measured s=100,T=100: retain vector and export 100 without refinement. T=99 triggers search.
- thsad=5,8-bit block 4x4,YUV420: base T=1; chroma adjustment leaves 1+2*trunc(1/4)=1.
- Current rows [10,20,30,30], reference [0,10,20,30], block 4x4,pad=4,pel=1,old vector zero,thsad=0,search=4,range=1,pnew=0: initial s=120, final vector (1,0), SAD=0.
