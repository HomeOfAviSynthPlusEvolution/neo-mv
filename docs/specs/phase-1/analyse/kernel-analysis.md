# Multilevel motion analysis

## Inputs and output

Inputs: current/reference logical Super pyramids, validated block/overlap parameters and analysis controls from [Analyse](plugin.md), temporal field shift f, and sample precision b. Output: one finest-level row-major grid of (vx,vy,raw SAD). Dependencies are [block error](kernel-block-error.md), [motion search](kernel-motion-search.md), and [vector prediction](kernel-vector-prediction.md).

This is a composition of kernels. Intermediate level grids carry mathematical dependency values; they do not prescribe classes, buffers or a particular scheduling strategy. Writable grids/scratch are private to a call. Parent data must remain readable until dependent predictions are complete.

## Grid and layer geometry

Let Wr,Hr be actual luma dimensions, W,H the Super working dimensions, hp,vp its luma padding, Bx,By block size, Ox,Oy overlap and sx=Bx-Ox,sy=By-Oy.

$$N_x=\left\lceil(W_r-O_x)/s_x\right\rceil,\quad W_B=N_xs_x+O_x,$$
$$N_y=\left\lceil(H_r-O_y)/s_y\right\rceil,\quad H_B=N_ys_y+O_y.$$

Require WB<=W,HB<=H. Calculate Lmax using the [Super level-count rule](../super/kernel-geometry.md) with starting dimensions W,H, this block size, and the Super ratios/padding. For argument t=levels,

$$L=\min(L_{Super},\ t>0\ ?\ t:L_{max}+t).$$

Require L>=1. A positive t is limited by LSuper, not separately by Lmax; nevertheless every resulting grid must be positive and safely sampleable. At layer l, zero being finest,

$$N_{xl}=\operatorname{trunc}\frac{\lfloor W_B/2^l\rfloor-O_x}{s_x},\quad
N_{yl}=\operatorname{trunc}\frac{\lfloor H_B/2^l\rfloor-O_y}{s_y}.$$

Blocks keep their sizes and overlaps at all layers. p=SuperPel for l=0, otherwise 1. For block origin x=bx*sx,y=by*sy and actual Super working dimensions wl,hl at this layer, ordinary candidates belong to Omega:

$$-p(x+\lfloor hp/2^l\rfloor)\le v_x<p(w_l-x-B_x+\lfloor hp/2^l\rfloor),$$
$$-p(y+\lfloor vp/2^l\rfloor)\le v_y<p(h_l-y-B_y+\lfloor vp/2^l\rfloor).$$

Require a nonempty integer domain and the [whole-domain sampling precondition](kernel-block-error.md#sampling-admissibility-and-geometry-errors): every integer vector in Omega must be sampleable in all enabled planes, for every block and layer. Reject failing geometry before analysis, rather than reducing Omega or waiting for a search to visit an unsafe point. `clip` below still clamps to the inclusive lower and exclusive-upper-minus-one bounds of this unchanged Omega.

Additionally validate the special seed Z=(0,f) for every block, even if outside Omega; do not clamp or omit it. For geometry admission the set of f values is {0} at coarser layers, and also at the finest layer when fields=false, pel=1 or delta is even. Otherwise use {0,-pel/2,+pel/2}, regardless of tff or the parities eventually read from individual frames. Require Safe(Z) for every member of that set at creation. This validates geometry without predicting frame metadata; actual f and missing-field errors still follow the plugin's field rules. No image content or search path can bypass these requirements.

## Parameter scaling

Let area=Bx*By, M=2^min(16,b)-1. With binary64 rounding after each operation, define

$$Q_b(z)=\operatorname{trunc}(((\operatorname{binary64}(z)M)/255)+0.5).$$
$$\lambda_0=Q_b(\operatorname{trunc}(mvlambda\,area/64)),$$
$$L_s=\operatorname{trunc}(Q_b(lsad)\,area/64),\quad B_s=\operatorname{trunc}(Q_b(badsad)\,area/64).$$

The inner area scaling for lambda is integer arithmetic; it precedes Qb. lsad/badsad use the opposite order and may be negative. Do not clamp them to zero or interpret +0.5 then trunc as symmetric rounding for negatives.

At layer l let lambdaBase=floor(lambda0/p^2)*2^(l*plevel), except that the first block row always uses zero. All conversions, products and squared-distance costs must be representable.

## Per-layer controls and predictors

Determine layers from coarsest to finest. Coarsest initial triples are zero. Each finer grid uses parent interpolation; global prediction uses parent final vectors when enabled. `f` is nonzero only at the finest layer. [Vector prediction](kernel-vector-prediction.md) defines all global and spatial values and meander dependencies.

The finest layer uses requested search and pelsearch. Coarser layers use requested searchparam and search=1, except requested search=4/5 stays 4/5. trymany=0 disables multiple starts; 1 enables them only at l>0; 2 enables them everywhere.

For the current block retain uPre=clip(initial parent predictor), with its original SAD. Form P0..P3. On the coarsest layer set u=P0; elsewhere set u=uPre. Let su be this predictor's retained SAD, and calculate in binary64

$$t=L_s/\max(L_s+\lfloor s_u/2\rfloor,1),\qquad
\lambda=\operatorname{trunc}((\lambda_{Base}t)t).$$

The block's global position G is clamped from the immutable layer-global predictor as described by the prediction kernel. Seeds, in priority order, are Z=(0,f), G, u, P0,P1,P2,P3. Measure fresh raw errors using the block-error kernel. Initial seed costs are s(Z)+floor(pzero*s(Z)/256), s(G)+floor(pglobal*s(G)/256), and s(u). If globalmv=false use pglobal=pzero. For each Pi use search cost C0 with fixed u and lambda. A Pi coinciding with Z,G,u or an earlier Pi has no separate priority. Z,G,u retain their own seed costs even when their positions coincide.

## Single-start selection

Start with Z's position/cost/raw error. Strictly compare G, then u, then each distinct Pi, retaining the earlier result on ties. Apply the selected search refinement with q=pnew, fixed u, lambda and Omega to that result. Z and G initial costs have no distance term; ordinary refinement candidates do.

## Multiple-start selection

Let F be the selected search relation with q=pnew and the same fixed predictor u, adaptive lambda, range and Omega for every start. For each seed j admitted above, independently define

$$R_j=F(v_j,c_j,s(v_j);u,\lambda).$$

The initial position is that seed's position, its initial comparison cost is the seed cost defined above, and its raw error is freshly measured there. No start receives another start's result. Duplicate Pi contributes no additional start.

Select the Rj having the smallest final comparison cost; break ties by the seed priority Z,G,u,P0,P1,P2,P3. Return its vector and raw error. The mathematical minimum has no finite sentinel or implicit cost ceiling. Different starts may be computed in any order or concurrently without changing this result.

## Bad-block expansion

Let sf be the current block's ordinary raw error. Expand exactly when sf>Bs; equality leaves the ordinary result unchanged. This threshold applies to every block, including the first row, and does not depend on how many previous blocks were expanded. Continue comparisons from the ordinary result, keeping its cost, predictor u, adaptive lambda and q=pnew.

- badrange>0: apply search=3 with range badrange*p, fixed initial cross/G centre (0,0), and the ordinary result as the retained initial state.
- badrange<0: compare Q((0,0),r,p) for r=1,1+p,... strictly below -badrange*p. After each ring stop if current raw SAD<floor(sf/4).
- badrange=0 adds no candidates in this step.

Still within the sf>Bs expansion path, fix o to the resulting winner and compare Q(o,1,1) through Q(o,p-1,1). With p=1 this final list is empty. If sf<=Bs, skip these final rings as well. Strict comparison cost decides winners; the raw-SAD stopping condition does not replace it.

The final triple for every block is its position and raw error, regardless of internal comparison cost. Export only the finest grid, always in row-major order.

## Examples

- Wr=18,Hr=10, block 8x8, overlap 4x4 gives grid 4x2 covering 20x12. The Super working extent must contain it.
- Lmax=4,LSuper=3: levels 0,-1,-2,1 gives L=3,3,2,1.
- 10-bit 4x4 with mvlambda=5 gives lambda0=Qb(1)=4. lsad=5 gives Ls=trunc(20*16/64)=5.
- lambdaBase=100,Ls=100,su=200 gives t=0.5,lambda=25. Squared distance 16 adds floor(25*16/256)=1. On the first row it adds zero.
- A seed with s=100,pzero=256 has comparison cost 200; if selected its exported SAD is still 100.
- Identical constant frames and phases, fields=false, yield only (0,0,0) triples: zero costs cannot be strictly improved.
- For 4x4 GRAY8, single level, pad=4,pel=1,search=4,pelsearch=1,pnew=0,trymany=0, current rows [10,20,30,30] and reference rows [0,10,20,30], SAD at zero/left/right is 120/240/0. The result is (1,0), packed vector 1, SAD=0.
- Single-block 4x4 YUV444, current all 0, reference all 255, trymany=2, all penalties zero: all positions have raw error 12240, so every start retains its seed. The first seed wins the final tie; output is zero motion with SAD=12240. No comparison ceiling changes the winner or exported error.
- If independent starts Z,G,u finish with comparison costs 6000,5000,5000, choose G by minimum cost and seed priority, regardless of which start finishes last.
- A block with raw error 100 expands when Bs=99 and does not expand when Bs=100, regardless of its grid index or previous blocks' errors.
- 8-bit 8x8 with lsad=-2 gives Qb=-1,Ls=-1. With su=0 and lambdaBase=100, t=-1 and lambda=100. Negative lsad does not disable the penalty.
