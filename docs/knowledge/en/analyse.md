# Analyse: from image blocks to motion vectors

This article follows `Analyse` from two frames of Super data to motion vectors: block grids and sample positions, error measurement, prediction, candidate search, and result encoding.

## 1. What the function computes

`Analyse` divides the current image into rectangular blocks, with or without overlap. For each block it selects a reference-image displacement that wins under the defined candidate comparison rules. Each block finally has a horizontal displacement, vertical displacement, and newly measured raw matching error.

Winning does not mean finding the lowest error anywhere in the image. Only positions visited by the search rules are compared, and costs may include predictor-distance and candidate-error penalties. Vectors depend on errors, predictors, candidate sets, and tie rules.

Inputs mainly come from `super`: current/reference pyramid images, finest-level subpixel phases, and their descriptors. Analyse neither builds the pyramid nor interpolates new subpixel images.

The output is a video node with the same visible pixels, dimensions, format, frame count, and rate as Super. Analysis results reside in properties. The visible image is not a vector visualization; the new results are the finest-level vector and error arrays.

## 2. Objects and notation

### 2.1 Frames and direction

Output index is n; reference index is `n+delta`. Delta 1 means the following frame and −1 the preceding frame. Zero is invalid.

Vector `(vx,vy)` tells how many vector units to move from the current block's corresponding position when sampling the reference. Positive X is right and positive Y down. This is a reference sampling displacement: its sign alone, without the temporal relationship between current and reference frames, does not describe an object's direction through time.

### 2.2 Levels, blocks, and units

| Symbol | Meaning |
| --- | --- |
| `l` | Pyramid level; 0 is finest, larger indices mean smaller images |
| `Wr,Hr` | Visible luma dimensions |
| `W,H` | Finest Super working dimensions without padding; may exceed visible size |
| `hp,vp` | Horizontal/vertical luma padding |
| `Bx,By` | Block size in the current level's luma pixels |
| `Ox,Oy` | Horizontal/vertical overlap |
| `sx,sy` | Block steps `Bx-Ox,By-Oy` |
| `bx,by` | Zero-based block column/row |
| `p` | Vector units per current-level luma pixel |
| `rx,ry` | Luma-to-chroma dimension ratios; 2, 2 for YUV420 |

The finest level uses `p=SuperPel`; all coarser levels use p=1. With pel=2, vector `(-3,2)` samples at `(-1.5,+1)` luma pixels. A component cannot retain the same interpretation when transferred between levels.

`floor` rounds toward negative infinity; `trunc` toward zero: `floor(-3/2)=-2`, `trunc(-3/2)=-1`. Negative vectors make this distinction observable.

### 2.3 Three different matching quantities

- **Inherited predictor error** comes from a parent interpolation or neighbor. It need not describe the current block at that position.
- **Raw error s** is freshly measured luma plus chroma error against the current reference image.
- **Comparison cost c** selects winners and can include penalties beyond raw error.

Inherited error adapts the prediction penalty; cost determines replacement; exported `AnalysisSAD` stores raw error. These quantities are not interchangeable.

## 3. Overall calculation

For an available reference:

1. Determine grids, levels, candidate domains, and validate complete sampling support.
2. Start at the coarsest level with zero initialization. Finer levels derive initial predictors from completed parent results.
3. Process blocks in each level's defined dependency order.
4. Form parent, spatial, global, and zero-displacement seeds.
5. Measure seed errors and initial costs, then run one or multiple searches.
6. If the resulting raw error exceeds the bad-block threshold, extend the search.
7. Pass results to the next level, then export finest-level vectors/errors in row order.

The nesting is level → block → candidate. Dependencies exist between levels and neighboring blocks, but frame n does not depend on whether output n−1 has already been evaluated.

## 4. Step-by-step calculation

### 4.1 Covering the image with blocks

Origins are separated by `sx=Bx-Ox,sy=By-Oy`. Finest-level coverage rounds block counts upward:

$$N_x=\left\lceil\frac{W_r-O_x}{s_x}\right\rceil,\qquad
N_y=\left\lceil\frac{H_r-O_y}{s_y}\right\rceil.$$

Actual coverage is:

$$W_B=N_xs_x+O_x,\qquad H_B=N_ys_y+O_y.$$

The final block must exist in full, not just its visible part. Thus `WB≤W,HB≤H`: Super must contain every block.

Visible `18×10`, block `8×8`, overlap `4×4` gives step 4, a `4×2` grid, and `20×12` coverage. This does not resize the original image; analysis reads the working edge samples prepared by Super.

### 4.2 Levels and their grids

Let LSuper be available Super levels, Lmax the limit calculated through the Super level-count rule, and t the user's levels value:

$$L=\min\left(L_{Super},\begin{cases}t&t>0\\L_{max}+t&t\le0\end{cases}\right).$$

Zero levels starts from Lmax rather than disabling the pyramid; negative values reduce that count. If `Lmax=4,LSuper=3`, values `0,-1,-2,1` give `3,3,2,1` levels. L must remain at least 1.

The Lmax counting rule reduces one dimension using:

$$F(d,r,h)=r\left\lfloor\frac{\lfloor d/r\rfloor+\mathbf1_{h\ge r}}{2}\right\rfloor.$$

Start from Super working dimensions W, H with global chroma ratios and luma padding. Count consecutive reduced sizes still at least as large as the block; call this k, and set `Lmax=max(1,k)` without adding one for the original. GRAY `64×64`, block `8×8`, padding 16 has passing reductions 32, 16, 8, then failing 4: count 3. LSuper still bounds the final result.

Positive levels is bounded directly by LSuper, not additionally by Lmax. Every selected level must nevertheless have a valid grid and safe sample support.

Block dimensions and overlap stay constant; image reduction reduces block counts:

$$N_{xl}=\operatorname{trunc}\frac{\lfloor W_B/2^l\rfloor-O_x}{s_x},\qquad
N_{yl}=\operatorname{trunc}\frac{\lfloor H_B/2^l\rfloor-O_y}{s_y}.$$

Both counts must be positive. Grid quantity `floor(WB/2^l)` differs from the actual Super level width wl obtained through Super's reduction rule. The latter controls sample bounds.

### 4.3 Ordinary candidate domains

For block origin `(x,y)=(bx·sx,by·sy)` and level working size wl, hl, ordinary vectors lie in integer rectangle Ω:

$$-p\left(x+\lfloor h_p/2^l\rfloor\right)\le v_x
<p\left(w_l-x-B_x+\lfloor h_p/2^l\rfloor\right),$$
$$-p\left(y+\lfloor v_p/2^l\rfloor\right)\le v_y
<p\left(h_l-y-B_y+\lfloor v_p/2^l\rfloor\right).$$

Lower bounds are inclusive, upper bounds exclusive. Range `[-8,24)` ends at integer 23; predictor clipping therefore uses −8 and 23 as inclusive endpoints.

Candidate eligibility alone does not prove safe reads on all planes. Chroma conversion, rounded padding, and smaller support of some subpixel phases can invalidate positions. Geometry validation must establish that **every integer vector in Ω** can read complete current/reference blocks, not merely the positions one search happens to visit.

For example, YUV420 with pel=2 and luma padding 3 has chroma padding 1. Some negative vectors can fit luma but exceed chroma support. Such geometry is rejected, even with a small search range; samples are not dynamically clamped.

The special zero seed `Z=(0,f)` is separately validated, even outside Ω, and is neither clipped nor omitted. f is the field shift explained below. When a half-pixel field shift is possible, creation validates finest-level `f=0,-pel/2,+pel/2`; otherwise f=0.

### 4.4 From vectors to sample positions

Luma reference displacement is `(vx/p,vy/p)`. Decompose into integer offset and phase:

$$v_x=pq_x+a_x,\qquad q_x=\lfloor v_x/p\rfloor,\quad0\le a_x<p.$$

Do the same vertically. Reference sample i, j is read from phase ax, ay at:

$$(h_P+x+i+q_x,\ v_P+y+j+q_y),$$

where hP, vP are actual padding of that level's luma plane. The current block always uses the integer phase. Analyse selects existing Super samples without new interpolation.

For p=2, vx=−1, integer offset −1 plus phase 1 means `−1+1/2=−0.5`. Truncation toward zero would choose the wrong integer position or a negative phase.

For chroma, first convert vector components:

$$v_{xc}=\operatorname{trunc}(v_x/r_x),\qquad v_{yc}=\operatorname{trunc}(v_y/r_y).$$

Then floor-decompose them with the same p. Divide block origin and size by the chroma ratios and use that plane's padding. The first division truncates, the second floors; they cannot be merged arbitrarily.

With rx=2, p=2, vx=−1 becomes chroma vector zero. vx=−3 becomes −1, then integer −1/phase 1: −0.5 chroma pixels.

### 4.5 Measuring raw error

#### Integer SAD

For current and reference sample matrices S, R:

$$SAD(S,R)=\sum_j\sum_i|S_{j,i}-R_{j,i}|.$$

No area division or automatic 8-bit conversion occurs. An `8×8` block differing by 2 at every sample gives 128, whether storage is 8-bit or 10-bit, provided the numeric difference is 2.

Call luma error eY. With chroma enabled, sum separate U/V SADs into eC without compensating for subsampling area; otherwise eC=0. Raw error is `s=eY+eC`. A YUV420 `8×8` luma block with differences 1 on all planes gives `64+16+16=96`. For a `6×6` block, the two chroma blocks are `3×3`; integer samples differing by 1 give `36+9+9=54`.

#### Luma SATD

With satd=true, split luma into nonoverlapping `4×4` cells. For difference matrix E=S−R, compute:

$$H=\begin{pmatrix}1&1&1&1\\1&-1&1&-1\\1&1&-1&-1\\1&-1&-1&1\end{pmatrix},\qquad F=HEH^T.$$

The transform converts 16 spatial differences into 16 sum/difference combinations. Sum absolute coefficients, divide by 2 with floor, and sum cells:

$$e_Y=\sum_{cells}\left\lfloor\frac{\sum_{a,b}|F_{a,b}|}{2}\right\rfloor.$$

A `4×4` cell of differences all 1 has SAD 16 and only one nonzero transform coefficient, 16, giving SATD 8. One isolated difference of 1 has SAD 1 and sixteen absolute coefficients of 1, also giving SATD 8. SATD is not a fixed scaling of SAD.

Larger blocks do not use one large transform. `6×6` and `16×2` cannot be tiled this way and disallow SATD. U/V always use SAD. The property remains named `AnalysisSAD` even when luma uses SATD.

#### Encoding float32 errors

Floating SAD computes binary32 differences/absolute values and accumulates from +0 in row order, rounding every subtraction/addition. Finite samples may lie outside `[0,1]`; do not clip them first.

Floating SATD applies this transform to rows, then columns, with binary32 rounding at every add/subtract:

$$h(z)=\big((z_0+z_1)+(z_2+z_3),\ (z_0-z_1)+(z_2-z_3),
\ (z_0+z_1)-(z_2+z_3),\ (z_0-z_1)-(z_2-z_3)\big).$$

Visit cells in row order. In each cell, visit columns 0–3, sum each column's four absolute coefficients left-associatively, then add the column sum to a total starting at +0. Multiply by 0.5 after accumulating all cells; do not convert each cell to integer first.

Independently encode each plane's floating error e. With fl denoting binary32 rounding:

$$z=fl(e\cdot65535),\qquad Q(e)=\begin{cases}
0&z\le0\\4294967295&z\ge4294967040\\
\operatorname{trunc}(fl(z+0.5))&\text{otherwise}.
\end{cases}$$

Compute Q(eY), Q(eU), Q(eV) separately, then add in wide integers. Two plane errors 0.5 contribute `32768+32768=65536`; encoding their sum 1 would give 65535 instead. All actual floating intermediates must remain finite.

### 4.6 Forming predictors before search

Predictors supply starting positions and inherited errors. They still need fresh measurement on the current block.

#### Parent predictors: from coarse to fine grids

At the coarsest level every initial triple is `(0,0,0)`. Finer levels interpolate the completed parent grid.

For child block k, j, clamp indices to the doubled parent-grid extent to obtain i, t. Define:

$$d_x=2(i\bmod2)-1,\quad d_y=2(t\bmod2)-1,\quad A=(\lfloor i/2\rfloor,\lfloor t/2\rfloor).$$

A is a parent-grid location; dx, dy are ±1 and select a neighboring side. Obtain four triples V1…V4:

| Child index at first/last boundary of doubled grid | Parent locations |
| --- | --- |
| Both axes | `A,A,A,A` |
| X only | `A,A,A+(0,dy),A+(0,dy)` |
| Y only | `A,A,A+(dx,0),A+(dx,0)` |
| Neither | `A,A+(dx,0),A+(0,dy),A+(dx,dy)` |

Offsets here are grid indices, not vector components. Repetition avoids nonexistent neighbors.

Without overlap, each coordinate forms numerator `U=9V1+3V2+3V3+V4`. Error uses the same weights plus 8. Weights sum to 16; coordinates also require level-unit conversion.

With overlap on either axis, derive weights from block spacing:

$$a_x=\begin{cases}3B_x-2O_x&d_x>0\\3B_x-4O_x&d_x<0\end{cases},\qquad b'_x=4s_x-a_x.$$

Define Y similarly. Weights are `ax·ay,b'x·ay,ax·b'y,b'x·b'y`. For each component, including inherited error, form the exact integer weighted sum, convert to binary64, multiply by separately rounded `1/(sx·sy)`, then truncate to obtain the numerator. This path does not add 8 to error.

For parent/current pel factors pP, p, set `r=3-log2(p)+log2(pP)`. Each predicted coordinate becomes:

$$\left\lfloor U/2^{\max(r,0)}\right\rfloor\,2^{\max(-r,0)}.$$

Inherited error is its numerator divided by 16 with floor. Coordinate division also floors: without overlap, parent X values `-1,-2,0,1` and `pP=1,p=2` give U=−14, r=2 and predicted X=−4, not −3.

#### Global predictor: common parent displacement

With globalmv enabled, find independent modes mx, my of the parent final X/Y components, selecting the smallest numeric value on ties. This is not the most frequent complete vector pair.

Retain parent blocks satisfying both `|vx-mx|<6,|vy-my|<6`. If any remain:

$$g_x=\operatorname{trunc}\frac{2\sum v_x}{\text{count}},\qquad
g_y=\operatorname{trunc}\frac{2\sum v_y}{\text{count}}.$$

Otherwise use `(2mx,2my)`. Every block counts, including repeated vectors. Multiply components by current p and add field shift f to Y. At the coarsest level or with global prediction disabled, start from zero and apply the same field handling.

Parent vectors `(0,0),(0,0),(3,0)` all pass and yield `(2,0)` before current-level scaling; deduplicating them would incorrectly change the count.

The layer's global position stays fixed. Each block independently clips it to its own Ω to obtain G; never pass a previous block's clipped value onward.

#### Spatial predictors: completed neighbors and existing initial values

Process rows top to bottom. With meander=true, even rows run left to right and odd rows right to left; otherwise all run left to right. Let direction σ be +1 or −1:

- P1 is `(bx-σ,by)`, the previous block in the row, or `(0,f,0)` if absent.
- P2 is `(bx,by-1)`, the block above, or `(0,f,0)` if absent.
- P3 prefers `(bx+σ,by+1)`, then `(bx+σ,by-1)`, otherwise `(0,f,0)`.

P3 may come from an unprocessed next row. It reads the parent-interpolated initial triple, not an uninitialized value or a future result. Completed neighbors supply final triples.

Clip coordinates of all three to Ω without changing inherited error. Except in the first row, P0 takes independent coordinate medians of P1, P2, P3 and their maximum error. In the first row, P0=P1.

Also preserve the clipped parent initial triple uPre. The fixed predictor is u=P0 at the coarsest level, otherwise u=uPre. Distance penalties remain relative to u even as the winner moves.

### 4.7 Converting controls to comparison costs

#### Area, depth, and level scaling

Set `area=Bx·By`, precision b, and `M=2^min(16,b)-1`. Float32 b=32 also uses M=65535, matching floating-error encoding. Depth conversion is:

$$Q_b(z)=\operatorname{trunc}\left(\frac{\operatorname{binary64}(z)M}{255}+0.5\right).$$

Round each floating operation to binary64. Base quantities are:

$$\lambda_0=Q_b\left(\operatorname{trunc}\frac{mvlambda\cdot area}{64}\right),\qquad
L_s=\operatorname{trunc}\frac{Q_b(lsad)\cdot area}{64},\qquad
B_s=\operatorname{trunc}\frac{Q_b(badsad)\cdot area}{64}.$$

Area scaling precedes depth conversion for λ0 but follows it for Ls, Bs. Intermediate truncations prevent rearrangement. For 10-bit `4×4`, mvlambda=5 gives `trunc(5·16/64)=1`, then λ0=Qb(1)=4. lsad=5 gives Qb(5)=20, then Ls=5.

The current level's base penalty is:

$$\lambda_{Base}=\lfloor\lambda_0/p^2\rfloor\,2^{l\cdot plevel}.$$

Every level's first block row instead uses λBase=0. This removes predictor-distance penalties, not the error penalties from pzero, pglobal, pnew.

#### How inherited error adapts the penalty

For fixed predictor u with inherited error su:

$$t=\frac{L_s}{\max(L_s+\lfloor s_u/2\rfloor,1)},\qquad
\lambda=\operatorname{trunc}((\lambda_{Base}t)t).$$

Compute in binary64 order. With positive Ls, larger inherited error generally lowers t and weakens the distance penalty. `λBase=100,Ls=100,su=200` gives t=0.5,λ=25. Inherited error therefore controls the constraint toward the fixed predictor.

This is not a monotonic rule for every parameter: lsad may be negative, and Qb's +0.5 then truncation is not symmetric rounding of negatives. At 8-bit `8×8`, lsad=−2 gives Ls=−1; su=0,λBase=100 gives t=−1 and λ=100. Negative lsad does not disable prediction penalties.

#### Ordinary candidate cost

For candidate w with freshly measured eY(w), eC(w), distance from u contributes:

$$D(w)=\left\lfloor\frac{\lambda((w_x-u_x)^2+(w_y-u_y)^2)}{256}\right\rfloor.$$

With error penalty q:

$$C_q(w)=D(w)+e_Y(w)+e_C(w)
+\left\lfloor\frac{q\,e_Y(w)}{256}\right\rfloor
+\left\lfloor\frac{q\,e_C(w)}{256}\right\rfloor.$$

This is distance penalty, raw error, and separately floored luma/chroma penalties. Do not combine the errors before penalty division.

With u=(0, 0),λ=256, q=0, candidates (1, 0),(2, 0) both of raw error 5 have costs 6, 9. A closer candidate can win despite equal raw error; its exported error remains 5.

### 4.8 Seeds and single/multiple starting points

Seed priority is:

$$Z=(0,f),\quad G,\quad u,\quad P_0,P_1,P_2,P_3.$$

Measure fresh raw errors and assign initial costs:

| Seed | Initial comparison cost |
| --- | --- |
| Z | `s(Z)+floor(pzero·s(Z)/256)` |
| G | `s(G)+floor(pglobal·s(G)/256)` |
| u | `s(u)` |
| Each Pi | `C0(Pi)`: fixed-predictor distance, without q error penalties |

With globalmv disabled, effective pglobal becomes pzero. Z/G initial costs have no distance term and floor a penalty on total error once, unlike ordinary Cq's separate luma/chroma terms.

A Pi coinciding with Z, G, u or an earlier Pi loses its separate priority/start. Z, G, u retain their own initial costs even when coincident, so one sample position may compete under several seed identities.

With trymany=0, start at Z and compare remaining seeds in order, replacing only on strict improvement. Run the selected search once from the winning seed with q=pnew.

With multiple starts, every retained seed independently runs the same search from its own position, initial cost, and fresh raw error. All share fixed u,λ,Ω but cannot inherit each other's winners. Pick the smallest final cost, breaking ties by original seed priority.

If Z, G, u finish with costs 6000, 5000, 5000, G wins regardless of evaluation completion order. There is no hidden fixed cost ceiling for accepting a result.

trymany=1 enables multiple starts only at l>0; trymany=2 enables them everywhere. The finest level uses search and pelsearch. Coarser levels normally force search=1 and effective range `max(1,searchparam)`, except user-selected horizontal/vertical search=4/5 remains that mode.

### 4.9 Generating and comparing search candidates

All searches admit new candidates only inside Ω, replace only on **strictly lower** cost, and update position, comparison cost, and fresh raw error together. Ties retain the earlier result, making visitation order part of the algorithm.

Retain the seed's initial cost rather than replacing it by C_pnew. A favorable seed cost continues competing against ordinary candidates. Search centers may move; fixed predictor u never does.

#### Square ring Q(o, r, t)

For fixed center o, radius r, and edge step t, visit offsets in order:

1. For `i=-r+t,-r+2t,…<r`, visit `(i,-r)` then `(i,r)`.
2. For the same sequence as j, visit `(-r,j)` then `(r,j)`.
3. Visit corners `(-r,-r),(-r,r),(r,-r),(r,r)`.

Q(o, 1, 1) visits up, down, left, right, then top-left, bottom-left, top-right, bottom-right. Finding a better point does not move the ring center.

#### search=1: concentric rings around a fixed start

For start o and range a, compare Q(o, 1, 1) through Q(o, a, 1), covering the integer square of radius a except points outside Ω. All rings use original o, even if a first-ring point wins.

#### search=4/5: one-axis search

Horizontal order is `(-1,0),(1,0),(-2,0),(2,0),…` through distance a; vertical swaps axes, up before down. The initial center stays fixed. Unvisited vertical points cannot win horizontal search. Equal left/right candidates favor the earlier left point unless the retained result already has that same cost.

#### search=2: moving hexagon followed by a small square ring

For a=1, execute Q(o, 1, 1) once and stop without a second ring. For a>1, first visit:

$$h_0=(-2,0),\ h_1=(-1,2),\ h_2=(1,2),\ h_3=(2,0),\ h_4=(1,-2),\ h_5=(-1,-2).$$

Without improvement, keep z=o. If direction j wins, set z=o+hj, then perform at most `max(0,floor(a/2)-1)` advances. Each compares `z+h(j-1),z+hj,z+h(j+1)` with indices modulo 6. Stop on no improvement; otherwise update position/direction to the winner.

Finally compare Q(z, 1, 1). For a=2 there are no extra advances after the first hexagon, but the final ring remains. a controls advances, not a hard displacement radius from the start.

#### search=3: cross, sparse pattern, and hexagon refinement

Around fixed original o, first visit positive odd distances less than a: all horizontal left/right pairs, then vertical up/down pairs. Then visit `o+kG` for `k=1…max(1,floor(a/4))`, using this ordered pattern:

```text
(-4, 2), (-4, 1), (-4, 0), (-4,-1), (-4,-2),
( 4,-2), ( 4,-1), ( 4, 0), ( 4, 1), ( 4, 2),
( 2, 3), ( 0, 4), (-2, 3), (-2,-3), ( 0,-4), ( 2,-3)
```

After all groups, run hexagon search of range a from the current winner. Even a=1 includes one G group, so (−4, 2) may win if inside Ω. a is not a one-pixel limit. Bad-block expansion can assign the cross/pattern a center different from the retained result.

#### search=0: decreasing steps with directional hints

Steps are `a,floor(a/2),floor(a/4),…` through 1. Improvements can continue moving at the same step; only no further improvement advances to a smaller step. a is not a final displacement bound.

Each step starts with unrestricted hint h=*. Freeze the current winner as center o for an iteration and first examine axial directions:

$$A=[(1,0),(-1,0),(0,1),(0,-1)].$$

Multiply by step d and add o. With a hint, retain only directions of positive dot product with h, preserving list order. A right hint permits right only; down-right permits right and down.

Two branches follow:

1. **An axis improves.** Let t be its winner. A horizontal move then tests down/up around t; a vertical move tests right/left, still at distance d. If this perpendicular pair improves again, retain only that perpendicular direction as the next hint; otherwise retain the axial move. Continue the same step from the winner.
2. **No axis improves.** Test diagonal list F(h) around original o. An improvement supplies its diagonal direction as the next hint and continues the step; otherwise end the step.

| Incoming h | Ordered F(h) |
| --- | --- |
| `*` | `(1,1),(-1,1),(1,-1),(-1,-1)` |
| `(1,0)` | `(1,1),(1,-1)` |
| `(-1,0)` | `(-1,1),(-1,-1)` |
| `(0,1)` | `(1,1),(-1,1)` |
| `(0,-1)` | `(1,-1),(-1,-1)` |
| `(1,1)` | `(1,1),(-1,1),(1,-1)` |
| `(-1,1)` | `(1,1),(-1,1),(-1,-1)` |
| `(1,-1)` | `(1,1),(-1,-1),(1,-1)` |
| `(-1,-1)` | `(-1,-1),(-1,1),(1,-1)` |

Do not replace this with a more symmetric-looking order. Suppose initial cost 9, d=1, right (1, 0) costs 4, bottom-left (−1, 1) costs 2, and everything else costs 9. After moving right, only positions above/below that winner are checked; the full diagonal list is skipped. The search can stop at (1, 0), missing the lower unvisited cost at (−1, 1).

If right costs 4 and bottom-right 2, successive improvements leave hint (0, 1), not overall displacement direction (1, 1). The next axial iteration therefore tests down only.

Each continued iteration strictly lowers a nonnegative integer cost, ensuring termination. A smaller step resets the hint but preserves result, predictor, and cost parameters. All modes omit new candidates outside Ω without changing centers/order or later rules. A separately validated special initial position outside Ω can remain the winner.

### 4.10 Extending a bad block's search

Let sf be the ordinary winner's raw error. Extend only if sf>Bs; equality does not extend. This is not a comparison against its penalized cost.

Keep the existing result's cost/error, fixed u, adaptive λ, and q=pnew:

- Positive badrange runs the multiscale search with range `badrange·p`, but fixes its initial cross/G-pattern center at (0, 0). The retained result still competes; subsequent hexagon refinement starts at the resulting winner.
- Negative badrange visits `Q((0,0),r,p)` for `r=1,1+p,…`, strictly below `-badrange·p`. After each ring, stop if current raw error is below `floor(sf/4)`. sf stays the pre-extension error.
- Zero badrange adds no candidates at this stage.

Once sf>Bs has been entered, also freeze the current winner as o and visit Q(o, 1, 1) through Q(o, p−1, 1). This list is empty for p=1. Thus badrange=0 does not universally disable all extension candidates at a subpixel level.

Replacement always uses strict comparison cost. The negative-badrange raw-error condition controls stopping, not vector selection. Expansion also applies on the first block row and does not depend on how many earlier blocks expanded.

### 4.11 Field shifts and output

#### Where field shift enters

With fields=false, motion ignores tff and `_Field`, and f=0. Otherwise determine current parity from explicit `top(k)=bool(tff) XOR (k is odd)` or the used frame's integer `_Field`, nonzero meaning top.

When the reference exists, SuperPel>1, and delta is odd, reference parity is also required. At the finest level current top/reference bottom gives f=+pel/2; the reverse gives −pel/2; other cases give zero. Coarse levels always use zero.

f participates in the zero seed, global position, and missing-neighbor initialization. It is part of the search, not a uniform postprocessing addition to final Y. Exported vy already includes its computational effect; do not subtract f again.

#### Exported data

Only the finest level is exported, always in row order `i=by·Nx+bx`, regardless of meander. Arrays contain Nx·Ny vectors and raw errors.

Each vector occupies an int64 bit pattern: signed X in low 32 bits, signed Y in high 32 bits. With u32(v) the nonnegative residue modulo 2^32:

$$P=u32(v_x)+2^{32}u32(v_y).$$

Store P if P<2^63, otherwise P−2^64. Negative host integers can be valid packed vectors. `(-3,2)` gives `0x00000002FFFFFFFD`, host value 12884901885, representing (−1.5,+1) pixels at pel=2.

Errors store s=eY+eC without distance or candidate penalties. A seed of s=100, pzero=256 has initial cost 200 but still exports error 100 if it wins.

Names concatenate prefix and suffix without separators, such as `MVUtensilsAnalysisVectors` and `MVUtensilsAnalysisSAD`. Scalar properties describe working/visible size, padding, pel, levels, chroma, ratios, blocks, overlap, grid, delta, and precision. `AnalysisWidth/Height` are Super's W, H, not WB, HB.

Pixels and initial properties come from current `super[n]`; unrelated properties remain. Reference properties do not replace them. `AnalysisLevels` records the level count, not arrays for every level.

## 5. A complete numerical example

Use GRAY8, one level, visible/working `4×4`, block `4×4`, overlap 0, padding 4, pel=1. Set delta=1 with an available next frame; `fields=false,trymany=0,search=4,pelsearch=1,pnew=0`, other controls default.

Every current row is `[10,20,30,30]`; every reference row is `[0,10,20,30]`. Reference edges extend with 0 on the left and 30 on the right.

### 5.1 Geometry and initialization

Step 4 gives a `1×1` grid at (0, 0). Each axis domain is `[-4,4)`, containing −1, 0, 1.

With no parent or neighbors, fixed predictor and all seeds are (0, 0) with inherited error zero. P0…P3 duplicate earlier seeds and add no candidates. pzero inherits pnew=0 and pglobal defaults to zero, so Z, G, u initial costs equal raw error. The first block row has λBase=λ=0, making search cost raw error too.

### 5.2 Raw errors

| X displacement | Reference row | Absolute differences | Row SAD | Four-row SAD |
| --- | --- | --- | --- | --- |
| 0 | `[0,10,20,30]` | `[10,10,10,0]` | 30 | 120 |
| −1 | `[0,0,10,20]` | `[10,20,20,10]` | 60 | 240 |
| +1 | `[10,20,30,30]` | `[0,0,0,0]` | 0 | 0 |

The three zero-position seeds tie at 120, retaining Z. Horizontal search visits left then right: cost 240 cannot replace 120; cost 0 replaces it with (1, 0), raw error zero. Range 1 then ends. Default badsad=10000 becomes Bs=2500 for this 8-bit `4×4` block; zero does not exceed it, so no expansion runs.

### 5.3 Output

Vector (1, 0) samples one pixel right in the next frame, perfectly matching the current block. Packed value is `1+2^32·0=1`:

```text
MVUtensilsAnalysisVectors = [1]
MVUtensilsAnalysisSAD     = [0]
MVUtensilsAnalysisNBlkX   = 1
MVUtensilsAnalysisNBlkY   = 1
MVUtensilsAnalysisPel     = 1
MVUtensilsAnalysisLevels  = 1
MVUtensilsAnalysisDeltaFrame = 1
```

Other scalars follow this geometry; visible rows remain `[10,20,30,30]`. On the last frame, delta=1 instead has no reference: no search runs, scalars are still written, and vector/error arrays for this prefix are removed. `[0]` is not substituted to pretend zero motion was measured.

## 6. Parameters and their calculation steps

Int32 values saturate to signed 32-bit range before validation. Booleans use the original host integer's zero/nonzero truth without that saturation.

| Parameter | Default and key constraints | Calculation role |
| --- | --- | --- |
| `super` | Required node with needed Super data | Geometry, current/reference samples, visible output/properties |
| `blksize` | Inherited from Super; one/two integers | Grid, error area, area scaling |
| `overlap` | Inherited; 0 through half-block per axis, chroma-aligned | Steps and parent weights |
| `levels` | 0; effective count at least 1 | Pyramid selection |
| `search` | 2; 0–5 | Ordinary candidate generation |
| `searchparam` | 2; effective minimum 1 | Coarse search range |
| `pelsearch` | SuperPel; positive | Finest range in vector units |
| `mvlambda` | 1000; nonnegative | Base distance penalty |
| `chroma` | true; forced off for GRAY, needs Super chroma | Chroma support and error |
| `delta` | 1; signed nonzero | Reference frame and field condition |
| `lsad` | 400; negatives allowed | Inherited-error adaptation of λ |
| `plevel` | 1; 0, 1, 2 | Level factor `2^(l·plevel)` |
| `globalmv` | true | Global prediction; effective pglobal when disabled |
| `pnew` | 25; 0–256 | Search error penalty q |
| `pzero` | Effective pnew; 0–256 | Zero-seed initial penalty |
| `pglobal` | 0; 0–256, effectively pzero without globalmv | Global-seed initial penalty |
| `badsad` | 10000; negatives allowed | Bad-block threshold Bs |
| `badrange` | 24; positive/negative/zero differ | Expansion candidates and stopping |
| `meander` | true | Block dependency direction |
| `trymany` | 0; 0, 1, 2 | Independent seed searches |
| `fields` | false | Field-shift geometry and calculation |
| `tff` | Omitted reads `_Field` | Parity when needed |
| `satd` | false; unavailable for `6×6` and `16×2` | Luma metric; chroma stays SAD |
| `prefix` | `MVUtensils` | Select Super data and name analysis properties |

One-element blksize/overlap arrays duplicate to both axes; two mean horizontal/vertical. Explicit empty arrays inherit corresponding Super values; more than two fail. Supported block pairs are `4×4,6×6,8×4,8×8,12×12,16×2,16×8,16×16,24×24,32×16,32×32,48×48,64×32,64×64,128×64,128×128`, still subject to geometry/chroma alignment.

## 7. Boundaries, missing data, and errors

### 7.1 No reference is different from measured zero motion

When n+delta is outside the sequence, the current frame must still be valid and creation geometry remains checked. Output complete scalar metadata, but remove selected-prefix `AnalysisVectors/AnalysisSAD`, including inherited keys. Other prefixes/properties remain.

Identical constant images with valid geometry and fields=false instead produce complete zero-vector/zero-error arrays: the zero seed already costs zero and cannot be strictly improved. That is a measurement, not missing data.

Field mode still validates current parity without a reference, but never looks up nonexistent reference parity or measures it.

### 7.2 Creation-time checks

Errors known from parameters and Super frame 0 are handled at creation: illegal enums, blocks/overlap, zero delta, nonpositive pelsearch, no valid levels, missing Super data, insufficient coverage, empty domains, or unsafe support anywhere in the candidate domain.

A one-frame clip, short search, or coincidental pixel match does not waive domain safety. Skipping candidates, enlarging padding, filling undefined phases, or clamping individual reads does not repair invalid geometry.

### 7.3 Frame-time checks

Required current/reference frames must match established logical Super geometry and sample format. Changed metadata, missing required parity, invalid memory views, and nonfinite floating intermediates can fail at frame evaluation.

Views require positive byte strides, whole-element strides, natural element alignment, and valid accessed row ranges. SIMD alignment is not required, and legal extra stride cannot change results. Memory/numerical errors do not return partially calculated arrays as success.

## 8. Precision and determinism

- **Negative rounding has a direction.** Phase splitting/predictor scaling use their stated floor; chroma conversion and selected parameter/weight operations use truncation.
- **Errors are not universally normalized.** Integer errors retain sample scale, floating errors encode per plane, and designated parameters receive depth/area scaling.
- **Every floating rounding point matters.** Sample errors use binary32; specified prediction/parameter operations use binary64. Use nearest-even per operation without reassociation or fused multiply-add.
- **Cost is not exported error.** Seed costs differ from ordinary Cq. A winner's new error cannot replace the fixed predictor's inherited error.
- **Ties preserve priority.** Strict replacement, candidate order, and seed order all matter.
- **Processing order differs from storage order.** Meander changes dependencies, while exported arrays remain X-fastest row order.
- **Intermediate integers must not overflow.** Areas, scaling, squared distances, costs, and addresses use sufficiently wide representable arithmetic. Packing's explicit modulo rule does not authorize overflow elsewhere.
- **Determinism follows dependencies.** Candidate eligibility and comparison priorities define results independently of cache layout or evaluation count. Independent frames/starts may run in different orders but must not share mutable intermediate state that changes results.

[Back to the English index](README.md)
