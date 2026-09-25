# DepanAnalyse: iterative fitting of global motion

## 1. What the function computes

`DepanAnalyse` treats block displacements as observations, fits translation, scale, and rotation with a bounded iteration, and exports five Depan properties. Clip pixels do not enter fitting and remain unchanged with info=false.

This is not an arbitrary-precision least-squares solve. Initialization, step sizes, reweighting, and stopping all affect results.

Block error here means the stored `AnalysisSAD`, which may contain SAD, SATD, DCT, or mixed luma error plus enabled chroma SAD. This function does not recompute pixel SAD or convert between metrics; its existing threshold and scaling formulas apply to the stored value. See [analysis data](shared/analysis-data.md).

## 2. Objects and notation

Creation vector delta d must be ±1. For d=+1, output n reads vectors[max(0, n−1)]; for d=−1, vectors[n]. Prefix is fixed to `MVUtensils`; there is no prefix argument.

Aspect is `a=pixaspect/(fields?2:1)`, center `cx=fl32(clip width)/2,cy=fl32(clip height)/2`. Each observation has block center Xi, Yi, pixel displacement dxi, dyi, raw SAD, and mask base weight mi.

## 3. Overall calculation

Save creation scene thresholds and d; decode using the current vector frame's descriptor; create centers/displacements; initialize model/weights; iterate updates and reweighting; judge residual strictly; convert directly or invert according to d; apply final field shift and write properties.

## 4. Step-by-step calculation

### 4.1 Observations and admission

Compute and save T1, T2 through [SCDetection](sc-detection.md) at creation. Later frames may have different valid grids, depth, and descriptors. Validate counts/vector bounds against current geometry, but compare errors/counts with saved T1, T2 without rescaling.

Invalid metadata or missing arrays skips fitting; complete malformed arrays fail. Saved d determines frame selection/conversion direction. A changed current delta, even missing and decoded as zero, does not replace it.

Centers are `Xi=bx(Bx-Ox)+floor(Bx/2)` and similarly Y, without padding. Displacements are `fl32(fl32(vx)·fl32(1/p))`, using current analysis p.

Optional mask is an integer8 image with clip dimensions, sufficient frames, and a full-resolution first plane. mi is the byte at an in-range block center, otherwise 1; without mask, mi=1. Value 128 means weight 128, not 128/255. Initialize bi=mi without border or bad-block exclusion before the first update.

### 4.2 One update

Initialize `tx=ty=v=w=0,u=h=1,E=2·error,iter=0`. Residuals under the old map are:

$$e_{xi}=(((t_x+uX_i)+vY_i)-X_i)-dx_i,$$
$$e_{yi}=(((t_y+wX_i)+hY_i)-Y_i)-dy_i.$$

Accumulate N, X2, Y2, R in block-row order, each starting at binary32 0.1. Add bi,`fl32(Xi²)bi`,`fl32(Yi²)bi`, and `(exi²+eyi²)bi` respectively. The 0.1 remains even with zero weights.

Accumulate gradient numerators from +0 in the same order:

$$g_x=\frac{\sum(2e_x)b}{2N},\quad g_y=\frac{\sum(2e_y)b}{2N},$$
$$g_{xx}=\frac{\sum(2X)e_xb}{(2X2)1.5},\quad g_{yy}=\frac{\sum(2Y)e_yb}{(2Y2)1.5},$$
$$g_{xy}=\frac{\sum(2Y)e_xb}{(2Y2)3},\quad g_{yx}=\frac{\sum(2X)e_yb}{(2X2)3}.$$

When scale is disabled, xx/yy numerators remain zero and their terms are not evaluated; likewise xy/yx with rotation disabled. Zero-numerator divisions still run. Numerator products associate left to right; Xi² and 2Xi are computed as exact integers first.

For step s, set `E'=sqrt(R/N)` and update simultaneously:

$$t'_x=t_x-sg_x,\quad t'_y=t_y-sg_y,$$
$$u'=u-(s\cdot0.5)(g_{xx}+g_{yy})\quad\text{when scale is enabled},\quad h'=u',$$
$$v'=v-(s\cdot0.5)(g_{xy}-g_{yx}/(a^2)),\quad w'=((-a)a)v'.$$

Disabled scale leaves u unchanged. The v update still executes with zero gradients when rotation is disabled. E' measures the old map, not a fresh residual of the updated map.

### 4.3 Reweighting after an update

Check in this order; the first failure sets bi=+0 and skips later arithmetic:

1. Without a mask, exclude four block rows/columns at every edge. With a mask, omit this exclusion.
2. SAD>T1.
3. With all eight neighbors present, absolute dx difference from their mean exceeds wrong.
4. Repeat for dy.
5. Absolute horizontal residual under the new model exceeds G.
6. Absolute vertical residual exceeds G.

Neighbor order is upper-left, up, upper-right, left, right, lower-left, down, lower-right. Use original observations, not their weights. Incomplete neighborhoods skip both mean checks. Passing zero-displacement observations receive zerow·mi; others mi.

Every round reconsiders original observations; rejection is not permanent. Finite negative wrong/zerow are allowed and follow ordinary comparisons/arithmetic; invalid R/N or other numerical results fail.

### 4.4 Schedule and stopping

First five updates k=0…4 use s=0.3, translation only, each followed by reweighting with G=1000. They do not stop early for small E.

For k=5…99, enable scale/rotation as requested. Use s=0.3 before k=8, 0.6 for 8≤k<10, then 1. After updating, stop if `(Eold−E'<0.005 and k>9)` or E'<0.01, retaining the updated model and iter=k without another reweight. Otherwise reweight with G=2E'. Exhaustion gives iter=100.

### 4.5 Validity and output motion

Validity is solely E<error. Unavailable input retains initial E=2error and identity, then still makes this comparison. Negative error can therefore admit this branch; no extra “fitting ran” requirement is added.

Invalid output is `(0,0,0,1,0)`. Valid d=−1 converts the model with forward=true. Valid d=+1 first applies the [specific inverse](shared/global-motion.md), then converts with forward=false, rather than merely negating dx/dy.

For valid motion with fields=true, get parity from clip[n] or tff; add 1 to dy for top, subtract 1 for bottom. Do not read carrier parity. info=true writes diagnostics and invokes text rendering; other properties come from clip[n].

## 5. A complete numerical example

Clip `16×16`, old grid `2×2` of `8×8` blocks, all vectors/SAD zero, d=−1, zero mask with all centers in range, fields=false, error=15.

All weights remain zero. Every update has N=X2=Y2=R=0.1, zero gradients, identity model, E=sqrt(1)=1. The first five updates run, and stopping occurs at k=10 when improvement is insufficient. iter=10 and 1<15, so valid identity motion is exported; pixels remain unchanged.

error=1 instead fails the strict comparison and exports invalid motion. Zero weights alone do not imply invalidity: regularizing initial values, stopping, and final comparison jointly decide.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,vectors` | Required; constant clip geometry/format, sufficient vectors | Output/observations; clip can be RGB/float |
| `mask` | Omitted | Integer8 base weights and border-exclusion mode |
| `zoom,rot` | true, true | Terms enabled after the first five updates |
| `pixaspect` | 1.0; finite positive | Model aspect |
| `error` | 15.0; finite | Initialization and strict acceptance |
| `wrong` | 10.0; finite | Neighbor-outlier threshold |
| `zerow` | 0.05; finite | Zero-displacement weight multiplier |
| `thscd1,thscd2` | 400, 51.0 | Saved scene thresholds |
| `fields,tff` | false, omitted | Half aspect and final dy correction |
| `info` | false | `DepanAnalyse_info` and rendering |

## 7. Boundaries, missing data, and errors

There is no unconditional invalid frame-zero rule: d=+1, n=0 still reads vectors[0]. Supplied mask[n] must be obtainable even if the field is unavailable, although fitting samples are not read then. a² must be finite/nonzero, divisors nonzero, and R/N nonnegative. Numerical failure is an error, not ordinary invalid motion.

## 8. Precision and determinism

Updates retain binary32 order and old-state dependencies. Stopping keeps the new map but E describes the old residual. Center arithmetic in motion conversion rounds too, so exported dx need not be bitwise equal to model tx.

[Back to the English index](README.md)
