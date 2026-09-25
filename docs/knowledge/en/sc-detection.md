# SCDetection: scene flags from block errors

## 1. What the function computes

`SCDetection` reads neither reference images nor fresh motion measurements. It compares stored block errors against two thresholds: first classifying bad blocks, then checking whether there are enough of them.

Output preserves clip pixels and replaces one directional scene property. Analysis precision comes from vector metadata and need not match output clip depth.

Block error here means the stored `AnalysisSAD`, which may contain SAD, SATD, or DCT luma error plus enabled chroma SAD. This function does not recompute pixel SAD or convert between metrics; its existing threshold and scaling formulas apply to the stored value. See [analysis data](shared/analysis-data.md).

## 2. Objects and notation

Blocks are `Bx,By`, grid `Nx,Ny`, total `N=NxNy`, chroma ratios `rx,ry`, analysis precision b. T1 is the block-error threshold, T2 the bad-block-count threshold, and K the actual bad-block count.

## 3. Overall calculation

At creation, read vector frame 0's descriptor and establish thresholds. Per frame, validate data state and descriptor consistency, count bad blocks in complete arrays, then write the property selected by the current delta direction.

## 4. Step-by-step calculation

### 4.1 Scaling the block threshold

`thscd1` uses 8-bit, `8×8` reference units. Convert to the analysis scale:

$$A=B_xB_y/64,\quad C_f=\begin{cases}1+2/(r_xr_y)&\text{with chroma}\\1&\text{otherwise}\end{cases},$$
$$D=(2^{\min(16,b)}-1)/255,\quad T_1=\lfloor thscd1\,((AC_f)D)+0.5\rfloor.$$

Divisions are binary64 real divisions; area is first computed exactly. The chroma factor includes the area of both chroma planes. Float32 analysis uses D=257 because its errors are encoded on a 65535 scale, not maximum sample value 1.

### 4.2 From percentage to a count threshold

Convert `thscd2` to binary32, requiring a finite value in 0–100. Promote it to binary64, evaluate `((thscd2·Nx)·Ny)/100` in order, then round to binary32 T2.

T2 is not rounded to an integer count. Four blocks and 51% give binary32 2.04, not 2; 51 is not interpreted on a 0–255 scale.

### 4.3 Two strict comparisons

$$K=\sum_i\mathbf1_{SAD_i>T_1},\qquad change=\mathbf1_{binary32(K)>T_2}.$$

Error equal to T1 is not bad; count equal to T2 is not a scene change. SAD denotes the exported raw error, which may include luma SATD; this function does not infer its generating metric.

### 4.4 Choosing the direction

Current effective `DeltaFrame>0` writes `_SceneChangeNext=[change]`; otherwise write `_SceneChangePrev=[change]`. These keys have no prefix. Preserve the other direction's property, including its type and count. Other properties come from `clip[n]`, not the vector frame.

## 5. A complete numerical example

GRAY8, `8×8` blocks, a `2×2` grid, `thscd1=400,thscd2=50` give `A=Cf=D=1,T1=400,T2=2`.

Errors `[400,401,900,0]` have K=2, so change=0. Replacing the first value by 401 gives K=3 and change=1.

With delta 2 and clip properties Prev=`[7]`, Next=`[9,8]`, the second case keeps Prev=`[7]`, replaces Next with `[1]`, and preserves pixels.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,vectors` | Required nodes | Visible output and analysis data |
| `thscd1` | 400; int64 0–16320 | Per-block error threshold |
| `thscd2` | 51.0; finite binary32 in 0–100 | Bad-block percentage |
| `prefix` | `MVUtensils` | Analysis-property lookup only |

## 7. Boundaries, missing data, and errors

Creation needs valid metadata, but not arrays on frame 0. Per-frame invalid metadata or valid metadata without usable arrays yields change=1. Negative errors, out-of-range vectors, or other complete-array corruption fail; see [shared analysis data](shared/analysis-data.md).

Block size, grid, boolean chroma flag, ratios, and precision must match creation values. A valid but changed descriptor fails even if arrays are unavailable. Invalid metadata follows the unavailable branch.

No check is made that `n+delta` exists. Externally supplied complete data can be classified at sequence edges. `thscd2=100` prevents complete arrays exceeding the threshold but does not suppress change=1 from missing data.

## 8. Precision and determinism

Both strict comparisons, binary32 percentage conversion, and the fractional count threshold affect boundary decisions. Results depend on the current frame's data and fixed descriptor, not previous classifications.

[Back to the English index](README.md)
