# Cross-level and spatial vector prediction

Inputs: parent row-major vector/error grid, parent and child pel pP,p, block/overlap geometry, child grid, optional parent global vectors, current block position/direction, already determined neighbours and candidate domain Omega. Outputs: initial child triples, global predictor and spatial predictor triples. A triple is (vx,vy,s); s is retained confidence data, not necessarily the newly measured error at that position.

All coordinates and weighted sums use exact sufficiently wide integers until an explicit binary64 conversion. All binary64 operations round individually to nearest-even. Predictor coordinates may be negative. Inputs are immutable; output cannot alias parent data. A spatial query reads only the defined initial/final grid values described below, not uninitialized future blocks.

## Parent-grid interpolation

For child block (k,j), clamp k,j to [0,2NxP-1],[0,2NyP-1], yielding i,t. Let dx=2(i mod 2)-1,dy=2(t mod 2)-1 and A=(floor(i/2),floor(t/2)). Select four parent triples V1..V4:

| Location | V1,V2,V3,V4 |
| --- | --- |
| Both i and t at their first/last boundaries | A,A,A,A |
| Only i at its boundary | A,A,A+(0,dy),A+(0,dy) |
| Only t at its boundary | A,A,A+(dx,0),A+(dx,0) |
| Neither at a boundary | A,A+(dx,0),A+(0,dy),A+(dx,dy) |

Here A+offset denotes a grid lookup, not adding to vector components. NxP,NyP must be positive. Clamped border cases make the indicated lookups valid even for a one-block parent axis.

With zero overlap, form each vector numerator U=9V1+3V2+3V3+V4. Form the SAD numerator with the same weights, then add 8.

With overlap on either axis, let sx=Bx-Ox,sy=By-Oy, ax=3Bx-2Ox if dx>0 else 3Bx-4Ox, ay similarly, bx'=4sx-ax,by'=4sy-ay. Weights are ax*ay,bx'*ay,ax*by',bx'*by'. For each component, form the exact integer weighted sum; convert it to binary64, multiply by the separately rounded binary64 reciprocal 1/(sx*sy), and truncate toward zero to obtain its numerator. No extra 8 is added to SAD in this case.

Let r=3-log2(p)+log2(pP). Each child vector component is

$$\left\lfloor U/2^{\max(r,0)}\right\rfloor 2^{\max(-r,0)}.$$

Child predictor SAD is floor(SAD numerator/16). The coordinate division is floor, including for negative values. The coarsest grid instead begins entirely with (0,0,0).

## Global prediction

Use all final parent vectors. Independently select the most frequent X value and the most frequent Y value. For either component, break a frequency tie by the smallest numerical value. Call the modes mx,my. This definition depends only on the vector multiset, not its storage order or the number of distinct values.

Let I be the set of parent block indices i for which abs(vx[i]-mx)<6 and abs(vy[i]-my)<6. If nonempty, the next-level global position is

$$g=\left(\operatorname{trunc}\frac{2\sum_{i\in I}v_{x,i}}{|I|},\ \operatorname{trunc}\frac{2\sum_{i\in I}v_{y,i}}{|I|}\right);$$

otherwise it is (2mx,2my). Repeated equal vectors at different block indices contribute separately to both sums and the count. A disabled global predictor is (0,0). On entering the next level, multiply both components by that level's p and add the field shift f to Y. The coarsest level's incoming global position is zero before this field adjustment.

## Spatial predictors

Blocks are determined top to bottom. With meander, even rows run left to right and odd rows right to left; otherwise all rows run left to right. A prior block holds its final triple; a not-yet-determined block holds its initial parent-interpolated triple. Let sigma=+1 for rightward rows, -1 for leftward rows.

- P1: (bx-sigma,by), or (0,f,0) if absent.
- P2: (bx,by-1), or (0,f,0) if absent.
- P3: (bx+sigma,by+1) if present, otherwise (bx+sigma,by-1), otherwise (0,f,0).

Clamp each predictor's X/Y to Omega's integer bounds, preserving its SAD. For by>0, P0 has the componentwise median coordinates of P1/P2/P3 and max(s1,s2,s3). For by=0, P0=P1. For every block, obtain G by clamping the same immutable layer-global position to that block's Omega. Clamping for one block must not change another block's incoming global position.

## Examples

- No overlap, interior parent X values -1,-2,0,1, pP=1,p=2: U=-14,r=2, so predicted X=-4, not -3. Parent SADs 1,2,3,4 give floor((9+6+9+4+8)/16)=2.
- Parent vectors (2,0),(0,0) have tied X modes; mx=0,my=0. Both are within 6, so next global position is (2,0). With (10,0),(0,0), only (0,0) lies near the selected mode and the result is (0,0). Reordering the parent vectors changes neither result.
- Parent vectors (0,0),(0,0),(3,0) contribute three entries to I and give next-level global position (2,0); equal vector values are not deduplicated.
- A layer-global X of 10 clamped for consecutive blocks with X intervals [-4,5) and [-8,12) gives 4 and 10 respectively. The first clamp does not turn the second result into 4.
- A spatial triple may keep SAD=100 after its X coordinate is clamped. This confidence value is not silently replaced by a fresh block error.

Consumer: [analysis composition](kernel-analysis.md).
