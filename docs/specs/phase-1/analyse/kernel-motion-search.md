# Candidate cost and motion refinement

## Input, output and selection

Inputs: initial result (v,c,s), fixed predictor u, nonnegative integer lambda, penalty q in [0,256], rectangular integer candidate domain Omega, search type 0..5, positive integer range a, an optional initial cross/G centre for search=3 (default initial v), and an error evaluator returning eY(w),eC(w) from [block error](kernel-block-error.md). Output: a selected vector, comparison cost and raw error. v is a position; c is not necessarily equal to s.

$$D(w)=\left\lfloor\frac{\lambda((w_x-u_x)^2+(w_y-u_y)^2)}{256}\right\rfloor,$$
$$C_q(w)=D(w)+e_Y(w)+e_C(w)+\left\lfloor\frac{q e_Y(w)}{256}\right\rfloor+\left\lfloor\frac{q e_C(w)}{256}\right\rfloor.$$

For an ordered candidate list A, selection B(A;v,c,s) means: exclude points outside Omega, and retain the first item among the current result and eligible candidates with the smallest comparison cost. A candidate replaces the current result only when Cq(w)<c; its new raw s=eY(w)+eC(w). Ties preserve the existing result. This defines mathematical priority, not a required memory traversal or program structure.

Every candidate offset below is relative to the named fixed centre. A centre changes only at an explicitly defined recurrence. Before this relation is invoked, its caller must establish the [whole-domain sampling precondition](kernel-block-error.md#sampling-admissibility-and-geometry-errors) over all of Omega and the supplied initial position, including an initial position outside Omega. Omega alone controls ordinary candidate eligibility: there is no additional per-candidate sampling mask. Failure of sampling admissibility is an error for the supplied geometry, not a skipped candidate or an infinite candidate cost. No search traversal is started to discover whether the failure would matter. Do not replace comparison cost with exported SAD. Arithmetic must not overflow. Output/scratch do not alias the error inputs; different calls do not share mutable state.

## Square ring and one-axis searches

Define Q(o,r,t), r,t positive, by the following ordered offsets from centre o:

1. For i=-r+t,-r+2t,... strictly below r, in increasing order: (i,-r), then (i,r).
2. For j from the same sequence: (-r,j), then (r,j).
3. Corners (-r,-r),(-r,r),(r,-r),(r,r).

For search=1, take Q(o,1,1) through Q(o,a,1), with o the initial v. For search=4, take (-1,0),(1,0),...,(-a,0),(a,0) from o. For search=5 transpose X/Y, visiting up before down. These lists do not drift with intermediate improvements.

## Hexagonal refinement: search=2

Let h0..h5 be (-2,0),(-1,2),(1,2),(2,0),(1,-2),(-1,-2), indexed modulo 6.

If a=1, apply Q(o,1,1) once to the initial result and return that result; no second ring is applied. For a>1 compare o+h0 through o+h5. If none improves the result, z=o. If direction j wins, z=o+hj. After that first win, perform at most max(0,floor(a/2)-1) recurrences: compare z+h(j-1),z+hj,z+h(j+1); stop if none improves, otherwise replace z and j with the winning position/direction. Preserve the fixed u and lambda throughout.

For a>1 finally apply Q(z,1,1) about the resulting hexagonal position, retaining its current cost and raw error. Parameter a is not a hard maximum displacement from the initial centre.

## Multiscale refinement: search=3

From fixed initial centre o, compare horizontal offsets at positive odd distances strictly less than a, left before right; then vertical offsets at those distances, up before down. Append o+kG for k=1 through max(1,floor(a/4)), where G has this order:

$$(-4,2),(-4,1),(-4,0),(-4,-1),(-4,-2),$$
$$ (4,-2),(4,-1),(4,0),(4,1),(4,2),$$
$$ (2,3),(0,4),(-2,3),(-2,-3),(0,-4),(2,-3).$$

Apply the search=2 relation with range a to the selected result. Even a<4 includes one G group; only Omega excludes candidates outside the valid field. The caller may provide a fixed centre different from initial v for the initial cross/G lists; the subsequent hexagonal relation still begins at the selected result. Analyse uses this form for bad-block expansion centred on zero.

## Logarithmic neighbourhood refinement: search=0

Use the positive step sequence a,floor(a/2),floor(a/4),... ending at 1. Each step has its own direction hint h, initially the symbol * (unrestricted). Otherwise h is one of the eight unit axial/diagonal vectors. The hint is a mathematical dependency between successive selections at the same step; it is discarded when the step becomes smaller. The result triple, fixed predictor u and cost parameters are preserved between steps.

Define the ordered axial list

$$A=[(1,0),(-1,0),(0,1),(0,-1)].$$

Let A(*)=A. For a vector hint h, A(h) is the subsequence of A satisfying e dot h > 0, preserving A's order. Thus an axial hint admits only that axis direction, and a diagonal hint admits its two component directions. Define the ordered perpendicular pair

$$P(e)=\begin{cases}[(0,1),(0,-1)]&e_x\ne0,\\[(1,0),(-1,0)]&e_y\ne0.\end{cases}$$

The ordered diagonal fallback list F(h) is:

| h | F(h), in priority order |
| --- | --- |
| * | (1,1), (-1,1), (1,-1), (-1,-1) |
| (1,0) | (1,1), (1,-1) |
| (-1,0) | (-1,1), (-1,-1) |
| (0,1) | (1,1), (-1,1) |
| (0,-1) | (1,-1), (-1,-1) |
| (1,1) | (1,1), (-1,1), (1,-1) |
| (-1,1) | (1,1), (-1,1), (-1,-1) |
| (1,-1) | (1,1), (-1,-1), (1,-1) |
| (-1,-1) | (-1,-1), (-1,1), (1,-1) |

For a fixed step d, one recurrence is the following relation on the retained result R=(v,c,s) and hint h. Let o=v at the start of this recurrence.

1. Let R1=B(o+d A(h);R), where adding a centre and multiplying by d apply to each list element. The whole axial list has the fixed centre o.
2. If R1 has strictly smaller comparison cost than R, let e=(position(R1)-o)/d, necessarily axial, and fix t=position(R1). Let R2=B(t+d P(e);R1). If this perpendicular pair strictly improves R1, the next hint is (position(R2)-t)/d, the winning perpendicular unit vector alone. Otherwise the next hint is e. The recurrence's result is R2. In this case F(h) is not included: diagonals relative to o are eligible only insofar as they occur in the pair centred on the winning axial position t.
3. If the axial list does not improve R, instead let R2=B(o+d F(h);R), with fixed centre o and the same incoming hint h. If a diagonal improves R, its offset (position(R2)-o)/d is the next hint. Otherwise this step is complete.

After an improving recurrence, use its result as the new centre and its specified next hint for another recurrence at the same d. After a non-improving recurrence, retain the result and advance to the next smaller step, resetting h to *. Return when step 1 is complete. A successful axial selection followed by a non-improving perpendicular pair still counts as an improving recurrence. A successful perpendicular selection carries only its perpendicular direction into the next recurrence, not the sum of the axial and perpendicular displacements.

All selections use the same Cq, fixed u and strict comparison rule B. Equal-cost candidates never replace the retained result; among strictly improving equal-cost candidates, the first eligible position in the relevant ordered list wins. Exclude candidates outside Omega in every list without changing any list's centre, order or continuation rule. Keep a supplied, safely sampleable initial position even when it is outside Omega; do not clamp or reject it here. Its retained cost competes normally, all newly selected candidates must belong to Omega, and failure to find an eligible strict improvement may leave that initial position unchanged. Sampling admissibility remains the caller's precondition above.

Each continuing recurrence strictly lowers a nonnegative integer cost, so the relation terminates. This relation specifies candidate eligibility and mathematical priority; it does not require any particular evaluation count, storage layout or implementation traversal. The range a selects the initial step, not a hard displacement radius.

### Distinguishing cost examples

For these examples use a=1, q=lambda=0, eC=0, initial position (0,0) with cost/raw error 9, and a domain containing all named points unless stated otherwise. Unlisted points have error 9. Each row is a separate error field.

| Nondefault errors | Final position, raw error | Distinction |
| --- | --- | --- |
| E(1,0)=4; E(-1,1)=2 | (1,0), 4 | An improving axial selection restricts the subsequent diagonal choices; (-1,1) is not eligible in the perpendicular pair. |
| E(1,0)=4; E(1,1)=2; E(2,1)=1 | (1,1), 2 | After the perpendicular improvement the next hint is (0,1); the lower error at (2,1) is not automatically reachable. |
| E(1,0)=E(-1,0)=E(0,1)=E(0,-1)=4 | (1,0), 4 | Axial ties follow A's order. |
| E(1,0)=5; E(1,1)=E(1,-1)=3 | (1,1), 3 | Perpendicular ties prefer positive Y after a horizontal winner. |
| E(1,1)=E(-1,1)=4 | (1,1), 4 | With no axial improvement, the unrestricted diagonal list decides the tie. |
| E(1,-1)=7; E(2,0)=E(0,-2)=E(2,-2)=3 | (2,0), 3 | The (1,-1) diagonal hint has the exact fallback order listed above; it is not replaced by a rotationally symmetric tie rule. |
| E(1,0)=9, with every other error also 9 | (0,0), 9 | Equal cost never moves the result. |

Boundary examples, still with a=1 and all unlisted errors 9:

- Omega=[-1,1) x [-1,2), initial (0,0), cost 9; E(1,0)=0 and E(-1,0)=4. The excluded right-hand position cannot win; output is (-1,0),4, on the inclusive lower X bound.
- Omega=[0,3) x [0,3), safely sampleable initial (-1,0), cost 5; E(0,0)=4 and E(0,1)=3. Output is (0,1),3. If instead every in-domain error is 9, the output remains the supplied (-1,0),5 outside Omega. The latter does not authorize any out-of-domain refinement candidate.

### Image examples

Use GRAY8, 32x24, five frames, I(n,x,y)=(((x+n)*17+y*29) mod 192)+16; Super uses blksize=[8], overlap=[0], pad=[16], pel=1, onelevel=true. Observe frame 2, first block. Analyse uses delta=1, search=0, mvlambda=0, pnew=0, chroma=false, all other controls at their defaults except pelsearch=a. For Recalculate use the same Super and a public input field with matching Analysis geometry, delta=1, all 12 vectors zero and all 12 old errors zero; use thsad=0, smooth=false, search=0, searchparam=a, mvlambda=0, pnew=0, chroma=false, fields=false, satd=false.

| a | Analyse (vx,vy,SAD) | Recalculate (vx,vy,SAD) |
| --- | --- | --- |
| 1 | (-1,0,294) | (-1,0,294) |
| 2 | (-1,0,294) | (-1,0,294) |
| 3 | (-1,0,294) | (-1,0,294) |
| 4 | (8,8,64) | (8,8,64) |

For a=2, the first axial minimum is E(-2,0)=1584, improving E(0,0)=1878. Its perpendicular choices E(-2,2)=4410 and E(-2,-2)=4622 do not improve it. The lower E(2,-2)=1514 is not in that pair. Step 2 ends at (-2,0); step 1 then selects (-1,0),294.

The following Recalculate examples use two GRAY8 frames: frame 0 is all zero, and frame 1 is the reference image described below. Super uses blksize=[4], overlap=[0], pel=1, onelevel=true. The external public vector field has matching geometry, delta=1, all old SAD values zero and every vector set to the stated seed. Recalculate uses thsad=0, smooth=false, search=0, searchparam=1, mvlambda=0, pnew=0, chroma=false, meander=false, fields=false, satd=false. Observe frame 0, whose reference is frame 1 under delta=1. Observed block coordinates are zero-based block-grid indices: with these 4x4 blocks and zero overlap, block (i,j) has unpadded pixel origin (4i,4j). Every error is the sum of 16 reference samples, making the listed costs directly checkable.

For the two patch fixtures, the reference is 16x16, pad=16 and seed=(4,4). All reference pixels are 100 except the 6x6 patch at x=3..8,y=3..8, listed row by row. Observe block (0,0).

| Fixture | Patch rows | Output (vx,vy,SAD) |
| --- | --- | --- |
| Axial/diagonal competition | [11,5,15,20,5,15]; [8,5,7,20,11,20]; [9,11,16,19,3,9]; [19,19,19,11,3,0]; [18,15,11,19,17,20]; [2,11,8,9,7,13] | (4,3,189) |
| Diagonal tie | [0,10,0,0,10,0]; [10,10,0,0,10,10]; [0,0,0,0,0,0]; [0,0,0,0,0,0]; [10,10,0,0,10,10]; [0,10,0,0,10,0] | (5,5,30) |

The competition seed costs 206. In A's order its axial costs are [205,226,198,189]; in F(*)'s order its diagonal costs are [184,216,193,214]. The lower diagonal error 184 does not override the improving axial choice and its perpendicular pair. For the diagonal-tie fixture the seed and all four axial positions cost 40, and all four diagonals cost 30, selecting the first diagonal.

The remaining fixtures have identical reference rows, specified in full below:

| Fixture | Size; pad; seed; observed block | Reference row | Output (vx,vy,SAD) |
| --- | --- | --- | --- |
| Axial tie and equal-cost retention | 16x16; 16; (4,4); (0,0) | [100,100,100,0,10,0,0,10,0,100,100,100,100,100,100,100] | (5,4,40) |
| Exclusive upper bound | 8x8; 1; (0,0); (1,1) | [100,100,100,100,10,0,0,0] | (0,0,40) |
| Inclusive lower bound | 8x8; 1; (0,0); (0,0) | [0,10,10,10,100,100,100,100] | (-1,0,80) |

In the axial-tie fixture the seed costs 80 and both horizontal neighbours cost 40; the positive-X neighbour wins, and equal-cost vertical alternatives do not move its Y coordinate. In the upper-bound fixture Omega's upper X bound is 1, exclusive: displacement (1,0) would have error 0 under edge extension but is ineligible. In the lower-bound fixture the lower X bound is -1, inclusive, and that displacement improves the seed's error 120 to 80.

## Examples

Unless stated otherwise set q=lambda=0, eC=0, and include all named points in Omega.

- Initial centre (0,0) has cost 9; four axial neighbours each have error 4, all others 9. search=1,a=1 selects (0,-1), the first ring entry. An initial cost of 4 instead keeps (0,0).
- With left/right errors 4, up error 0 and initial error 9, search=4,a=1 chooses left; search=5 chooses up.
- search=2,a=2: only hexagonal point (2,0) improves from 9 to 3; only (2,-1) in its final ring improves to 1. Output is (2,-1), raw SAD=1.
- u=(0,0),lambda=256,q=0: positions (1,0),(2,0), each raw error 5, cost 6 and 9. From initial cost 8 only (1,0) improves; exported SAD remains 5.
- search=0,a=1: errors 9 initially, 4 at (1,0), 2 at (1,1), 9 elsewhere produce (1,1), through an axial selection followed by its perpendicular pair. The next recurrence has no improvement.
- search=3,a=1 with only (-4,2) having error 0 and other positions 9 selects (-4,2), despite the small range parameter.

Consumers: Analyse block composition and Recalculate refinement.
