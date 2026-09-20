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

Use the positive step sequence a,floor(a/2),floor(a/4),... ending at 1. At each step d, let o be the current result's position. Define the ordered neighbourhood

$$N_d(o)=o+d\,[(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)].$$

Apply B to this entire neighbourhood using the fixed centre o, current result, unchanged predictor u and penalty parameters. If the result improves, repeat at the same d with the winning position as the new centre. Otherwise advance to the next smaller d. After d=1 has no improving neighbour, return the result. Candidates outside Omega remain excluded.

The neighbourhood is independent of the direction of previous improvements. Only strictly lower nonnegative integer costs are accepted, so each fixed-step recurrence terminates. The ordered offsets define tie priority, not a prescribed implementation traversal.

## Examples

Unless stated otherwise set q=lambda=0, eC=0, and include all named points in Omega.

- Initial centre (0,0) has cost 9; four axial neighbours each have error 4, all others 9. search=1,a=1 selects (0,-1), the first ring entry. An initial cost of 4 instead keeps (0,0).
- With left/right errors 4, up error 0 and initial error 9, search=4,a=1 chooses left; search=5 chooses up.
- search=2,a=2: only hexagonal point (2,0) improves from 9 to 3; only (2,-1) in its final ring improves to 1. Output is (2,-1), raw SAD=1.
- u=(0,0),lambda=256,q=0: positions (1,0),(2,0), each raw error 5, cost 6 and 9. From initial cost 8 only (1,0) improves; exported SAD remains 5.
- search=0,a=1: errors 9 initially, 4 at (1,0), 2 at (1,1), 9 elsewhere produce (1,1), selected directly from the first eight-point neighbourhood. Its next neighbourhood has no improvement.
- search=3,a=1 with only (-4,2) having error 0 and other positions 9 selects (-4,2), despite the small range parameter.

Consumers: Analyse block composition and Recalculate refinement.
