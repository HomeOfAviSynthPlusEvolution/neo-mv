# Cumulative transforms and correction maps

Input is a decoded motion interval [b,h], aspect a and center (cx,cy). Output C is a sequence of six-coefficient backward sampling maps. Reuse [motion conversion and composition](../../phase-5/depan-analyse/kernel-motion-transform.md), with forward=true and time fraction=1. Write M(m) for that conversion and Z(z)=M(0,0,0,z). Z retains the conversion's z<=0 replacement by 1 before logarithm; it does not clamp a finite negative adaptive scale to a small positive scale.

$$C_b=I,\qquad C_j=M(m_j)\circ C_{j-1}\quad(b<j\leq h).$$

The rightmost map is applied first. All six coefficients are independently rounded; do not restore h=u or w=-v*a*a after a composition. No tuple at b is composed, even when it is valid. Tuples used for j>b must be valid after interval selection.

## Stabilization inverse

For this phase, Inv is the six-input extension of the Phase 5 analysis inverse: use its displayed equations and operation order on tx,ty,u,v,w, **without requiring h=u**. The input h does not enter those equations and the output h' equals u'. Rounded cumulative maps can have different h and u; that alone is not an error. This is a defined similarity-model inverse, not the generic inverse of an arbitrary affine matrix.

Explicitly, let e=sqrt((-w)/v) if v!=0, otherwise e=1. Require a nonnegative argument and nonzero divisors. Then

$$D=(u u)+(((v v)e)e),\quad u'=u/D,\quad h'=u',$$
$$v'=((-u')v)/u,\quad w'=((-v')e)e,$$
$$t_x'=((-u')t_x)-(v't_y),\quad t_y'=((-w')t_x)-(u't_y).$$

Require u!=0 even for an otherwise invertible quarter-turn. All required intermediates must be finite. A failure is a frame error.

Given smoothed S, the raw correction is Q=S composed with Inv(Cn). Method 0 then performs the specified end taper and limits unless b=n. Its S, raw Q and raw motion conversion can carry non-finite values under [inertial numerical recovery](kernel-numerical-recovery.md); this does not relax cumulative-map construction or Inv's checked domain above. The recovered final Q must be finite. Method 1 converts Q to motion and back using M, with no taper or limits. Never replace this round trip by retaining Q: it can change coefficients and sampling class.

## Examples

- With b=0 and translation tuples dx=[unused,1,2],dy=0,rotation=0,zoom=1, C has translations 0,1,3. S=I at n=2 yields raw correction translation -3.
- Direct maps C=(tx=2,ty=0,u=1,v=0,w=0,h=1.0000001192092896) and S=I are admitted. Inv uses u and returns translation -2 and unit diagonal, ignoring C's h difference.
- For direct C(x,y)=(2x,2y),S(x,y)=(x+3,y), the correction is (0.5x+3,0.5y), not (0.5x+1.5,0.5y).
