# Stabilization: temporal smoothing and correction limits

This article expands both [DepanStabilise](../depan-stabilise.md) methods. Write a sampling map as T=(tx, ty, u, v, w, h), meaning X=tx+ux+vy, Y=ty+wx+hy. See [global motion](global-motion.md) for conversion and composition order. Real operations round separately to binary32; composed coefficients remain independent.

## Constants derived from parameters

For frame-rate numerator/denominator N, D and user initzoom zuser:

$$fps=\operatorname{fl32}(N)/\operatorname{fl32}(D),\quad a=pixaspect/(fields?2:1),$$
$$c_x=\operatorname{fl32}(W)/2,\quad c_y=\operatorname{fl32}(H)/2,\quad z_0=1/z_{user}.$$

All must be positive and finite. Initial sampling scale is the reciprocal of user zoom. Effective zoom limit is:

$$L_z=\begin{cases}\max(zoommax,z_{user})&zoommax>0,\\-\max(-zoommax,z_{user})&zoommax\le0.\end{cases}$$

Thus zoommax=0 creates a negative limit rather than disabling it.

Temporal radii are:

$$r=\max(1,\operatorname{trunc}(fps/(4\,cutoff))),\quad
r_z=\min(r,\operatorname{trunc}((fps\,tzoom)/4)).$$

Quotients must be finite and their truncations fit int32; r≤2147483646. Both methods calculate these shared parameters, without silently capping large windows.

Smoothing weights are w(i)=cos(((i·0.5)π)/r) for 0≤i<r, with w(r)=0 explicitly. Zoom weights q(i)=cos(((i·0.5)π)/rz) for i<rz and zero elsewhere. Integers convert to binary32 at use sites; π is binary32. If rz=0, all q are zero and the cosine division is not evaluated.

The inertial method additionally derives from d=damping:

$$B=1+(6d)d,\quad\lambda=\sqrt{B+\sqrt{BB+3}},\quad f=cutoff/\lambda,$$
$$c_d=(12.56d)/fps,\quad c_q=39.44/(fps\,fps).$$

Set kx=5/|dxmax|, ky=5/|dymax|, kr=5/|rotmax|, replacing each by zero when its limit is zero. Require f>0 and finite required intermediates. Negative damping is allowed and changes cd's sign. Window mode does not calculate these inertial coefficients.

## Cumulative paths and inverse maps

For interval [b, h], let M(m) convert motion with forward=true, time=1 and Z(z)=M(0, 0, 0, z):

$$C_b=I,\qquad C_j=M(m_j)\circ C_{j-1}\quad(j>b).$$

The first frame is the base, so mb itself is not composed. Z retains the conversion's replacement of nonpositive scale by 1.

Stabilization uses this specific inverse rather than a generic affine inverse:

$$e=\begin{cases}\sqrt{(-w)/v}&v\ne0,\\1&v=0,\end{cases}\quad
D=uu+(((vv)e)e),$$
$$u'=u/D,\quad h'=u',\quad v'=((-u')v)/u,\quad w'=((-v')e)e,$$
$$t_x'=(-u')t_x-v't_y,\quad t_y'=(-w')t_x-u't_y.$$

Input h is unused, even if rounded accumulation makes h≠u. Require u≠0, nonnegative square-root argument, nonzero divisors, and finite intermediates. Smoothed map S gives raw correction:

$$Q=S\circ Inv(C_n).$$

This first removes the actual cumulative path, then applies the smoothed path. Cn(x)=2x, S(x)=x+3 gives correction 0.5x+3.

## Method 0: inertial smoothing

Assume b<n. Initialize unzoomed paths Rb=R(b+1)=I. For j=b+2…n, update tx, ty, v independently with:

| Component | α | β | κ |
| --- | --- | --- | --- |
| tx | cd | cq | kx |
| ty | cd | cq | ky |
| v | 2cd | 4cq | kr |

For one component, let B=R(j−1), Cprev=R(j−2), and Uj be its cumulative-path value. Predict constant-velocity position A, velocity error E, and position error D:

$$A=2B-C_{prev},\quad E=((B-C_{prev})-U_{j-1})+U_{j-2},\quad D=B-U_{j-1}.$$

One predictor step is:

$$p=(A-((\alpha f)E)(1+((0.5\kappa)/f)|E|))-(((\beta f)f)D)(1+\kappa|D|).$$

Use it to update velocity error and correct once:

$$E_c=((p-C_{prev})-U_j)+U_{j-2},$$
$$R_j=(A-(((\alpha f)0.5)E_c)(1+(((0.5\kappa)/f)0.5)|E_c|))-(((\beta f)f)D)(1+\kappa|D|).$$

There is no iteration to convergence. Nonlinear multipliers strengthen response to larger errors; the corrector's position term still uses original D.

Complete the map with:

$$u_{R_j}=0.5(u_{C_j}+u_{R_{j-1}}),\quad h_{R_j}=u_{R_j},\quad w_{R_j}=((-v_{R_j})a)a.$$

With addzoom=false, Sn=Z(z0)∘Rn. Otherwise finish the entire unzoomed sequence before the independent zoom recurrence below; zoomed results never feed back into R.

### Geometric adaptive-scale quantity

For map T, start B(T)=z0 and successively replace it only by strictly smaller candidates:

$$1+(t_x+vc_y)/c_x,$$
$$1-(((t_x+uW)+vc_y)-W)/c_x,$$
$$1+(t_y+wc_x)/c_y,$$
$$1-(((t_y+wc_x)+hH)-H)/c_y.$$

Use full dimensions, not final pixel indices. This examines four center-line intersections, not corner extrema. Ties keep the earlier value; there is no positive lower bound. W=100, H=80, translation (+10, −4), z0=1 gives 1.2, 0.8, 0.9, 1.1 and bound approximately 0.8. Horizontal translation 100 can give −1; subsequent Z(−1) uses unit scale by the conversion rule.

### Inertial adaptive zoom

Initialize Ab=A(b+1)=sb=s(b+1)=z0, but Sb=S(b+1)=I. For j≥b+2:

$$A_j=B(R_j\circ Inv(C_j)).$$

Let Bprev=s(j−1), Cprev=s(j−2):

$$H=2B_{prev}-C_{prev},\quad E=((B_{prev}-C_{prev})-A_{j-1})+A_{j-2},$$
$$D=B_{prev}-A_{j-1},\quad z_f=1/(cutoff\,tzoom),$$
$$p=(H-(((z_fc_d)f)E))-(((((z_fz_f)c_q)f)f)D),$$
$$E_c=((p-C_{prev})-A_j)+A_{j-2},$$
$$s_*=(H-((((z_fc_d)f)0.5)E_c))-(((((z_fz_f)c_q)f)f)D).$$

If s*>Bprev, multiply zf by 0.7 and recompute predictor/corrector once using original history; otherwise retain the first trial. Set sj=min(s*, 1), with no lower clamp, then Sj=Z(sj)∘Rj. Future recurrence keeps sj itself even if Z replaced a nonpositive value.

At n=b+1, addzoom=true gives Sn=I and has not applied initial scale. At n=b, the main function directly uses Q=Z(z0). tzoom=0 fails only when a zoom recurrence actually needs its reciprocal.

### End taper and limits

Convert raw Q to motion dx, dy, r, z. If exact integer comparison F<fitlast+n+1 holds, set e=(F−n−1)/fitlast, multiply displacement/rotation by e, and replace scale by z0+(z−z0)e. Nonpositive fitlast never enters this branch.

Then process **dx, dy, z, r in that order**. Before each magnitude test, check the current component's finiteness. A nonfinite value resets the whole tuple to (0, 0, 0, z0) and sets b=n. Exceeding a negative limit does the same. Continue remaining components after reset without restarting earlier ones.

For translation v with limit L, act only if |v|>|L|. Negative L resets the whole tuple. Otherwise replace v by sign(v)sqrt(|v|L), repeat once if still over limit, then clamp to sign(v)L only if still |v|>|1.5L|. This is not conventional clipping: v=20, L=10 gives approximately 14.142136 then 11.892071, retaining the latter.

Scale acts only if |z−1|>|Lz|−1. Negative Lz resets; otherwise use 1±sqrt(|z−1||Lz−1|), choosing the original side of 1, once only. Rotation exceeding |rotmax| resets for negative rotmax or becomes sign(r)sqrt(|r|rotmax) for nonnegative rotmax, also once only.

Rebuild Q from the resulting tuple even if no value changed. The b=n scene-start branch and method 1 skip this entire operator.

### Recovering from recurrence overflow

Finite inertial inputs may generate infinity/NaN. Propagate these with IEEE binary32 arithmetic through adaptive zoom, raw correction, motion conversion, and end taper. Ordered component checks above decide whether to reset. Ordered comparisons with NaN are false; do not guess replacement scales early.

Recovery covers only nonfinite propagation generated inside inertial smoothing. Public parameters, motion properties, cumulative paths, and cumulative inverses must first be valid. Independent zero divisors, square roots of finite negatives, overflow in finite limit expressions, and neighbor/pixel-coordinate failures remain errors. Method 1 has no such recovery.

Recovery changes this request's correction/base only, not input motion or future state. Neighbor selection and resampling still run. z0 need not equal 1, so recovery is not a direct source copy.

## Method 1: symmetric window smoothing

Use the symmetric interval [b, h] about n. For tx, ty, v:

$$S_j={\sum_{k=b}^{h} C_{k,j}w(|k-n|)\over\sum_{k=b}^{h}w(|k-n|)}.$$

For u, average only max(b, n−1)…min(h, n+1) with the same weights and its own denominator. Then hS=uS, wS=((−vS)a)a. Sums start at +0 and visit increasing frame indices, rounding each product before adding and accumulating denominators in the same order. Zero endpoint weights do not remove motion dependencies.

Without adaptive zoom, replace S by Z(z0)∘S. With it, set bz=max(b, n−rz), hz=min(h, n+rz), initialize A(bz)=z0, then:

$$A_k=B(C_k\circ Inv(C_k)),\quad k>b_z.$$

This uses the cumulative path and its particular inverse, not S. Do not replace the rounded composition by exact identity. Calculate:

$$z_s={\sum_{k=b_z}^{h_z}A_kq(|k-n|)\over\sum_{k=b_z}^{h_z}q(|k-n|)}.$$

If zs>1, set it to 1, then S←Z(zs)∘S. There is no lower clamp. rz=0 makes the denominator zero even for a static image and fails.

Finally Q=S∘Inv(Cn), then convert Q to motion and back. This mandatory round trip can change coefficients/sampling class. Method 1 applies no fitlast, translation/rotation limits, or Lz constraint.

[Back to the English index](../README.md)
