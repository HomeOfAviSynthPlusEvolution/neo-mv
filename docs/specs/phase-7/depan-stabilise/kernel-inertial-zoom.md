# Inertial adaptive zoom

Inputs are C,R over [b,n] with b<n, geometric parameters, z0, cutoff,tzoom,cd,cq,f. This operator is used only for method=0,addzoom=true. Outputs are S and the adaptive-scale sequences A,s. Initialize Ab=A(b+1)=sb=s(b+1)=z0 and Sb=S(b+1)=I.

For j=b+2 through n, calculate

$$A_j=B(R_j\circ Inv(C_j))$$

using the [geometric bound](kernel-zoom-bound.md). Let Bprev=s(j-1),Cprev=s(j-2) and define

$$H=2Bprev-Cprev,\quad E=((Bprev-Cprev)-A_{j-1})+A_{j-2},\quad D=Bprev-A_{j-1},\quad z_f=1/(cutoff\,tzoom).$$

For a trial coefficient zf, the predictor/corrector is

$$p=(H-(((z_f c_d)f)E))-(((((z_f z_f)c_q)f)f)D),$$
$$E_c=((p-Cprev)-A_j)+A_{j-2},$$
$$s_*=(H-((((z_f c_d)f)0.5)E_c))-(((((z_f z_f)c_q)f)f)D).$$

If s*>Bprev, replace zf by zf*0.7 and recompute p,Ec,s* once using the original H,E,D,Bprev,Cprev. Otherwise retain the first trial. Set sj=1 if s*>1, else sj=s*. Finally Sj=Z(sj) composed with Rj. No lower clamp, correction-limit test or further iteration occurs in this operator. The final raw correction is Sn composed with Inv(Cn).

This pass consumes the completed unzoomed R sequence; an earlier Sj is used only through the scalar s history, never as a replacement for R(j-1). The scale used for subsequent recurrence is sj itself, even if Z substituted unit scale for a nonpositive sj.

Required arithmetic is finite except for values propagated under [inertial numerical recovery](kernel-numerical-recovery.md). That boundary also applies to this operator's geometric-bound evaluation and map compositions; it does not authorize replacing a failed adaptive result by a guessed scale. With tzoom=0, failure occurs only when a j>=b+2 iteration is required. An empty recurrence at n=b+1 succeeds and returns Sn=I, even for z0!=1. The caller's b=n scene-start branch does not invoke this operator.

## Examples

- In a standalone scale-recurrence fixture let z0=1,cd=0.5,cq=0.25,f=1,cutoff=tzoom=1, previous A and s both 1, and current Aj=0.8. H=1,E=D=0,p=1 and Ec is approximately 0.2; the first trial is approximately 0.95. It is below the previous 1, so no second trial runs.
- In the same fixture with current Aj=1.2, the first trial is approximately 1.05, triggering a second trial with zf=0.7. That trial is approximately 1.035, then the upper clamp gives sj=1.
- With n=b+1,initzoom=2,addzoom=true, no initial Z(0.5) is applied: Sn=I. With n=b, the caller instead directly uses Q=Z(0.5).
