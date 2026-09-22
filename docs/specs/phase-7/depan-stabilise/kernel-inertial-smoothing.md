# Inertial cumulative-motion smoothing

Inputs are C over [b,n] with b<n, effective aspect a and the inertial coefficients. Output R is an unzoomed smoothed sequence. Set Rb=I and R(b+1)=I. For each j=b+2 through n, compute the following independently for components tx,ty,v. Their coefficient choices are:

| Component | alpha | beta | kappa |
| --- | --- | --- | --- |
| tx | cd | cq | kx |
| ty | cd | cq | ky |
| v | cd*2 | cq*4 | kr |

Let B=R(j-1)'s component, Cprev=R(j-2)'s component, and Uj=Cj's corresponding component. Define

$$A=2B-Cprev,\quad E=((B-Cprev)-U_{j-1})+U_{j-2},\quad D=B-U_{j-1},$$
$$p=(A-((\alpha f)E)(1+((0.5\kappa)/f)|E|))-(((\beta f)f)D)(1+\kappa|D|),$$
$$E_c=((p-Cprev)-U_j)+U_{j-2},$$
$$R_j=(A-(((\alpha f)0.5)E_c)(1+(((0.5\kappa)/f)0.5)|E_c|))-(((\beta f)f)D)(1+\kappa|D|).$$

The last expression defines the selected component, not the entire map. This is one predictor and one corrector; do not iterate until convergence. Do not cancel f across a nonlinear product or reuse the predicted error in place of D. Rotation doubles only alpha and quadruples beta; f in the nonlinear factor remains unchanged.

Complete each map with

$$u_{R_j}=0.5(u_{C_j}+u_{R_{j-1}}),\quad h_{R_j}=u_{R_j},\quad w_{R_j}=((-v_{R_j})a)a.$$

Finite inputs can cause the recurrence to overflow. Apply the [inertial numerical recovery arithmetic boundary](kernel-numerical-recovery.md): retain the specified binary32 expressions and propagate generated infinity/NaN toward the current correction rather than failing at the first non-finite intermediate. Recovery is decided by the ordered correction-limit contract. Arithmetic failures outside that boundary remain frame errors before rendering.

When addzoom=false, output Sn=Z(z0) composed with Rn. When addzoom=true, pass the entire unzoomed R sequence to [adaptive zoom](kernel-inertial-zoom.md). Adaptive results must never feed back into the recurrence above. In particular, b+1 has R=I regardless of C(b+1).

## Examples

- With b=0,n=1,z0=1 and a single +2 horizontal step, R1=I and the raw correction is -2. With addzoom=false,z0=0.5, Sn=Z(0.5); with addzoom=true, S1=I and this initial scale is not applied at that frame.
- A standalone component fixture with f=1,alpha=0,beta=0.25,kappa=0, B=Cprev=0 and U(j-2)=0,U(j-1)=1,Uj=2 gives E=-1,D=-1,p=0.25,Ec=-1.75 and Rj=0.25. These are exact binary32 results. It tests the recurrence independently of parameter normalization.
- If uCj=1.5,uR(j-1)=1, the new diagonal is 1.25. If vRj=0.25,a=2, the reconstructed w is -1.
