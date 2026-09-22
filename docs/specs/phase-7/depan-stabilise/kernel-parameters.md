# Stabilization parameters and coefficients

Inputs are validated plugin arguments, positive dimensions W,H and positive frame-rate integers N,D. Outputs are the following binary32 constants and finite weight sequences. Public domains/defaults are in [plugin.md](plugin.md). Let zuser be the public initzoom value and d the damping value.

$$fps=\operatorname{fl32}(N)/\operatorname{fl32}(D),\quad a=pixaspect/(fields?2:1),$$
$$c_x=\operatorname{fl32}(W)/2,\quad c_y=\operatorname{fl32}(H)/2,\quad z_0=1/zuser.$$

Require fps,a,cx,cy,z0 positive and finite. Define the effective zoom limit using the original argument, before its reciprocal:

$$L_z=\begin{cases}\max(zoommax,zuser)&zoommax>0,\\-\max(-zoommax,zuser)&zoommax\leq0.\end{cases}$$

Thus zoommax=0 is a negative hard-reset limit, not a disabled scale limit.

## Temporal weights

$$r=\max(1,\operatorname{trunc}(fps/(4\,cutoff))),\qquad r_z=\min(r,\operatorname{trunc}((fps\,tzoom)/4)).$$

Each quotient before truncation must be finite and its truncated value representable in signed int32; require r<=2147483646 so the sequence length r+1 is representable. These parameter-only checks apply to both methods. Allocation failure is an error; no silent radius cap is permitted. Exact integer temporal bounds are clipped to [0,F-1] before indexing.

Let pi be the nearest binary32 value of mathematical pi. For integer distance i:

$$w(i)=\cos(((\operatorname{fl32}(i)\,0.5)\,pi)/\operatorname{fl32}(r))\quad(0\leq i<r),\qquad w(r)=+0.$$

$$q(i)=\cos(((\operatorname{fl32}(i)\,0.5)\,pi)/\operatorname{fl32}(r_z))\quad(0\leq i<r_z),\qquad q(i)=+0\quad(r_z\leq i\leq r).$$

When rz=0, the first q range is empty and every q is zero; do not evaluate its division. Zero endpoint weights do not by themselves remove a frame from a motion/property dependency.

## Inertial coefficients

For method=0, also produce:

$$B=1+(6d)d,\quad \lambda=\sqrt{B+\sqrt{BB+3}},\quad f=cutoff/\lambda,$$
$$c_d=(12.56d)/fps,\qquad c_q=39.44/(fps\,fps),$$
$$k_x=\begin{cases}5/|dxmax|&dxmax\ne0\\0&dxmax=0\end{cases},\quad
k_y=\begin{cases}5/|dymax|&dymax\ne0\\0&dymax=0\end{cases},\quad
k_r=\begin{cases}5/|rotmax|&rotmax\ne0\\0&rotmax=0.\end{cases}$$

Require f>0 and every required coefficient/intermediate finite. Negative damping is admitted; it does not change the sign inside lambda's squared term but does change cd. There is no nonlinear scale coefficient in the smoothing contract. Method=1 does not require these inertial coefficients.

## Examples

- fps=24,cutoff=3 gives r=2. w is approximately [1,0.70710677,0]. With tzoom=0.25, rz=1 and q=[1,0,0].
- fps=24,cutoff=3,tzoom=0 gives rz=0 and q=[0,0,0]. Creation succeeds if other normalization succeeds. Window adaptive zoom then has a zero denominator and fails when requested; inertial adaptive zoom fails only if its recurrence is reached.
- initzoom=2,zoommax=1.05 gives z0=0.5,Lz=2. With zoommax=-1.05 instead, Lz=-2. With zoommax=0 it is also -2.
- W=48,H=32,pixaspect=1.5,fields=true gives center (24,16) and a=0.75. No frame parity is read.
