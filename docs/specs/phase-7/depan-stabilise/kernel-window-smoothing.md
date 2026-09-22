# Symmetric window smoothing

Inputs are C over the symmetric interval [b,h] centered on n, the normalized w,q,r,rz, and geometric parameters. Output is one smoothed map S and raw correction Q. Weighted accumulations below begin at binary32 +0, visit indices in increasing order, round each product before adding it, and accumulate the denominator in the same order. Include zero-weight entries; all referenced cumulative maps have already been constructed.

For components tx,ty,v:

$$S_j={\sum_{k=b}^{h} C_{k,j}w(|k-n|)\over\sum_{k=b}^{h}w(|k-n|)}.$$

For u, use only k from max(b,n-1) through min(h,n+1), with its own denominator and the same w weights. Set hS=uS and wS=((-vS)*a)*a. Every denominator must be nonzero and every intermediate finite.

When addzoom=false, replace S by Z(z0) composed with S.

## Window adaptive zoom

When addzoom=true, let bz=max(b,n-rz),hz=min(h,n+rz). Set A(bz)=z0. For each k=bz+1 through hz, set

$$A_k=B(C_k\circ Inv(C_k)).$$

Use the rounded composition and the stabilization inverse exactly. Do not replace this with a constant identity, or use S in place of Ck. The first A is initialized directly even when bz<n. Form

$$z_s={\sum_{k=bz}^{hz} A_kq(|k-n|)\over\sum_{k=bz}^{hz}q(|k-n|)}$$

in increasing index order. If zs>1, set it to 1; there is no lower clamp. Replace S by Z(zs) composed with S. With rz=0, q(0)=0 and this division is a controlled frame error, even for a static image or b=h=n.

Finally Q=S composed with Inv(Cn). Convert Q to motion and back through M before rendering or selecting neighbors. Method 1 applies neither fitlast nor any correction limit; dxmax,dymax,rotmax do not influence its smoothing, and Lz does not constrain zs.

## Examples

- r=1 has weights [1,0], so a valid full interval's tx,ty,v and u averages select Cn, subject to the specified rounded sums. Endpoints are nevertheless decoded and composed. As a concrete exact case, use a=1,center=(24,24),b=0,n=1,h=2, cumulative horizontal translations [0,1,2],initzoom=1,addzoom=false: the correction is numerically identity. This does not authorize an identity shortcut for other aspects or geometry; even Z(1)'s center arithmetic can leave a small translation.
- For b=0,n=1,h=2,r=2,addzoom=false,z0=1 and pure horizontal cumulative translations [0,1,1], use a=1,center=(24,24) and let c=w(1). The average translation is (1+c)/((c+1)+c), approximately 0.70710677. The raw correction is approximately -0.29289323, before the mandatory motion round trip. This tests asymmetric motion in a symmetric interval.
- If all C are exact identity,z0=0.75, every A=0.75 and the window adaptive scale is their specified rounded weighted mean. With rz=1 the sole nonzero central weight is 1, giving exactly 0.75 when the central A is 0.75.
- b=h=n=0 with addzoom=true and rz>=1 yields S=Z(min(z0,1)), since C0=I and the sole zoom sample is initialized to z0 before the upper clamp. With rz=0 and addzoom=true, it fails instead.
