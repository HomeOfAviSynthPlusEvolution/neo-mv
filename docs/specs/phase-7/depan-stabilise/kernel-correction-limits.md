# End taper and correction limits

Inputs are finite raw correction Q, n,F,b with b<n, z0, fitlast, dxmax,dymax,Lz,rotmax,aspect and center. Outputs are final Q and possibly updated b. This operator is used only for method=0 after smoothing. Convert Q to motion (dx,dy,r,z), forward=true, using the shared equations.

## End taper

If F<fitlast+n+1, where the comparison uses exact integer arithmetic, define

$$e=\operatorname{fl32}(F-n-1)/\operatorname{fl32}(fitlast).$$

Replace dx by dx*e, dy by dy*e, r by r*e, and z by z0+(z-z0)*e. Otherwise leave them unchanged. A zero or negative fitlast never enters the branch for valid n. Do not calculate a denominator on the unselected branch.

## Ordered limits

Process dx, then dy, then z, then r. A hard reset replaces the entire tuple with (0,0,0,z0) and sets b=n; continue testing the remaining components after the reset. All required intermediates must be finite; numerical failure is a frame error rather than a hard reset. Let sign(v) be +1 for v>=0 and -1 otherwise.

For a translation component v with limit L (dxmax or dymax), act only if |v|>|L|:

- If L<0, hard reset.
- Otherwise replace v by sign(v)*sqrt(|v|*L). If |v|>|L| still holds, repeat that replacement once. If the resulting |v|>|L*1.5|, replace v by sign(v)*L. Equality at any comparison does not trigger its branch.

For zoom, act only if |z-1|>|Lz|-1. If Lz<0, hard reset. Otherwise replace z by 1+sqrt(|z-1|*|Lz-1|) when z>=1, or 1-sqrt(|z-1|*|Lz-1|) when z<1. This is one pass, with no final clamp.

For rotation, act only if |r|>|rotmax|. If rotmax<0, hard reset. Otherwise replace r by sign(r)*sqrt(|r|*rotmax), once, with no final clamp.

Rebuild Q=M(dx,dy,r,z), even if no limit changed the tuple. This retains M's nonpositive-scale guard. The updated b controls previous-fill extent and diagnostics. Method 1 bypasses this entire operator. Method 0's b=n scene-start branch also bypasses it.

## Examples

- dx=20,dxmax=10 gives approximately 14.142136 after pass one and 11.892071 after pass two. This is below 15, so it remains greater than 10. The limit is not a conventional clamp.
- dx=-160,dxmax=10 gives -40, then -20, then -10 because 20>15.
- dx=20,dxmax=-10 resets all motion and b=n. Later zoom and rotation checks still run on the reset tuple.
- z=1.25,Lz=1.0625 becomes 1.125. r=4,rotmax=1 becomes 2. Neither is clamped to its nominal limit.
- F=10,n=8,fitlast=4 gives e=0.25. Motion (dx=8,dy=-4,r=2,z=0.9),z0=0.8 becomes approximately (2,-1,0.5,0.825) before limits. At n=9 it becomes (0,0,0,0.8).
