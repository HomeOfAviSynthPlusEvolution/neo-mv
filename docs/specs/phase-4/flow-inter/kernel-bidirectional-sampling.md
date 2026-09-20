# Bidirectional image samples

Inputs are validated logical level-zero images L,R, dense per-plane main vectors B,F, optional BB,FF, pel p, and integer t in [0,256]. Output is the tuple of samples needed by the chosen composition formula at every visible plane position. The plugin chooses l,r and the images. This kernel does not fetch video frames or choose availability.

For plane position x,y define q=(px,py) in unpadded plane-pel units and componentwise

$$D(v,t)=\lfloor vt/256\rfloor.$$

There is no +128 bias. The main samples are

$$A=L(q+D(F,t)),\qquad C=R(q+D(B,256-t)).$$

Basic composition additionally uses A0=L(q),C0=R(q). Extra-field composition instead additionally uses

$$E=L(q+D(FF,t)),\qquad K=R(q+D(BB,256-t)).$$

F sampling L and B sampling R is intentional. BB/FF still sample R/L, not farther temporal images. All coordinates are already in plane units; do not apply subsampling ratios again.

For an unpadded point a=(ax,ay), split qx=floor(ax/p),alphaX=ax-p*qx and likewise Y. Its stored sample is phase P(alphaX,alphaY) at (hP+qx,vP+qy), with that plane's actual padding. This is the logical phase map from [Super interpolation](../../phase-1/super/kernel-subpixel-interpolation.md), without any new image interpolation.

## Preflight and numeric input domain

Before reading image samples or publishing any generated output, check every required position in every visible plane against the selected phase's defined domain. Basic mode requires A,C,A0,C0; extra mode requires A,C,E,K. All required samples must be valid even if a mask, time endpoint or coefficient would give them zero arithmetic weight. A failed position is a frame error; do not clamp it, invent edge samples, skip a plane or downgrade extra mode to basic mode. Built-in quarter-phase excluded edges remain excluded. Extra mode does not require displaced samples from ineligible extra fields because such a field selects basic mode before this kernel.

Only visible positions are evaluated. For arithmetic composition, sampled integers must lie in [0,2^renderDepth-1], and sampled float32 values must be finite. Non-finite used samples or required arithmetic overflow are frame errors. Direct copy branches defined by plugins do not enter this arithmetic kernel.

## Examples

- V=-1,t=128 gives D=-1, unlike the single-reference Flow displacement 0. At p=2,x=0 with hP=1, it reads phase 1 at padded column 0.
- p=2,x=0,Fx=-3,t=256: L coordinate -3 splits into qx=-2,alphaX=1. hP=1 gives column -1 and a frame error; hP=2 gives column 0 and is allowed if the Y coordinate is valid.
- At t=0, A and E use zero displacement, but C and K use full B and BB. FlowInter still validates and computes the selected formula. FlowFPS may bypass this kernel at its endpoint.
- A zero-weight extra K position outside its domain is still an error when extra mode is selected. An unavailable extra field selects basic mode and removes E/K sample requirements altogether.
