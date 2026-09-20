# Block sampling and error metrics

## Inputs and outputs

Inputs: current/reference Super planes at the same level, block top-left (x,y) in working luma coordinates, block size Bx,By, pel p, vector (vx,vy), chroma ratios rx,ry, chroma enable, and SAD/SATD selection. Output: nonnegative int64 luma error eY, chroma error eC and raw error s=eY+eC. No search penalty is included.

Use p=SuperPel at level zero and p=1 above it. x,Bx must be divisible by rx, and y,By by ry. Current samples always use the integer phase. Reference samples use displacement (vx/p,vy/p). Decompose signed components with floor division and a nonnegative remainder: vx=p*qx+ax, 0<=ax<p, and similarly Y. For a plane with padding hp,vp, luma reference element (i,j) is phase (ax,ay) at (hp+x+i+qx,vp+y+j+qy).

For chroma, first use vxc=trunc(vx/rx), vyc=trunc(vy/ry), then decompose those components by p. Use block origin (x/rx,y/ry), size Bx/rx by By/ry, and chroma padding. This two-stage rounding is required. Every source/reference location must be in the [defined Super phase domain](../super/kernel-subpixel-interpolation.md); validate before reading. Samples are not interpolated again by this operator.

## Sampling admissibility and geometry errors

Define Safe(v) for a block and the supplied logical Super geometry as follows. For each enabled plane P, let its ratios be rPx,rPy (1,1 for Y and rx,ry for U/V), its actual level padding be hP,vP, and its block dimensions be bPx=Bx/rPx,bPy=By/rPy. For each axis define

$$t_x=\operatorname{trunc}(v_x/r_{Px}),\quad q_x=\lfloor t_x/p\rfloor,\quad a_x=t_x-pq_x,$$

and analogously for Y. The reference footprint is the integer-coordinate rectangle

$$[h_P+x/r_{Px}+q_x,\ h_P+x/r_{Px}+q_x+b_{Px})\ \times\ [v_P+y/r_{Py}+q_y,\ v_P+y/r_{Py}+q_y+b_{Py}).$$

Safe(v) holds exactly when this entire rectangle lies in that reference plane's defined phase (ax,ay) domain, and the corresponding current rectangle with qx=qy=0 lies in its integer-phase domain, for every enabled plane. Plane padding comes from the Super level geometry; do not derive it from a candidate bound or replace it with a larger value. With chroma=false, U/V footprints are not part of this predicate.

Analyse and Recalculate require a nonempty Omega and Safe(v) for **every integer v in Omega**, for every block at every layer they use. They also require Safe(v) for any separately admitted initial seed outside Omega. This is a geometry precondition independent of image values, search mode, search radius, thresholds, predictor values or which candidates a search would visit. A proof using bounds is permitted; enumerating all candidates or reading image samples is not required for this check.

If the precondition fails, report a controlled unsupported-sampling-geometry error before motion/error evaluation. At plugin creation, validate the complete known geometry, even if a reference would be unavailable on some or all frames. Requested frames must conform to the validated logical geometry and view contract; a violation discovered in a later frame is a frame error before motion/error evaluation. Do not defer a known geometry failure until a particular candidate happens to be evaluated.

Do not intersect or shrink Omega, skip an in-Omega candidate, clamp an individual sample coordinate, fill undefined phase edges, or enlarge padding to obtain a result. Ordinary candidates outside Omega are still excluded by the search relation. All existing seed clamping, candidate priorities and tie rules remain unchanged for admitted geometry. The special Analyse zero seed has no exemption from Safe, even when outside Omega. Built-in quarter phases use their restricted domains; external pel phases use their full declared domains. A direct block-error call with an unsafe supplied vector fails before reading samples and returns no metric.

### Sampling examples

- YUV420, actual/working dimensions 16x16, block 8x8, overlap 0, padding 3x3, pel=2, one level, chroma=true, fields=false: the top-left block has Omega=[-6,22) in each axis. At v=(-6,0), luma starts at (0,3), but U/V use padding 1 and tx=trunc(-6/2)=-3=2*(-2)+1. Their phase (1,0) rectangle starts at (-1,1), which is outside the domain. Analyse and Recalculate reject this target geometry at creation, even with a search radius of 1 or an error threshold that would avoid refinement. No vector/SAD output is produced. Super itself may produce these logical planes; its success does not imply that every analysis geometry is admissible.
- Change only padding to 4x4. The top-left Omega is [-8,24) in each axis and U/V padding is 2. The same v=(-6,0) reads U/V phase (1,0) over [0,4) x [2,6), within its 12x12 plane; luma reads [1,9) x [4,12) within its 24x24 plane. Every vector in every block's Omega is sampleable. For each axis, the first block has range [-8,24) and the second [-24,8); after their respective origins are included, reference starts range from 0 through 15 in luma and 0 through 7 in U/V. The blocks therefore end at most at exclusive coordinates 23 and 11. With valid constant-7 current/reference planes and SAD selected, the direct metric at (-6,0) is eY=eC=s=0; geometry is accepted.
- For a built-in pel=4 plane of padded width a=12, a four-sample footprint [8,12) in phase (3,0) is invalid: it includes u=11, while this phase requires u<11. If such a footprint belongs to any candidate in Omega or a separately admitted seed, reject the geometry; a footprint [7,11) is within the horizontal domain. Vertical coordinates and other enabled planes must independently pass. An external pel plane may define the full [0,12) width; that does not change the built-in phase definition.

## Integer SAD and SATD

For sample matrices S,R of width w and height h,

$$SAD(S,R)=\sum_{j=0}^{h-1}\sum_{i=0}^{w-1}|S_{j,i}-R_{j,i}|.$$

Use signed differences and wide exact sums. Do not normalize by area, bit depth, nominal range or black level.

SATD applies only to luma blocks tiled by nonoverlapping 4x4 cells; 16x2 blocks cannot use it. For each cell E=S-R,

$$H=\begin{pmatrix}1&1&1&1\\1&-1&1&-1\\1&1&-1&-1\\1&-1&-1&1\end{pmatrix},\quad F=HEH^T,$$
$$SATD=\sum_{cells}\left\lfloor\frac{\sum_{a,b=0}^3|F_{a,b}|}{2}\right\rfloor.$$

Do not substitute a larger transform over the whole block. U/V always use SAD; enabling chroma adds both errors without compensating for their smaller areas.

## Float32 baseline

Each operation rounds to binary32 nearest-even. SAD uses abs(fl(S-R)), summed from +0 in row-major order. Finite samples may be outside [0,1]; do not clip them.

For SATD, transform each row and then each column with

$$h(z)=((z_0+z_1)+(z_2+z_3),\ (z_0-z_1)+(z_2-z_3),\ (z_0+z_1)-(z_2+z_3),\ (z_0-z_1)-(z_2-z_3)).$$

Round each add/subtract. Visit cells in row-major order. Within each cell visit columns 0 through 3, taking the left-associated sum of four absolute coefficients in each column and adding that column sum to a total initially +0. Multiply the final total by 0.5 to obtain e. Do not integer-encode individual cells.

Encode each plane separately using z=fl(e*65535):

$$
Q(e)=\begin{cases}
0 & z\le0 \\
4294967295 & z\ge4294967040 \\
\operatorname{trunc}(fl(z+0.5)) & \text{otherwise.}
\end{cases}
$$

Require finite intermediate values. eY=Q(luma metric); eC=Q(U SAD)+Q(V SAD) when chroma is enabled, otherwise zero. Sum encoded values in int64. Integer metrics do not use Q. The exported property is s, regardless of subsequent candidate penalties.

## Examples and storage

- p=2,vx=-1 selects integer offset -1 and phase 1. With rx=2 it gives chroma displacement zero; vx=-3 gives chroma component -1 and chroma displacement -0.5.
- An 8x8 integer block with every sample difference 2 has SAD=128 at either 8 or 10 bits.
- A 4x4 cell whose differences are all 1 has SAD=16, SATD=8. A single difference of 1 also gives SATD=8, but SAD=1.
- YUV420 block 8x8, all plane differences 1: SAD total 64+16+16=96. SATD luma gives total 32+16+16=64.
- Float 4x4 with every difference 1/65536 gives encoded SAD=16 and SATD=8.

Inputs are immutable. Only scalar results and optional private temporary coefficient/sample storage are written; no frame pixels/properties are changed. Consumers: Analyse and Recalculate.
