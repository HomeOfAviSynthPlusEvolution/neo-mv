# Composition of a block grid into visible samples

Inputs: a row-major Nx by Ny grid of generated blocks q, block Bx,By and overlap Ox,Oy in the current plane, visible dimensions wr,hr, sample precision b, and [window coefficients](kernel-overlap-window.md). Output: exactly wr by hr visible samples. Block (bx,by) has origin (bx*(Bx-Ox),by*(By-Oy)).

Require positive steps and complete visible coverage, with the coverage rectangle contained in the supplied working image as specified by the [render geometry](kernel-reference-availability.md). All blocks have complete, valid sample rectangles before cropping. Geometry errors are not repaired with uncovered black or centre-image pixels.

## Without overlap

When Ox=Oy=0, copy the unique covering block's sample. At the right and bottom, emit only coordinates within the visible dimensions. Cropping is based on each block origin and the visible rectangle, not on W-Wr or H-Hr. Working dimensions larger than grid coverage do not shift or enlarge the visible result.

## With overlap

When either overlap is nonzero, for each visible coordinate collect the covering blocks in row-major block order. Let qk be each block's already-generated sample and Wk its local window coefficient. For integer samples define

$$a_k=\left\lfloor q_kW_k/64\right\rfloor,\qquad
out=\min(2^b-1,\max(0,\lfloor(\sum_k a_k+16)/32\rfloor)).$$

The per-contribution division precedes the final sum. Keep any previous rounding in qk; do not merge block generation and overlap arithmetic into a differently rounded expression. Use sufficiently wide exact accumulations.

For float samples set z0=+0 and, for covering blocks in row-major order,

$$a_k=\operatorname{fl32}(\operatorname{fl32}(q_k W_k)/64),\quad
z_{k+1}=\operatorname{fl32}(z_k+a_k),\qquad out=\operatorname{fl32}(z_{final}/32).$$

There is no +16 bias and no clipping to [0,1]. Do not divide by the sum of window weights; fixed scaling by 2048 defines the operation. Small window-quantization effects are part of the numerical contract. Degrain applies change limiting after this step.

Each output coordinate depends only on its covering blocks. Work may be partitioned without changing the scalar contribution order at a coordinate. Input blocks/windows remain immutable; output and writable scratch cannot alias them. No layout of intermediate image rows is required.

## Examples

- Two contributions W=1024 with q=10 and 21 give a=160 and 336. Integer out=floor((496+16)/32)=16; float out=15.5.
- One block with W=2048 and q=100 gives out=100. A single-block grid keeps that result with nonzero overlap because both outer edges have unit windows.
- No overlap, block width 8, Nx=3, visible width 18 and working width 32: emit 8,8,2 columns from the three blocks, respectively. The six other generated columns and the eight working columns outside coverage are not visible outputs.
- Visible width 18, Bx=8,Ox=4,Nx=3 gives coverage 16 and is rejected; no rule silently fills the last two columns.
