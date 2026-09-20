# Block selection and time-scaled compensation

Inputs: a Complete scene-eligible vector field, current/reference logical Super images, block grid, saved d, nonnegative thsad, finite binary64 time in [0,100], fields and optional field parity. Output: one generated block per grid location and render plane, ready for block composition. An unavailable field is handled by the plugin before invoking this operator.

## Time and error threshold

With independently rounded binary64 arithmetic define

$$t=\operatorname{trunc}(\operatorname{fl64}(\operatorname{fl64}(time\,256)/100)),\qquad T=S(thsad).$$

Use [S and the analysis descriptor](kernel-reference-availability.md#thresholds), not render bit depth. t is in [0,256], T is a nonnegative representable int64. thsad=0 is valid; no block can have nonnegative SAD strictly below zero.

## Field shift

fields=true requires p>1 at creation. Otherwise fields=false gives f=0. For fields=true and odd d, establish parity for both Super[n] and Super[n+d]. If tff is supplied, top(k)=bool(tff) XOR (k is odd). If omitted, each required frame must have a readable integer _Field; its first value is nonzero for top. Missing, empty or wrongly typed field data is a frame error. Later elements are ignored.

$$f=\begin{cases}p/2&top(n)\land\neg top(n+d),\\-p/2&\neg top(n)\land top(n+d),\\0&\text{otherwise}.\end{cases}$$

With even d, f=0 and no field-property read is required. With fields=false, ignore tff and _Field. For creation-time admission of current-block positions (0,f), check f in {0,-p/2,+p/2} when fields=true and d is odd, and {0} otherwise. The reference map is checked over all of V with f=0 at creation; actual reference vectors plus the actual f are checked at frame evaluation as specified by [sampling admission](kernel-block-sampling.md#whole-domain-admission). A failure is a frame error before any block is generated, even for a block whose SAD would select the current image.

## Selected samples

For block i with vector (vx,vy) and stored raw SAD s, select

$$image_i=\begin{cases}Super[n+d]&s<T,\\Super[n]&s\ge T,\end{cases}$$
$$ (d_x,d_y)_i=\begin{cases}
(\operatorname{trunc}(v_xt/256),\operatorname{trunc}(v_yt/256)+f)&s<T,\\
(0,f)&s\ge T.
\end{cases}$$

Products and divisions in the displacement formula are exact integer operations with truncation toward zero. The field shift is added after time scaling and is not scaled by t. Feed the selected image and displacement into [render block sampling](kernel-block-sampling.md), then assemble blocks with [block composition](kernel-overlap-composition.md). All actual render planes are processed, even when AnalysisChroma is false.

There is no image blend proportional to time or SAD. s=T selects the current Super block. A current-block selection still applies f; it is not necessarily an integer-phase copy. Raw SAD and vectors are read-only and are not recomputed or re-exported.

## Examples

- 8-bit luma-only 8x8 analysis gives F=1. With thsad=100, fields=false, time=100, current block 10 and reference block 80: s=99 produces 80; s=100 produces 10. Properties in both cases remain those of clip[n].
- With s=99 and time=0, use the reference's unshifted samples, still 80 in that example. time=0 does not select the current frame.
- p=2, vx=-3,time=50 gives t=128 and dx=-1. At x=0, the Y sample is half a luma pixel left; with rx=2 the sampling map also selects half a chroma pixel left. This differs from first truncating vx/rx and then applying time.
- fields=true, p=2, d=1, time=100, pad=4, 16x16 YUV420 with 8x8 blocks, no overlap and zero vectors: current top and reference bottom gives f=1 and valid footprints. With s>=T, sample the current Super at +0.5 luma pixel vertically. With fields=false the same block has f=0.

The operator has no cross-block writable state. Every source footprint must have passed the applicable creation and actual-field-shift checks; unexpected missing sample data is a controlled error, not another fallback selection.
