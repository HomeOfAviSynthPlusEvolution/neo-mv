# DepanEstimate: global motion from periodic correlation peaks

## 1. What the function computes

`DepanEstimate` compares the first planes of current and previous frames, locates periodic correlation peaks in one or two rectangular windows, and exports horizontal/vertical displacement, scale, and validity. It does not estimate rotation; Depan_rot is always zero.

By default pixels copy the current frame exactly. Results are [Depan motion properties](shared/global-motion.md); show can draw correlation surfaces into the first plane and info can overlay text.

## 2. Objects and notation

Image size is W×H, count F. Basic observation j compares current A=clip[j] with previous B=clip[max(0, j−1)]. Windows are wx×wy, area K=wxwy, and search radii mx, my.

Correlation C(i, j) is indexed by periodic shifts inside the window. Horizontal index wx−1 means displacement −1, not nearly a full window to the right.

Except integer indices/counts and stated exceptions, real arithmetic below is binary32, rounding each operation. Integer samples retain their stored scale; integer16 is not normalized to 0–1.

## 3. Overall calculation

1. Determine window geometry and peak-search limits.
2. Compute surfaces for the current observation and its existing immediate neighbors.
3. Search peaks in fixed order and calculate confidence.
4. Refine accepted peaks to subpixels and apply field correction.
5. Combine one/two windows, then compare neighboring basic confidences.
6. Copy the current image, write motion, and optionally display surfaces/text.

## 4. Step-by-step calculation

### 4.1 Window geometry

Set L=max(wleft, 0). winx must be a nonnegative even value no greater than W−L. Zero selects the largest power of two ≤min(W−L, 8192), with result at least 2. Explicit widths are not capped at 8192.

zoommax=1 selects one window; any other value selects two and halves that width. The halved width must still be positive and even, so original two-window width must be divisible by four.

With wleft<0, single-window left is floor((W−wx)/2); two-window left is floor((W−2wx)/4). Otherwise use wleft. The second begins at:

$$left_2=left+\lfloor W/2\rfloor.$$

Both windows must fit the first plane. The second need not directly adjoin the first.

Vertically set T=max(wtop, 0), require 0≤winy≤H−T. Zero chooses the largest power of two ≤min(H−T, 8192); explicit odd heights are supported. Negative wtop centers vertically; otherwise use it directly.

Negative dxmax/dymax choose floor(wx/4), floor(wy/4). Final bounds are:

$$0\le m_x<\lfloor w_x/2\rfloor,\qquad0\le m_y<\lfloor w_y/2\rfloor.$$

Height 1 is therefore invalid; zero search radii are valid. Fields/aspect do not change window sizes or alternate sampled rows.

For 1920×1080 defaults, one window is 1024×1024 at (448, 28), radii (256, 256). Two-window mode gives 512×1024 each at horizontal origins 224, 1184, with horizontal radius 128.

### 4.2 What periodic correlation computes

Apply negative-exponential real forward FFTs to both windows, then:

$$\widehat H=\overline{\widehat A}\,\widehat B.$$

For components ar+i ai, br+i bi:

$$h_r=\operatorname{fl32}(\operatorname{fl32}(a_rb_r)+\operatorname{fl32}(a_ib_i)),\quad
h_i=\operatorname{fl32}(\operatorname{fl32}(a_rb_i)-\operatorname{fl32}(a_ib_r)).$$

Use an unnormalized positive-exponential inverse FFT. The ideal result is:

$$C(i,j)=K\sum_{y=0}^{w_y-1}\sum_{x=0}^{w_x-1}A(x,y)B((x+i)\bmod w_x,(y+j)\bmod w_y).$$

Conceptually, periodically shift the previous window, multiply corresponding samples with the current window, and sum. Large values indicate stronger agreement at that shift. Positive displacement matches a current sample to a previous sample to its right/below.

There is no mean subtraction, taper, zero padding, or depth normalization. K cannot be omitted because confidence contains additive 0.1, making overall scale observable.

The real half-spectrum is wy×(wx/2+1); Hermitian symmetry supplies the omitted half. Do not erase the imaginary components of an entire first/last spectral column; only fully self-conjugate positions must be real.

### 4.3 Ordered peak search and confidence

Visit rows `0…my,wy-my…wy-1`, columns `0…mx,wx-mx…wx-1` within each. Second intervals are empty for zero radius. Middle positions are not candidates.

Initialize maximum at C(0, 0), replacing only on strict increase; ties retain the first visited position. Accumulate all visited values in binary32 order. Count J=(2mx+1)(2my+1), then:

$$P=C_{max}/K,\qquad M=(\mathrm{sum}/J)/K.$$

An index satisfying exact integer 2i<wx is positive displacement i; otherwise subtract wx. Likewise vertically.

Confidence and displacement penalty are:

$$T_0=((P-M)100)/(P+0.1),$$
$$A_x=m_x+1,\quad A_y=m_y+1,\quad d_x=A_x+stab|dx|,\quad d_y=A_y+stab|dy|,$$
$$T=T_0\left(\left((A_x/d_x)A_y\right)/d_y\right).$$

A more distinct peak increases P−M; positive stab penalizes larger integer displacement. T is not clamped to 0–100. Negative finite stab can raise it or produce a zero divisor and fail.

Accept a window exactly when T≥trust. An invalid window returns zero refined displacement, retains T, and skips refinement.

### 4.4 Subpixel refinement

For peak c0 and periodic horizontal neighbors c−, c+:

$$f_1=(c_+-c_-)/2,\qquad f_2=(c_++c_-)-2c_0,$$
$$a_x=\begin{cases}0&f_2=0,\\\min(1,\max(-1,-f_1/f_2))&f_2\ne0.\end{cases}$$

If |dx+ax|>mx, discard ax completely rather than clipping the final position. Vertical neighbors give ay similarly. Neighbors may lie outside the peak-search region.

Ordinary output is dx'=dx+ax, dy'=(dy+ay)/pixaspect. Field mode first adds +0.5 for top or −0.5 for bottom to ay, then doubles ay and integer dy, and finally divides by pixaspect. Do not reapply dymax afterward.

With neighbors 6, 8 and peak 10, ax≈1/6. Keep it at integer dx=0, but discard it if dx already equals the positive search limit.

### 4.5 Two windows and neighboring confidence

One window directly supplies displacement, confidence, validity, with zoom=1. With two windows, D=left2−left, first calculate:

$$z_*=1+(dx'_2-dx'_1)/D.$$

Accept only if both windows are valid and |z*−1|<zoommax−1. Average the two displacements and use z*. Otherwise reset displacement to zero and zoom to 1. In both cases retain T=min(T1, T2), preserving T1 on ties. Vertical disagreement does not estimate rotation. zoommax<1 still selects two windows but can never pass this strict test.

Basic observation zero is forced invalid with confidence zero only after correlation, sample validation, and required parity.

Output n also needs its existing basic observations n−1, n+1. Reject current motion if either neighbor satisfies:

$$T_n<2\,trust\quad\text{and}\quad T_n<0.5T_{neighbor}.$$

Both comparisons are strict. Neighbor validity does not suppress its confidence. These are basic confidences, not recursively filtered final outputs.

Final invalid tuple is `(dx,dy,rot,zoom,good)=(0,0,0,1,0)`. Valid motion keeps displacement/scale with zero rotation. Output n can need source images n−2…n+1, clipped to sequence bounds. Required neighbor observations still run when current motion is already invalid.

### 4.6 Surface and text display

show=true independently finds each complete current surface's extrema and computes:

$$q=(C-C_{min})\left(M_{pixel}/(C_{max}-C_{min})\right).$$

Mpixel is 2^b−1 for integers, 1 for floats. Integer results truncate toward zero and validate the resulting integer range; floats store directly. Constant surfaces fail through zero division. Draw at the original window position without centering zero shift or cropping to search bounds. Normalize windows separately; preserve chroma and pixels outside rectangles.

info=true writes `DepanEstimate_info`, for example `fn=2 dx=1.00 dy=0.00 zoom=1.00000 trust=88.01 bad=0`. Invalid motion displays reset displacement but retained basic confidence. The host renders that property; VapourSynth uses text.FrameProps. Text follows surface display.

## 5. A complete numerical example

Three `4×4` GRAY8 frames each contain one sample 1, others zero, at (2, 0),(1, 0),(0, 0). Use a `4×4` window, radii 1, stab=0, trust=4, zoommax=1.

Observation 1 compares current point (1, 0) with previous (2, 0). Its ideal surface has only C(1, 0)=16 nonzero. J=9 gives:

$$P=1,\quad M\approx1/9,\quad T\approx((1-1/9)100)/1.1\approx80.80808.$$

Equal neighboring values produce zero refinement, so displacement is (1, 0). Observation 2 matches. Observation 0 is forced invalid/confidence zero and cannot reject observation 1's positive confidence. Both later confidences exceed 8, so neighbor rejection does not apply.

Final tuples are invalid on frame 0 and `(1,0,0,1,1)` on frames 1, 2. With show/info false, all images remain unchanged. Actual binary32 FFT results approximate these ideal values.

## 6. Parameters and their calculation steps

| Parameter | Default | Role |
| --- | --- | --- |
| `clip` | Required | First-plane windows and output pixels |
| `trust` | 4 | Local/neighbor confidence threshold, 0–100 |
| `winx,winy` | 0, 0 | Window dimensions; zero automatic |
| `wleft,wtop` | −1, −1 | Origins; negative automatic |
| `dxmax,dymax` | −1, −1 | Peak radii; negative automatic |
| `zoommax` | 1 | One/two windows and strict scale test |
| `stab` | 1 | Integer-peak displacement penalty |
| `pixaspect` | 1 | Positive vertical displacement divisor |
| `fields,tff` | false, omitted | Field correction and optional parity override |
| `show,info` | false, false | Surface/text display |

## 7. Boundaries, missing data, and errors

Supports constant GRAY/YUV 8–16-bit integer and float32, not RGB. Only the first plane is analyzed, so chroma subsampling itself is irrelevant. Used integer samples must fit declared depth; used floats must be finite. Samples outside windows are not subject to these checks.

With fields=true, every required basic observation's current image needs parity, or tff alternating by index. Images used only as predecessors need no separate parity. Invalid motion cannot hide missing properties, bad samples, or FFT errors.

Output replaces all five Depan properties and removes `DepanEstimateFFT,DepanEstimateFFT2,DepanEstimateX,DepanEstimateY,DepanEstimateZoom,DepanEstimateGood,DepanEstimateTrust`. These old keys are not inputs. Other properties remain; info=false also preserves inherited diagnostics.

## 8. Precision and determinism

FFT uses single-precision PocketFFT with unnormalized forward/inverse transforms. A double FFT followed by float conversion is not equivalent. A fixed build/profile is repeatable; optimized FFT profiles may differ through ordinary rounding, and peak/threshold comparisons can turn small differences into discrete decisions. [KernelInfo](kernel-info.md) reports the execution configuration.

Required FFT outputs, correlations, and later arithmetic must be finite; zero divisors fail. Peak ties, accumulation order, and strict/non-strict tests matter. Diagnostic decimals use nearest-even, preserve negative zero, and store at most 127 bytes. Final text pixels also depend on the host renderer.

[Back to the English index](README.md)
