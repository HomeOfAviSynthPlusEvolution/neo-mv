# Quantized bicubic compensation

Inputs and output are as in [nearest sampling](kernel-nearest.md), with mode 2 [coordinates](kernel-sampling-coordinates.md). Define ax=trunc(fl32(256*fx)) and ay likewise, in [0,256]. For an integer a in that interval, define signed coefficients using exact integer products and truncation toward zero:

$$c_0(a)=\operatorname{trunc}{-a(256-a)^2\over8192},$$
$$c_1(a)=\operatorname{trunc}{256^3-512a^2+a^3\over8192},$$
$$c_2(a)=\operatorname{trunc}{a(256^2+256a-a^2)\over8192},\quad c_3(a)=\operatorname{trunc}{-a^2(256-a)\over8192}.$$

Let cx[e]=ce(ax), cy[f]=cf(ay), e,f in 0..3. A complete footprint requires 1<=i<W-2 and 1<=j<H-2 and consists of P(i+e-1,j+f-1). Every tap must exist, including taps with zero coefficients.

For class T, define kef=trunc(cx[e]*cy[f]/2048) independently for every tap, then

$$Q_T=\operatorname{clamp}\left(\left\lfloor{\sum_{e,f}k_{ef}P(i+e-1,j+f-1)+1024\over2048}\right\rfloor,0,M\right).$$

For classes Z/R, do not quantize the two-dimensional coefficient:

$$Q_{ZR}=\operatorname{clamp}\left(\left\lfloor{\sum_{e,f}c_x[e]c_y[f]P(i+e-1,j+f-1)\over2^{22}}\right\rfloor,0,M\right).$$

There is no rounding bias in QZR. Accumulate signed integers with sufficient width; floor on a negative sum is not truncation. Coefficients do not require renormalization to an exact sum. Do not force constant preservation for every quantized fraction.

## Near-edge bilinear value

For specified near-edge rows, use fx and fy without 1/32 quantization. T uses the constant fyT; Z uses the fraction from its current Y. Convert fractions and integer samples exactly to binary64 wherever required in the following expression. Only the two products explicitly marked fl32 are binary32:

$$B_2=\operatorname{trunc}\left((1-f_y)((1-f_x)P_{00}+\operatorname{fl32}(f_xP_{10}))+f_y((1-f_x)P_{01}+\operatorname{fl32}(f_xP_{11}))\right).$$

All remaining arithmetic is individually rounded binary64 without contraction. Here Pef=P(i+e,j+f). T stores B2; Z clamps it to [0,M] before storage. This is a separate numeric rule from mode 1's quantized bilinear sampling.

## Border dispatch

For T/Z, obtain fractions first, then reflect j vertically once. Preserve fractions after reflection. Apply the first matching row/column condition:

| Reflected row | Column | Output |
| --- | --- | --- |
| 1<=j<H-2 | 1<=i<W-2 | QT for T, QZR for Z |
| 1<=j<H-2 | 0<=i<W and i is 0,W-2 or W-1 | P(i,j) |
| 1<=j<H-2 | Other i | Ordinary horizontal edge fill with blur |
| j=0 or H-2 | 0<=i<W-1 | Near-edge B2 |
| j=0 or H-2 | i=W-1 | P(i,j) |
| j=0 or H-2 | Other i | Single-sample horizontal edge fill, no blur |
| j=H-1 | 0<=i<W, class T | P(i,j) |
| j=H-1 | 0<=i<W, class Z | floor((P(i,j)+P(i,j-1))/2) |
| j=H-1 | Other i | Single-sample horizontal edge fill, no blur |
| Any other j | Any | B |

For R, use QZR only on the original complete 4x4 domain. Otherwise apply general two-axis reflection and return its single sample or B. R has no near-edge B2, no last-row averaging and no blur.

The table order also applies to small planes. At H=2, row 0 is a near-edge row and row 1 is the bottom row. At H=3, rows 0 and 1 are near-edge rows and row 2 is the bottom row. There are no cubic middle rows in either case. On non-bottom rows at W=1, an in-range i=0 takes the explicit last-column or edge-column copy instead of a two-column interpolation. The bottom-row T/Z rules remain unchanged. For W=1, W-2=-1 is not a valid edge column; it follows horizontal edge fill, yielding B when the left bit is disabled. No tap outside the visible plane is synthesized.

## Examples

- At a=0, c=[0,2048,0,0]; at a=128, c=[-256,1280,1280,-256]; at a=256, c=[0,0,2048,0].
- At ax=ay=128, a constant 4x4 patch of 100 produces 100 in both QT and QZR. A single 100-valued sample at e=f=1 with all other taps zero gives QT=39 and QZR=39; its T coefficient is 800.
- On a 4x4 plane, an identity T map produces an exact copy in mode 2, including its bottom row. At very large dimensions the specified binary32 coordinate rounding still applies. A Z map sampling its bottom row instead averages that row with the preceding row for in-range columns. If those samples are 21 and 10, the result is 15, regardless of the vertical fraction.
- A near-edge footprint [0,10,20,31] with fx=fy=0.5 gives B2=15. A complete cubic footprint must not be inferred from that 2x2 example.
- For a 1x2 plane with P(0,0)=10,P(0,1)=21, use a Z map tx=ty=v=w=0,u=2,h=1. At output (0,0), i=j=0 gives the last-column copy 10. At output (0,1), i=0,j=1 gives the bottom-row average 15, not 21. There is no complete cubic footprint, but both outputs are defined.
