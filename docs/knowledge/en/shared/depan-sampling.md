# Depan sampling: coordinate classes, interpolation, and edges

Global compensation and stabilization sample integer GRAY/YUV images through the six-coefficient map `X=tx+ux+vy,Y=ty+wx+hy`. They do not use Super phases.

## 1. Plane conversion and coordinate classes

Y/GRAY and 444 chroma use the original map. For 422 chroma, divide tx, v by 2 and multiply w by 2, leaving ty unchanged. For 420, divide tx, ty by 2 and leave other coefficients unchanged. In 420/422 chroma, blur becomes floor(blur/2); 444 keeps it.

For current plane W, H, first clamp finite tx, ty independently to `±fl32(W+H+64)`. Other coefficients are not clamped. Exact comparisons select:

- T: u=h=1, v=w=0, a pure translation.
- Z: v=w=0 but not T, axis-aligned scale/translation.
- R: every other map.

Classification changes rounding and edges. An R map merely close to translation cannot use T.

## 2. Per-pixel coordinates

The following floating operations round separately to binary32.

T nearest uses `i=x+floor(fl32(tx+0.5))`, `Y=fl32(ty+fl32(y))`, `j=floor(fl32(Y+0.5))`. T interpolation uses i=x+floor(tx), fx=the fractional part of tx, j=floor(Y), fy=the fractional part of Y. Bicubic near-edge handling additionally uses constant `fyT=fl32(ty-fl32(floor(ty)))`.

Z independently computes `X=fl32(tx+fl32(u·fl32(x)))` and the Y equivalent. Nearest floors X+0.5, Y+0.5; interpolation uses floor and fractions.

R nearest/bilinear starts each row at `X0=fl32(tx+fl32(v·y)),Y0=fl32(ty+fl32(h·y))`, then advances by `X←fl32(X+u),Y←fl32(Y+w)` each column. Do not replace recurrence by multiplication. Nearest uses `trunc(fl32(X+0.5))`, including negative values; bilinear uses floor/fractions.

R bicubic instead independently computes `X=fl32(fl32(tx+fl32(u·x))+fl32(v·y))`, `Y=fl32(fl32(ty+fl32(w·x))+fl32(h·y))`, without row recurrence.

Thus T nearest at X=−0.75 selects −1 while R at that coordinate selects 0. Finite out-of-image coordinates reach border handling; nonfinite coordinates are errors, not background.

## 3. Reflection and horizontal filling

Mirror bits are top=1, bottom=2, left=4, right=8. A single reflection first replaces q<0 by −q if the low edge is enabled, then replaces the resulting q≥D by 2D−q−2 if the high edge is enabled. Each condition runs once; results may remain outside. This is not periodic reflection.

General two-axis reflection handles X/Y separately, returning border action B if still outside. R edges use this without horizontal blur.

For valid rows, T/Z use specialized horizontal filling with C(k)=clip(k, 0, W−1):

- Left i<0 with the left bit: blur=0 reads P(C(−i), j). Otherwise ell=min(blur, −i), and average k from −i−ell+1 through −i, flooring the integer result.
- Right bit enabled and i≥threshold R: q=2W−i−2. blur=0 reads P(C(q), j). Otherwise ell=min(blur, i−W+δ), averaging k=q…q+ell−1.

Ordinary filling uses R=W,δ=1. Bilinear interior-row filling uses R=W−1,δ=2. Clamp each column independently; repeated edge samples do not reduce the divisor. Disabled edges return B. “Single-sample filling” forces blur=0.

For row `[0,10,20,30,40]`, i=−2, left reflection, blur=0 gives 20 and blur=2 gives 15. At i=5, right reflection, blur=2, ordinary filling gives 30 and bilinear filling 35.

## 4. Nearest, subpixel=0

T/Z first reflect integer j vertically once. A valid row reads an in-range column directly or uses horizontal filling; an invalid row returns B. R first tries the original position, then general two-axis reflection for a single sample or B, without blur.

## 5. Bilinear, subpixel=1

Quantize fractions as `ax=floor(fl32(32fx)),ay=floor(fl32(32fy))`, including endpoint 32. Coefficients are `[32-ax,ax]` and `[32-ay,ay]`. With a complete `2×2` footprint:

$$Q=\left\lfloor\frac{\sum_{e,f=0}^1c_x[e]c_y[f]P(i+e,j+f)}{1024}\right\rfloor.$$

There is no rounding bias. Zero coefficients do not excuse absent samples.

T/Z compute fractions before reflecting j and retain those fractions. For 0≤j<H−1, complete columns use Q and others use bilinear horizontal filling. At j=H−1, legal columns copy directly; outside columns use single-sample filling for T and B for Z. Other rows return B.

R interpolates only if the original coordinate has a complete footprint. Otherwise reflect both axes and read a single sample or B, without retrying bilinear interpolation.

An identity T with mirror=0 returns B in the last column of nonbottom rows because the right neighbor is absent, but copies the bottom-right sample. Identity does not universally mean an image copy.

## 6. Bicubic, subpixel=2

Set ax=trunc(fl32(256fx)), similarly Y. For a in [0, 256], coefficients are:

$$c_0=trunc\frac{-a(256-a)^2}{8192},\quad c_1=trunc\frac{256^3-512a^2+a^3}{8192},$$
$$c_2=trunc\frac{a(256^2+256a-a^2)}{8192},\quad c_3=trunc\frac{-a^2(256-a)}{8192}.$$

A full footprint requires 1≤i<W−2, 1≤j<H−2 and reads P(i+e−1, j+f−1). T first quantizes each 2D coefficient `kef=trunc(cx[e]cy[f]/2048)`, then:

$$Q_T=clip\left(\left\lfloor\frac{\sum k_{ef}P+1024}{2048}\right\rfloor,0,M\right).$$

Z/R do not quantize 2D coefficients and use `QZR=clip(floor(ΣcxcyP/2^22),0,M)` without bias. Coefficients can be negative; use wide signed sums without renormalizing.

### Near-edge two-by-two sampling

Designated near-edge rows use fractions without 1/32 quantization. T uses constant fyT; Z uses current Y's fraction. Only marked terms below use binary32; remaining operations use separate binary64 rounding:

$$B_2=trunc\left((1-f_y)((1-f_x)P_{00}+fl32(f_xP_{10}))+f_y((1-f_x)P_{01}+fl32(f_xP_{11}))\right).$$

T stores B2 directly; Z additionally clips it. This differs from subpixel=1's quantized bilinear calculation.

### Bicubic edge dispatch

T/Z compute fractions, then reflect j vertically once:

| Row | Column | Result |
| --- | --- | --- |
| `1≤j<H-2` | `1≤i<W-2` | QT for T, QZR for Z |
| Same | Legal edge columns 0, W−2, W−1 | P(i, j) |
| Same | Other columns | Ordinary horizontal filling, with blur |
| `j=0` or `H-2` | `0≤i<W-1` | B2 |
| Same | i=W−1 | P(i, j) |
| Same | Other columns | Single-sample horizontal filling |
| j=H−1 | Legal columns, T | P(i, j) |
| j=H−1 | Legal columns, Z | floor((P(i, j)+P(i, j−1))/2) |
| j=H−1 | Other columns | Single-sample horizontal filling |
| Other rows | Any | B |

R uses QZR only for a complete original `4×4` footprint; otherwise general reflection gives one sample or B, without B2, bottom averaging, or blur.

Heights 2/3 have no bicubic interior rows but retain defined edge behavior. Width 1 is supported: the legal final column copies without reading an absent neighbor. Mode 2 requires H≥2 on every actual plane.

## 7. Border actions and examples

For DepanCompensate, B is Y/GRAY=0, U/V=2^(b−1), independent of range tags. Later DepanStabilise layers may interpret B as preserving the existing destination pixel; see that function.

Bilinear `[0,10;20,31]` at fx=fy=0.5 gives 15. Bicubic half-phase coefficients are `[-256,1280,1280,-256]`; a single sample 100 at one central coefficient position, all others zero, gives 39 for T and Z/R.

At a bottom-row sample 21 with 10 above, Z's bicubic edge gives 15 regardless of fy; T directly gives 21. These are explicit class/edge rules, not interchangeable implementations of a generic interpolation.

[Back to the English index](../README.md)
