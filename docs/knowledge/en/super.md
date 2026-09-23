# Super: working images, pyramids, and subpixel phases

## 1. What the function computes

`Super` prepares auxiliary images for later calculations: a working image covering the entire block grid, extended borders, progressively reduced integer-position images, and subpixel sampling phases at the finest level. The visible output remains the input image.

A *phase* represents a fixed fractional sampling offset. With `pel=2`, there are four logical images: integer positions, horizontal half-pixels, vertical half-pixels, and half-pixels on both axes. Consumers select a phase and integer position from a vector instead of repeating the interpolation.

## 2. Objects and notation

The original luma dimensions are `W,H`, block dimensions `Bx,By`, overlap `Ox,Oy`, and luma padding `px,py`. Chroma ratios `rx,ry` give luma pixels per chroma pixel; GRAY uses 1, 1.

For integer depth b, the maximum is `M=2^b-1`. Define:

$$R_k(z)=\lfloor(z+2^{k-1})/2^k\rfloor,\quad K(z)=\min(M,\max(0,z)).$$

`Rk` divides with rounding; `K` clips a signed filter result to the sample range. `A(a,b)=R1(a+b)` is a two-sample average; `D(a,b,c,d)=R2(((a+b)+c)+d)` is one four-sample average. Float32 divides without the bias, omits `K`, and rounds each operation to binary32.

## 3. Overall calculation

1. Determine working dimensions and level count from block geometry.
2. Extend the original image over the working region and surrounding padding.
3. Reduce integer-position images level by level, extending each new level.
4. Generate fractional phases at the finest level, or extract them from `pelclip`.
5. Preserve the visible image and attach geometry properties and auxiliary samples.

## 4. Step-by-step calculation

### 4.1 Working dimensions from block coverage

Block steps are `sx=Bx-Ox,sy=By-Oy`. Complete coverage requires:

$$N_x=\left\lceil\frac{W-O_x}{s_x}\right\rceil,\quad W_0=N_xs_x+O_x,$$
$$N_y=\left\lceil\frac{H-O_y}{s_y}\right\rceil,\quad H_0=N_ys_y+O_y.$$

An `18×10` image with `8×8` blocks and `4×4` overlap has a `4×2` grid and `20×12` working dimensions. Extra samples copy the nearest edge; the original image is not resized.

For plane ratios `ux,uy`, real dimensions are `W/ux,H/uy`, working dimensions `W0/ux,H0/uy`, and padding `hp=floor(px/ux),vp=floor(py/uy)`. Positive luma padding can correspond to zero chroma padding.

### 4.2 Level count and dimensions

One dimension is reduced by:

$$F(d,r,p)=r\left\lfloor\frac{\lfloor d/r\rfloor+\mathbf1_{p\ge r}}2\right\rfloor.$$

Starting from real luma dimensions `W,H`, repeatedly reduce using global chroma ratios and luma padding. Count consecutive reduced sizes still at least as large as the block; call the count t. The level count is `max(1,t)`, without adding another level for the original. `onelevel=true` forces one level. GRAY `64×64`, block 8, padding 16 has passing candidates 32, 16, 8, then failing 4. It stores three levels of working sizes 64, 32, 16.

Each plane's level 0 begins with its working dimensions. Subsequent levels use `F`; even U/V use global `rx,ry`, but their own padding. Padding stays constant across levels. Dimensions must remain positive. A next level `w',h'` also requires `2w'≤w+hp,2h'≤h+vp` and sufficient samples for the actual filter footprint.

### 4.3 Nearest-edge extension

For original plane `S`, real size `w,h`, and working size `ww,hh`, the padded output is:

$$B(u,v)=S(\min(w-1,\max(0,u-h_p)),\min(h-1,\max(0,v-v_p))).$$

This handles top/left padding, right/bottom working extension, and right/bottom padding together. Samples are copied without normalization or color conversion. Reduced levels apply the same extension to their newly computed working images.

### 4.4 Reducing the integer-position pyramid

Here `S(0,0)` is the working-image origin, not the padding origin.

`rfilter=0` averages a neighboring `2×2` region:

$$T(x,y)=D(S(2x,2y),S(2x+1,2y),S(2x+1,2y+1),S(2x,2y+1)).$$

Integers round once. Input `[0,0;0,1]` gives 0; averaging the rows separately and then averaging them would incorrectly give 1.

`rfilter=1` filters vertically, then horizontally. Interior offsets `-1,0,1,2` use weights `1,3,3,1` and rounded division by 8. The vertical pass produces intermediate samples `V` of width `2w`, height `h`, already rounded to sample precision. The horizontal pass filters `V`. First/last destination rows and columns use corresponding two-sample averages, including a single-row or single-column result.

`rfilter=2` has the same pass order and intermediate rounding. Interior offsets are `-2,-1,0,1,2,3`, weights `1,5,10,10,5,1`, and division by 32, grouped as `R5(a+((f+10(c+d))+5(b+e)))`. Borders still use two-sample averages. All weights are positive, unlike the signed subpixel filters below.

For a large image with only `S(4,4)=10`, integer `rfilter=1` at destination `(2,2)` first gives `V(4,2)=R3(30)=4`, then `R3(12)=2`. `rfilter=2` gives `R5(100)=3`, then `R5(30)=1`. Float32 gives 1.40625 and 0.9765625 respectively.

### 4.5 Half-pixel phases

Interpolation covers the entire padded plane. For a sequence z of length m, copy the original sample at the last position. Other positions use:

| `sharp` | Interior formula | Interior positions |
| --- | --- | --- |
| 0 | `A(z(i),z(i+1))` | All `i<m-1` |
| 1 | `K(R4(9(z(i)+z(i+1))-z(i-1)-z(i+2)))` | `1≤i<m-3` |
| 2 | `K(R5(z(i-2)+z(i+3)+20(z(i)+z(i+1))-5(z(i-1)+z(i+2))))` | `2≤i<m-4` |

Other nonfinal positions use a two-sample average. Negative coefficients can produce out-of-range integer results, so clip after rounding, not during accumulation. Each padded plane must be at least 2, 4, 6 samples wide and high for `sharp=0,1,2` respectively.

Let `H,V` be horizontal/vertical operators. For `sharp=0`, the phase `J` offset by half a pixel on both axes uses one four-sample average from the original image. Its last column uses a vertical pair, last row a horizontal pair, and bottom-right corner a copy. For `sharp=1/2`, `J=H(V(B))`, retaining vertical rounding and clipping.

The `pel=2` phases are `P00=B,P10=H(B),P01=V(B),P11=J`.

Float32 preserves expression grouping: four-tap horizontal numerator `-(a+d)+9(b+c)`, vertical numerator `((-a)-d)+9(b+c)`, and six-tap numerator `a+(f+5(4(c+d)-(b+e)))`. Divide by 16 or 32 respectively, without clipping.

### 4.6 Quarter-pixel phases

For `pel=4`, name integer and half-pixel phases `P00,P20,P02,P22`. Generate the rest with two-sample averages. `+x/+y` means one integer position right/down in the indicated phase:

| Phase | Calculation | Defined region for padded dimensions `a,c` |
| --- | --- | --- |
| `P10` | `A(P00,P20)` | Entire plane |
| `P12` | `A(P02,P22)` | Entire plane |
| `P01` | `A(P00,P02)` | Entire plane |
| `P21` | `A(P20,P22)` | Entire plane |
| `P11` | `A(P01,P21)` | Entire plane |
| `P30` | `A(P00(+x),P20)` | `u<a-1` |
| `P32` | `A(P02(+x),P22)` | `u<a-1` |
| `P31` | `A(P01(+x),P21)` | `u<a-1` |
| `P03` | `A(P00(+y),P02)` | `v<c-1` |
| `P23` | `A(P20(+y),P22)` | `v<c-1` |
| `P13` | `A(P03,P23)` | `v<c-1` |
| `P33` | `A(P03(+x),P23)` | Both restrictions |

Each average rounds separately. Phases needing right/bottom neighbors can be undefined in the final column/row. They are not implicitly filled by another copy operation. Later analysis must keep every required candidate's entire block away from undefined positions.

### 4.7 External subpixel images

With `pelclip` and `pel>1`, extract each nonzero phase from enlarged input `E`:

$$P_{a_x,a_y}(x+h_p,y+v_p)=E(pel\,x+a_x,pel\,y+a_y).$$

Extract the real-image region first, then extend each phase independently by nearest-edge copying. External phases cover the entire working and padded region. Integer phases still come from `clip`; `sharp` does not generate external phases. Coarse levels still reduce original integer-position images.

`pelclip` must have dimensions `pel` times the original, the same frame count, and the same format. With `pel=1`, only constant dimensions and matching format are checked. Its frames are not requested, and its dimensions/frame count need not satisfy the enlargement relationship.

## 5. A complete numerical example

Take a `4×4` GRAY8 image whose rows are `[10,14,18,22]`, block 4, overlap 0, padding 1, and `onelevel=true,pel=2,sharp=0`.

Working size stays `4×4`; padded size is `6×6`. Every integer-phase row is `[10,10,14,18,22,22]`. Identical rows mean vertical padding introduces no new values.

At padded coordinate `(1,1)`, the integer phase is 10, horizontal half-phase `(10+14)/2=12`, vertical half-phase 10, and two-axis half-phase `(10+14+10+14)/4=12`. The four values are `10,12,10,12`.

Visible output remains `4×4`. Attached Super data describes working size 4, padding 1, pel 2, one level, and these auxiliary samples.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip` | Required; supported constant GRAY/YUV, integer 8–16 or float32 | Original samples and visible output |
| `blksize` | Required; supported pairs in [Analyse](analyse.md) | Working coverage and level count |
| `overlap` | Required; each axis 0 through half a block, chroma-aligned | Block steps |
| `pad` | `[16,16]`; both luma values positive | Extension and reduction geometry |
| `onelevel` | false | Keep only level 0 |
| `rfilter` | 1; 0–2 | Reduction filter |
| `sharp` | 2; 0–2 | Built-in half-pixel filter |
| `pel` | 2; 1, 2, 4 | `pel²` phases at level 0 |
| `pelclip` | Omitted | Replace noninteger phases |
| `prefix` | `MVUtensils` | Super property/data names |

One axis value is copied to both axes; two mean X/Y; more than two is an error. Explicit empty arrays for `blksize/overlap/pad` fall back to `8,8`, `0,0`, and `16,16`. Omitting a required parameter remains an error.

## 7. Boundaries, missing data, and errors

Images must not be smaller than blocks. Dimensions, blocks, and overlap must meet chroma alignment. Every filter footprint must exist; allocated storage does not make undefined `pel=4` positions valid.

Visible samples pass through unchanged. Auxiliary data remains valid across frame copies and concurrent readers. New results replace old Super data with the same prefix; other properties remain. Only the current source frame and, when needed, the same-index `pelclip` frame are requested.

## 8. Precision and determinism

Two two-sample averages differ from one four-sample average. Intermediate pass rounding must remain. Integer interpolation keeps signed intermediates until explicit clipping; floating samples are not restricted to `[0,1]`. Logical levels/phases follow these equations independently of storage stacking, row buffers, or execution order.

[Back to the English index](README.md)
