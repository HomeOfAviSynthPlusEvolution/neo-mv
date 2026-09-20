# Quantized bilinear compensation

Inputs and output are as in [nearest sampling](kernel-nearest.md), with mode 1 [coordinates](kernel-sampling-coordinates.md). Define integer coefficients

$$a_x=\lfloor\operatorname{fl32}(32f_x)\rfloor,\quad a_y=\lfloor\operatorname{fl32}(32f_y)\rfloor,\quad c_x=[32-a_x,a_x],\quad c_y=[32-a_y,a_y].$$

Both quantized fractions are in [0,32], including a fraction rounded to 1. With a complete 2x2 footprint at i,j, output

$$Q(i,j)=\left\lfloor{\sum_{e=0}^1\sum_{f=0}^1 c_x[e]c_y[f]P(i+e,j+f)\over1024}\right\rfloor.$$

All products/sums are exact integers. There is no +512 rounding term. The complete footprint must exist even when some coefficients are zero. Do not read a missing zero-weight tap.

## Border dispatch

For T/Z, compute fractions before reflecting j vertically. Preserve those fractions after reflection. Then:

| Reflected row | Column | Output |
| --- | --- | --- |
| 0<=j<H-1 | 0<=i<W-1 | Q(i,j) |
| 0<=j<H-1 | Otherwise | Bilinear horizontal edge fill, R=W-1,delta=2, with blur |
| j=H-1 | 0<=i<W | P(i,j), no interpolation |
| j=H-1 | Outside, class T | Single-sample horizontal edge fill, no blur |
| j=H-1 | Outside, class Z | B, no horizontal reflection |
| Any other j | Any | B |

For R, apply Q only if the original i,j have a complete 2x2 footprint. Otherwise use general two-axis reflection and return its single sample or B. Do not interpolate again after reflection. R ignores blur.

## Examples

- Within a larger valid plane, a 2x2 footprint with row-major samples [0,10,20,31] and fx=fy=0.5 produces floor(61/4)=15.
- fx=1/64 gives ax=0, so horizontal interpolation uses only the left value; footprint admission still needs both columns.
- On a 4x4 identity T map, mirror=0: output (3,1) is B because its 2x2 footprint is incomplete and the right bit is disabled. Output (3,3) is P(3,3) by the last-row rule. This applies even when identity arose from invalid motion.
- With that identity and right reflection enabled, output (3,1)=P(3,1). There is no generic identity-copy shortcut when mirror=0.
