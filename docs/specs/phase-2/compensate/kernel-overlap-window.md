# Separable block-overlap windows

Inputs: positive block length B, overlap 0<=O<=floor(B/2), number of blocks N>=1 along an axis, and block index k in [0,N). Output: B real window coefficients, then a quantized two-dimensional window when both axis windows are supplied. Plane-specific B,O are used; the block count is unchanged by chroma subsampling.

## Axis window

For O=0, w(i)=1 for all 0<=i<B. Otherwise define the interior-block window

$$w_m(i)=\begin{cases}
\cos^2\left(\frac{\pi(i-O+1/2)}{2O}\right)&0\le i<O,\\
1&O\le i<B-O,\\
\cos^2\left(\frac{\pi(i-B+O+1/2)}{2O}\right)&B-O\le i<B.
\end{cases}$$

Set w(i)=1 on the left shoulder i<O when k=0, and on the right shoulder i>=B-O when k=N-1. Else use wm(i). These conditions apply independently: a single block is both first and last and has w(i)=1 throughout. The two axes use the same rule. No choice of evaluation order changes the boundary conditions.

For the scalar baseline use pi32=fl32(pi). For a shoulder with integer z=i-O or i-B+O, evaluate u=fl32(fl32(pi32*fl32(z+0.5))/fl32(2O)), c=fl32(cos(u)), and w=fl32(c*c). The [common cosine tolerance](../README.md#common-contracts) applies. Exact outer-edge replacements remain 1.

## Two-dimensional coefficients

For local (i,j), combine the plane's X and Y windows before quantization:

$$W(i,j)=\operatorname{trunc}(\operatorname{fl32}(\operatorname{fl32}(\operatorname{fl32}(w_y(j)w_x(i))\,2048)+0.5)).$$

W is an integer in [0,2048]. Do not separately quantize axis weights. The output is immutable coefficient data; no physical table arrangement or allocation strategy is prescribed. All sizes and conversions must be representable.

## Examples

- B=8,O=0 gives all ones. With no taper on either axis, every two-dimensional coefficient is 2048.
- B=4,O=1 gives an interior axis approximately [0.5,1,1,0.5]; k=0 of N=2 gives [1,1,1,0.5], and k=1 gives [0.5,1,1,1]. An untapered other axis produces integer shoulder weight 1024. Two such shoulder weights in different axes produce 512.
- N=1 gives all ones for any valid O, including O=B/2. A one-block-by-one-block grid therefore uses 2048 everywhere even if its metadata declares nonzero overlap.

Consumer: [block composition](kernel-overlap-composition.md). Coefficients may be shared read-only among concurrent calls with identical geometry.
