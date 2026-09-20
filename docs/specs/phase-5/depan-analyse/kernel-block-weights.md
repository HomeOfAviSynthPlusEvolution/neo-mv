# Global-fit weight selection

Inputs are [observations](kernel-block-observations.md), the newly updated map, the Nx by Ny grid, scaled SAD threshold T1, wrong, zerow, global residual threshold G, and whether a mask node was supplied. Output is one binary32 weight bi per block. Weights replace the previous set; rejection is not permanent.

Let L=0 with a mask, otherwise L=4. For each block at grid column bx and row by, reject if any of the following applies, in this order:

1. bx<L, bx>=Nx-L, by<L or by>=Ny-L.
2. SAD_i>T1.
3. The block has all eight neighbors and its horizontal observation differs from their arithmetic mean by more than wrong.
4. The same test for the vertical observation.
5. abs(ex_i)>G or abs(ey_i)>G, using the updated map and the residual expression in [global fitting](kernel-global-fit.md).

An eight-neighbor mean is accumulated in binary32 in the order upper-left, upper, upper-right, left, right, lower-left, lower, lower-right, then divided by 8. Start with the first value; add the other seven in that order. Edge blocks without eight neighbors skip both neighbor tests. These tests use the original observations, not their weights or fitted vectors. All comparisons are strict `>`.

A rejected block has bi=+0. Otherwise, if dxi=0 and dyi=0, set bi=zerow*mi; else set bi=mi. Do not normalize by total weight or by 255. Evaluate tests in order and stop at the first rejection; arithmetic in later tests is unused. wrong and zerow may be negative finite values. Their effects follow these rules; arithmetic failure in a later fitting update remains a controlled error.

## Examples

- Without a mask, a 9x9 grid retains at most the center block (4,4) after border rejection; an 8x8 grid retains none. The first fit update still used all initial weights.
- With a mask, an edge block with SAD=T1, zero observations, zero residual and mi=100 has bi=5 for zerow=0.05. SAD=T1+1 instead gives zero.
- Interior dx=2 with all eight neighboring dx=0 passes wrong=2 and fails wrong=1.99. A block can regain its base weight in a later pass when its updated global residual passes.
- A supplied mask with every sample zero and every block center inside its bounds gives zero initial and subsequent weights but does not itself mean invalid motion; the fit's regularizing terms still determine an error value. An out-of-mask center instead has base weight 1 and does not satisfy this example's premise.
