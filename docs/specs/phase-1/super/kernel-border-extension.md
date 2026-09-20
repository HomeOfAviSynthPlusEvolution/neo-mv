# Nearest-edge extension

Inputs: immutable source plane S of actual size w,h; working size ww>=w,hh>=h; nonnegative hp,vp. Output: a=(ww+2hp) by c=(hh+2vp) plane B of the same sample type.

For all 0<=u<a and 0<=v<c,

$$B(u,v)=S(\min(w-1,\max(0,u-hp)),\min(h-1,\max(0,v-vp))).$$

This one rule covers left/top padding, the working extension on the right/bottom, and right/bottom padding. Values are copied exactly, without rounding, clipping, colour conversion or range remapping. The source must be nonempty. For a reduced level, its freshly computed working image is the source; its actual and working sizes are identical for this operation.

Output must not overlap source. No scratch is mathematically required. Read only actual source samples; write the entire logical output rectangle. Repeated/concurrent calls must not modify the source or another output.

Example: source [10,20,30], working width 4, hp=1, height 1 and vp=0 produces [10,10,20,30,30,30]. A 1x1 source with value 7 produces only 7, for any valid working size and padding.

Consumers: integer Super phases, reduced levels, and each externally supplied fractional phase.
