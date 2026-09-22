# Motion interval and property dependencies

Inputs are n in [0,F), normalized parameters and a data clip of at least F frames. Outputs are bounds b,h and the decoded tuples needed by smoothing. The tuple at index zero is always the synthetic invalid motion (0,0,0,1,0). Do not read or validate data[0]'s properties, even if the caller supplied them. At every other visited index decode all five [motion properties](../../phase-5/depan-analyse/kernel-motion-properties.md); malformed data is an error even if its goodmotion value is zero.

## Inertial interval, method=0

Compute t=(10*fps)/cutoff in binary32. This one intermediate may overflow to positive infinity. If !(t<fps*5), replace it by fps*5; the cap must be finite. Set

$$b_0=\max(0,\operatorname{trunc}(\operatorname{fl32}(n)-t)),\qquad h=n.$$

The subtraction must be finite and truncation representable in signed int32. If b0>n, return a frame error; do not silently change the float-derived lower bound. Starting at n, visit indices in decreasing order through b0. Stop at the first invalid tuple and set b to that index; otherwise b=b0. A malformed visited tuple fails before its validity can be used. Do not decode earlier indices beyond a found scene boundary solely for this operation.

When b=n, the final current-frame map is Z(z0). Skip cumulative smoothing, end taper and correction limits. This branch still renders pixels, applies any requested neighbor layers and constructs diagnostics.

## Symmetric interval, method=1

Start with b0=max(0,n-r),h0=min(F-1,n+r), using exact integers. Scan backward from n through b0 as above to obtain b1. Independently scan forward from n+1 through h0: stop at the first invalid index j and set h1=max(j-1,n); otherwise h1=h0. A bad frame exactly at h0 is excluded too. The forward scan is still required when the backward scan stopped immediately at n.

Then set k=min(n-b1,h1-n), b=n-k,h=n+k. This symmetrization may discard successfully decoded frames. It does not erase an earlier decoding error. Even b=h=n goes through window smoothing and map-to-motion-to-map conversion.

## Further dependencies

Previous fill uses motion already present in [b,n], after any method-0 hard reset of b. Next fill additionally decodes every data tuple from n+1 through min(n+next,F-1), in ascending order, before selection. This can require tuples beyond the smoothing interval or beyond a scene boundary. See [neighbor selection](kernel-border-selection.md).

The public result depends on clip[n] for properties and the current layer, plus the selected fill images. Scheduling may prefetch a superset of candidate frames, but may not impose extra motion-property validation on unvisited indices. Required input frame failures remain errors; caching must not change which property values are logically decoded.

## Examples

- With fps=25,cutoff=1,n=140, the capped lookback is 125, giving b0=15. If the closest bad tuple is at 120, b=120; a malformed tuple at 119 is not inspected by smoothing.
- With method=1,n=5,r=3,F=12 and bad indices 3 and 8, the scans produce b1=3,h1=7, then b=3,h=7. Cumulative motion begins with identity at 3; neither bad tuple is composed.
- With n=0, the backward scan uses synthetic invalid motion. Method 0 skips smoothing. Method 1 still performs its forward scan before shrinking to [0,0].
- A malformed index after the first forward bad tuple is ignored by the method-1 scan, but is an error if next fill's full decode range reaches it.
- Float conversion can round a large n upward. fps=25,cutoff=2500,n=16777219 gives b0=16777220 and is a controlled frame error.
