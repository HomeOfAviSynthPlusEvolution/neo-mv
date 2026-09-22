# Neighbor image and map selection

Inputs are final current-frame map Q, n,F,final b, prev,next and decoded motions. Outputs are zero, one or two ordered pairs (image index,map). Selection is independent of image samples. Use full-resolution geometry for all map-to-motion conversions, forward=true. Scores use binary32 arithmetic, including conversion of each absolute frame index separately; do not replace their differences by an exact integer distance before conversion. Strict improvement selects a candidate; ties retain the earlier selection.

## Previous layer

Skip this operator if prev=0. Otherwise let p=max(b,n-prev), best=p,score=10000,U=Q. Visit k=n-1,n-2,...,p. On each visit:

$$U\leftarrow M(m_{k+1})\circ U,$$

convert U to motion dx,dy,r,z and compute

$$s=((|dx|+|dy|)+\operatorname{fl32}(n))-\operatorname{fl32}(k).$$

If s<score, set score=s,best=k. Return (best,U) **using the final accumulated U**, not the U at the winning candidate. If p=n, the loop is empty and the result is (n,Q). If no score is below 10000, best remains p. No motion at p itself is composed.

## Next layer

Skip if next=0. Otherwise let e=min(n+next,F-1), using exact integers. First decode all data tuples from n+1 through e in ascending order, including entries after any invalid-motion tuple. Malformed data anywhere in that range is an error. Set best=e,score=1000,U=Q, then visit k=n+1,...,e:

- If mk is invalid, set best=k-1 and stop, retaining the map accumulated so far. This assignment overrides any earlier winning index.
- Otherwise set U=U composed with Inv(M(mk)), convert U to motion, and calculate s=((|dx|+|dy|)+fl32(k))-fl32(n). If s<score, replace score and best.

Return (best,U) using the final map. An empty range gives (n,Q). If all tuples are valid and no score is below 1000, best remains e. Each side starts independently from Q; next does not start from the previous layer's accumulated map.

## Examples

- n=2,b=0,prev=2,Q=I and both steps dx=1: k=1 has U translation +1 and score 2, selecting image 1. k=0 has translation +2 and score 4. The returned pair is (1,translation +2), not (1,+1).
- n=2,next=3,F>=6,Q=I with dx=1 at indices 3,4 and invalid motion at 5: scores at 3,4 are 2,4, initially choosing 3. The invalid tuple then sets best=4. Return image 4 with translation -2.
- In that example, a missing property at index 5 is an error, not an invalid tuple. With an invalid tuple at 3 and malformed index 5, full next-range decoding still fails.
- A hard reset setting b=n makes previous selection return (n,Q), even when prev is large. The requested previous layer still exists and participates in layer order.
