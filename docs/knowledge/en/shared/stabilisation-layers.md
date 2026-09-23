# Stabilization: neighbor selection and layer overwriting

After [DepanStabilise](../depan-stabilise.md) determines current correction Q, it selects previous/next layers independently. Selection uses motion and frame indices, not image samples. See [temporal smoothing](stabilisation-smoothing.md) for M and the specific inverse Inv.

## Previous layer

With prev=0 there is no layer. Otherwise p=max(b, n−prev), initially best=p, score=10000, U=Q. Visit k=n−1, n−2,…, p:

$$U\leftarrow M(m_{k+1})\circ U.$$

Convert U to motion, then calculate:

$$s=((|dx|+|dy|)+\operatorname{fl32}(n))-\operatorname{fl32}(k).$$

Only s<score changes the best image. Return **the best image with the final accumulated U**, not its map at the winning candidate. If no candidate improves, retain p. If p=n, return (n, Q).

For n=2, b=0, prev=2, Q=I and both steps dx=1, frame 1 scores 2 and frame 0 scores 4. Select image 1, but pair it with final translation +2.

## Next layer

With next=0 there is no layer. Otherwise e=min(n+next, F−1). First decode every motion n+1…e in ascending order, including entries after invalid motion; any malformed data fail.

Initialize best=e, score=1000, U=Q. Visit that range again. At invalid motion, set best=k−1 and stop, overriding any previous winner. Otherwise:

$$U\leftarrow U\circ Inv(M(m_k)),$$

convert to motion and calculate:

$$s=((|dx|+|dy|)+\operatorname{fl32}(k))-\operatorname{fl32}(n).$$

Update only on strict improvement and pair the selected image with final U. An empty range returns (n, Q). Both sides start independently from Q.

From n=2, next=3, valid +1 motions at 3, 4 and invalid motion at 5 initially favor image 3, but invalidity changes selection to image 4 with translation −2. Even if motion 3 is already invalid, malformed motion 5 still fails the full preliminary decode.

Convert absolute frame indices separately to float32 before subtraction; at large indices this differs from converting an exact integer distance.

## Pixel layer order

Render previous if present, then next if present, then current. Neighbor layers always use nearest sampling; current uses subpixel. A selected neighbor image equal to current n is still an independent layer.

All layers use [Depan sampling](depan-sampling.md), including plane conversion, translation clamps, T/Z/R classification, reflection order, and edge fallback. Pass blur to every layer before plane conversion.

The first layer uses requested mirror bits and concrete background Y/GRAY=0, chroma=2^(bits−1), initializing every pixel. Later layers use mirror=0 and return either:

- `Write(sample)`: a real sample or defined edge fallback, replacing the destination.
- `Preserve`: a branch that would have returned background, retaining the earlier destination.

Real zero or neutral-chroma samples still overwrite. Zero is not a hole sentinel. Successful later layers always replace earlier pixels; there is no temporal averaging, incomplete-footprint blending, or treating arithmetic failure as Preserve.

For a valid width-4 row: first constant-10 layer at +1 writes `[10,10,10,0]`; second constant-20 layer at −1 gives `[10,20,20,20]`; current constant-30 layer at +1 gives `[30,30,30,20]`.

Identity also follows edge rules. In a `4×4` T bilinear image, (3, 1) takes the border branch, while (3, 3) reads a real bottom-row sample. Current-only fills the former with background; with an earlier layer it preserves that pixel. Thus adding a neighbor layer that selects current n can still alter edges.

All properties come from clip[n], not selected fill images. Recovered inertial maps follow the same layer composition.

[Back to the English index](../README.md)
