# Stabilization layer composition

Inputs are clip images, final current map Q and the optional neighbor pairs. Output is a fully initialized image with the same format and dimensions as clip[n]. Compose layers in this order: previous if prev>0; next if next>0; current always. Selected neighbors use nearest sampling regardless of subpixel. Current uses mode subpixel (0 nearest,1 bilinear,2 bicubic). A neighbor selected at n is still a separate first or second layer.

Reuse the Phase 5 [plane geometry and coordinate rules](../../phase-5/depan-compensate/kernel-sampling-coordinates.md), [border rules](../../phase-5/depan-compensate/kernel-border-sampling.md), [nearest](../../phase-5/depan-compensate/kernel-nearest.md), [bilinear](../../phase-5/depan-compensate/kernel-bilinear.md) and [bicubic](../../phase-5/depan-compensate/kernel-bicubic.md) equations. This includes per-plane conversion, translation clamps, T/Z/R classification, interpolation quantization, integer rounding and reflection order. Pass the requested blur to every layer, then perform its per-plane conversion.

## Explicit preserve result

Extend the sampler result type for this phase to either Write(sample) or Preserve. A sample is never interpreted as a sentinel. On the first existing layer, use the requested mirror bits and concrete border B=0 for Y/GRAY, B=2^(bits-1) for U/V. Every sampler result is Write, so this layer initializes every visible output sample.

On each later layer, use mirror=0 and replace every branch that would return the symbolic border B in the Phase 5 rules by Preserve. Other branches, including edge fallbacks that read a real sample, remain Write. Apply Write by replacing the destination sample; apply Preserve by retaining the previously written sample. A real source sample equal to B still overwrites. Do not substitute Preserve for an arithmetic failure, and do not mix border constants or previously rendered pixels into an incomplete interpolation footprint.

Every successful later sample overwrites earlier layers, even if the destination was already covered. There is no persistent hole mask and no averaging between temporal layers. Mirror is enabled only on the first layer, regardless of how many of its border pixels actually needed filling. All properties originate from clip[n], not from a selected fill image.

## Examples

- For a 4x4 identity T map, bilinear mode,mirror=0, a first/current-only layer writes B at (3,1), but reads P(3,3) at (3,3). If a previous layer already initialized the image, current bilinear instead preserves the previous pixel at (3,1), still overwriting (3,3). Thus prev>0 can change pixels even when its selected image is n.
- With first-layer sample 77, a later valid source sample 0 writes 0. A later symbolic-border branch preserves 77. Comparing the numeric sample with border color would be incorrect.
- A GRAY8 first nearest layer from a constant-10 image, translated +1 horizontally, produces [10,10,10,0] in each valid row of a 4-wide plane. A second constant-20 layer translated -1 with preserve borders gives [10,20,20,20]. A final constant-30 layer translated +1 gives [30,30,30,20].
- A 2x2 YUV420 input has 1x1 chroma: modes 0/1 are supported, mode 2 is a creation error. A 2x4 YUV420 input has 1x2 chroma and supports all modes under the shared edge rules.
