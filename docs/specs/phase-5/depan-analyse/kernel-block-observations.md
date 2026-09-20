# Block observations for global fitting

Inputs are the creation analysis descriptor, current analysis properties, an optional first-plane mask, and clip dimensions Wc,Hc. Outputs are either ineligible, a controlled error, or row-major observations `(Xi,Yi,dxi,dyi,SADi,mi)` plus the scaled scene threshold T1. No clip or vector-carrier pixels are read.

Decode the fixed `MVUtensils` analysis prefix with the Phase 1 [decoder](../../phase-1/analyse/kernel-vector-validation.md). Creation requires valid metadata and DeltaFrame in {-1,+1}; arrays need not be available. Save all effective scalars. Current InvalidMetadata is ineligible. With current valid metadata, every saved scalar except Levels must match, even if arrays are unavailable; a mismatch is a frame error. Levels may vary in its positive domain. Matching MetadataOnly is ineligible; malformed complete arrays are errors. Complete data is eligible only if the [scene classifier](../../phase-1/sc-detection/kernel-scene-classification.md) gives change=0 using the creation descriptor.

No additional temporal-reference availability condition is applied here. In particular the chosen vector frame may be zero for a positive delta at output zero. Clip geometry and sample depth need not equal the public analysis geometry and precision. Do not require a Super image or a render block-size table. Analysis geometry must satisfy the public decoder; no image sampling is performed at vector endpoints.

For block index i=by*Nx+bx, form exact integer centers

$$X_i=b_x(B_x-O_x)+\lfloor B_x/2\rfloor,\qquad Y_i=b_y(B_y-O_y)+\lfloor B_y/2\rfloor.$$

Do not add analysis padding. Let q=fl32(1/p). Set dx_i=fl32(fl32(vx_i)*q), and likewise dy_i. Retain SAD_i as its exact int64 value. Xi,Yi and their integer squares must be representable in the chosen exact arithmetic; convert to float only where the fitting equations require it.

If a mask is supplied, it has constant format/dimensions, the same Wc,Hc as clip, at least clip's frame count, integer 8-bit samples, and a full-resolution first plane. Other mask planes are ignored. At output n, mi is that first plane's byte sample at (Xi,Yi) when 0<=Xi<Wc and 0<=Yi<Hc. Otherwise mi=1. Without a mask mi=1 everywhere. Mask values are weights 0..255, not values divided by 255. Only an eligible field needs mask samples; an ineligible field does not acquire a mask-frame dependency.

The initial fitting weight is bi=mi for every block. Border exclusion and SAD/neighbor rejection occur only after the first fitting update. A supplied all-zero mask is distinct from an omitted mask, including the border exclusion width defined by [weight selection](kernel-block-weights.md).

## Examples

- Bx=By=8, Ox=Oy=0 yields centers (4,4),(12,4),... independently of padding. At p=2, vector (-3,2) becomes (-1.5,1).
- On a 16x16 clip, a center (20,4) receives weight 1 even with a supplied mask. A center (12,4) reads mask[12,4]; value 128 yields weight 128.
- Complete zero vectors with zero SAD and a zero mask are eligible if scene thresholds permit them. Missing vector arrays are ineligible, not a zero observation set.
- A readable changed DeltaFrame is an error even if arrays are missing. Invalid current metadata is ineligible and does not replace the saved delta used for frame selection.
