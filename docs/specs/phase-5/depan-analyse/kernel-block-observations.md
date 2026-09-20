# Block observations for global fitting

Inputs are the creation analysis descriptor, current analysis properties, an optional first-plane mask, and clip dimensions Wc,Hc. Outputs are either ineligible, a controlled error, or row-major observations `(Xi,Yi,dxi,dyi,SADi,mi)` plus the scaled scene threshold T1. No clip or vector-carrier pixels are read.

Decode the fixed `MVUtensils` analysis prefix with the Phase 1 [decoder](../../phase-1/analyse/kernel-vector-validation.md). Creation reads vectors[0] in metadata-only decoder mode and requires valid metadata with DeltaFrame in {-1,+1}. The frame must be retrievable, but its arrays are neither required nor inspected at creation, even when present with matching counts. Save that delta for frame selection and motion-conversion direction. Compute and save numeric thresholds T1,T2 from the creation descriptor and user arguments using the [scene-threshold formulas](../../phase-1/sc-detection/kernel-scene-classification.md#thresholds). Do not rescale these thresholds from later metadata.

At evaluation, decode the selected vector frame independently in vector-data mode. There is no equality requirement between its effective scalars and the creation descriptor. Use the current scalar domains, current Nx*Ny for both array-count checks, and current geometry, padding and pel for vector-bound validation. Current InvalidMetadata or MetadataOnly is ineligible; malformed complete arrays are errors. Missing, empty or wrongly typed current scalar properties follow the decoder's zero-default rules, not inheritance from vectors[0]. In particular a current DeltaFrame of zero, including one obtained from an omitted key, is permitted by the decoder and does not replace the saved delta.

For a Complete current field, count K entries whose SAD_i>T1 over its current Nx*Ny blocks. The field is eligible exactly when fl32(K)<=T2. Both comparisons retain the scene classifier's strict rejection boundaries. Only its threshold formulas and count comparison apply here; its requirement that the current descriptor equal the threshold descriptor does not apply to DepanAnalyse. Current block size, grid, chroma, plane ratios or precision may therefore change while T1,T2 retain their creation values.

No additional temporal-reference availability condition is applied here. In particular the chosen vector frame may be zero for a positive delta at output zero. Clip geometry and sample depth need not equal the public analysis geometry and precision. Do not require a Super image or a render block-size table. Analysis geometry must satisfy the public decoder; no image sampling is performed at vector endpoints.

For block index i=by*Nx+bx, use the current field's grid, block size and overlap to form exact integer centers

$$X_i=b_x(B_x-O_x)+\lfloor B_x/2\rfloor,\qquad Y_i=b_y(B_y-O_y)+\lfloor B_y/2\rfloor.$$

Do not add analysis padding. With the current field's pel p, let q=fl32(1/p). Set dx_i=fl32(fl32(vx_i)*q), and likewise dy_i. Retain SAD_i as its exact int64 value. Xi,Yi and their integer squares must be representable in the chosen exact arithmetic; convert to float only where the fitting equations require it.

If a mask is supplied, it has constant format/dimensions, the same Wc,Hc as clip, at least clip's frame count, integer 8-bit samples, and a full-resolution first plane. Other mask planes are ignored. Every output n requires that mask[n] be retrievable, including when the selected vector field is ineligible; a mask-frame failure is a frame error. Only an eligible field reads mask samples for fitting. At output n, mi is that first plane's byte sample at (Xi,Yi) when 0<=Xi<Wc and 0<=Yi<Hc. Otherwise mi=1. Without a mask mi=1 everywhere. Mask values are weights 0..255, not values divided by 255.

The initial fitting weight is bi=mi for every block. Border exclusion and SAD/neighbor rejection occur only after the first fitting update. A supplied all-zero mask is distinct from an omitted mask, including the border exclusion width defined by [weight selection](kernel-block-weights.md).

## Examples

- Bx=By=8, Ox=Oy=0 yields centers (4,4),(12,4),... independently of padding. At p=2, vector (-3,2) becomes (-1.5,1).
- On a 16x16 clip, a center (20,4) receives weight 1 even with a supplied mask. A center (12,4) reads mask[12,4]; value 128 yields weight 128.
- Complete zero vectors with zero SAD and a zero mask are eligible if scene thresholds permit them. Missing vector arrays are ineligible, not a zero observation set.
- With creation DeltaFrame=-1, a current DeltaFrame of +1 or zero, or an omitted current DeltaFrame, does not change frame selection or motion-conversion direction. Valid current HPad=17 is accepted after creation HPad=16, provided the current arrays satisfy the current bounds. A current missing Pel becomes zero and makes the field ineligible before array values are inspected.
- With a creation 2x2 grid, 8x8 blocks, no chroma, depth 8, thscd1=400 and thscd2=51, retain T1=400 and T2=fl32(2.04). A valid current 3x2 grid requires six entries in each array: four-entry arrays are MetadataOnly, not interpreted using the creation grid. Six zero vectors with zero SAD are eligible. Six valid vectors with SAD [401,401,401,0,0,0] are ineligible because K=3 exceeds the saved T2; do not recompute T2 as fl32(3.06). Changing only the current depth to 10 also does not rescale T1: four valid vectors with SAD=500 are ineligible under the same creation thresholds.
- With valid metadata but absent vector arrays, a supplied mask[n] that fails to produce a frame makes output n fail. A retrievable mask permits the ineligible branch without reading fitting samples, even if all of its samples are zero.
- Valid vectors[0] metadata with four blocks and four-element float vector/SAD arrays passes the metadata-only creation read. A request selecting that frame fails the vector-data read because matching arrays must have integer elements. If the arrays are absent instead, creation still succeeds and a request selecting the frame produces an ineligible fitting input; the plugin's final error comparison determines output motion validity.
