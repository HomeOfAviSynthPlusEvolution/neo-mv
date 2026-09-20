# External integer vector scaling

The typed [analysis format](data-format.md) can be transformed independently of the carrier's visible pixels. Readers must accept valid external data without requiring a particular producer identity. This describes an interchange operation, not an additional exported plugin function.

Inputs: valid analysis metadata and, optionally, a complete vector/error field; uniform integer scale s>=1. Output: the same carrier video and unrelated properties, with these replacements:

| Fields | Result |
| --- | --- |
| Width, Height, RealWidth, RealHeight | Multiply by s |
| BlkSizeX/Y, OverlapX/Y, HPad, VPad | Multiply by s |
| Each decoded vector component | Multiply by s; repack as signed-32 components |
| Each stored SAD | Multiply by s squared |
| Pel, Levels, Chroma, XRatioUV/YRatioUV, NBlkX/Y, DeltaFrame, BitsPerSample | Preserve |

All field suffixes above have the Analysis prefix from the data-format table. Metadata is scaled even on frames with no vectors. Missing arrays are not manufactured. Scale results must fit canonical signed-32 geometry/components and nonnegative int64 SAD; reject overflow rather than wrapping or relying on the reader's scalar saturation.

The unchanged carrier dimensions are not the analysis dimensions. A subsequent image operation supplies separately generated samples matching the scaled metadata. Inherited old Super data on the vector carrier must not override those explicit samples or the modified Analysis properties. Recalculate additionally requires matching analysis/sample precision.

Example: W=Wr=32,H=Hr=16, block 8x8, overlap 4x4, pad 4x4, grid 7x3, pel=2, vector (-3,2), SAD=7. For s=2, dimensions become 64x32, block 16x16, overlap/pad 8x8; grid and pel stay unchanged. Vector (-6,4) packs to 21474836474, and SAD=28. The carrier remains 32x16. This transformation does not assert equality to a fresh full-resolution motion analysis.
