# DepanCompensate

Samples a temporally selected frame using the motion accumulated between it and the output frame. Parameters in interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| clip | Required node | Constant integer GRAY/YUV420/YUV422/YUV444, b=8..16 |
| data | Required node | Depan motion properties; at least as many frames as clip |
| offset | 0.0 | Finite binary32 in [-10,10]; positive selects an earlier source |
| subpixel | 2 | Int64 converted by saturation to int32, then required in [0,2] |
| pixaspect | 1.0 | Finite positive binary32 pixel aspect |
| matchfields | true | Apply target-field alignment when fields=true |
| mirror | 0 | Int64 converted by saturation to int32, then required in [0,15] |
| blur | 0 | Int64 converted by saturation to int32, then required nonnegative |
| info | false | Diagnostic property and host text rendering |
| fields | false | Use field aspect; enable matchfields behavior |
| tff | Omitted | Optional boolean field parity override |

No prefix argument is accepted. data's carrier dimensions, sample format, frame rate and pixels are irrelevant; only its frame count and required frame properties are used. No producer identity check is permitted.

## Creation

Validate descriptions and all supplied parameters even when offset=0. clip must have valid positive dimensions for its format. For subpixel=2 every visible plane must have height at least 2; modes 0/1 also support one-row planes. Width one is supported in all modes. RGB, float and unsupported subsampling/depths are creation errors. No Super data, analysis descriptor or block geometry is needed. Compute a=pixaspect/(fields?2:1), cx=fl32(width)/2 and cy=fl32(height)/2, requiring finite positive a. Do not require data[0] properties at creation. info=true additionally requires the [diagnostic renderer](../depan-analyse/kernel-diagnostics.md).

## Evaluation and properties

Apply [temporal selection](kernel-temporal-transform.md). On bypass, the base result is clip[n], including its pixels and all properties. Do not add or overwrite `DepanCompensate_info` on bypass. An enabled info renderer still wraps that result.

On the render branch, obtain clip[s]. Create independent output storage with its full property map, while retaining clip's video description. Convert the accumulated map into [plane maps and coordinates](kernel-sampling-coordinates.md). For GRAY or Y, B=0; for U/V, B=2^(b-1). Render every visible plane with the selected [nearest](kernel-nearest.md), [bilinear](kernel-bilinear.md) or [bicubic](kernel-bicubic.md) operator and plane-specific blur. No existing `_Range` or `_ColorRange` changes those numeric border samples.

Inherited properties come from clip[s], not from data or clip[n]. Do not overwrite any inherited Depan motion keys with the accumulated transform; those keys remain ordinary source-frame properties. If info=true, compute and replace this filter's diagnostic property, then invoke the renderer. If info=false, preserve an inherited same-named diagnostic too.

Motion tuples are consumed in the temporal operator's specified order; later tuples after the first good=false tuple are unused. clip[n] is required on a render branch only if parity is needed and no explicit tff supplies it. clip[s] remains required for every render branch. A missing required dependency is a frame error, not a bypass. All outputs are deterministic for repeated and out-of-order requests.

## Examples

- offset=1,subpixel=0,fields=false, valid dx=2 translation: at n>=1, output (0,0) takes `clip[n-1](2,0)`. The output's inherited frame marker also comes from n-1. At n=0, bypass clip[0].
- For n=2,offset=1 and goodmotion=0, the renderer uses identity on clip[1]. It does not copy clip[2]. With subpixel=1,mirror=0, the rightmost non-bottom column is border-filled as defined by bilinear sampling.
- A 10-bit YUV render with no applicable mirror uses Y=0,U=V=512 for uncovered pixels, independently of range tags. blur=3 becomes chroma blur=1 on 420/422 and remains 3 on 444.
- An 8x8 GRAY clip with offset=0 succeeds without reading malformed data properties. A float clip with offset=0 still fails creation because the plugin's declared render format is unsupported.
- With fields=true,matchfields=false and no tff, `_Field` is not required, but a is still pixaspect/2. With both flags true, top target parity subtracts 0.5 from ty even when goodmotion=0.
