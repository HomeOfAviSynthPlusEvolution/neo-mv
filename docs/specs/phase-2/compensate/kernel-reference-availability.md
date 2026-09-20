# Reference availability and render geometry

Inputs: clip and Super video descriptions, logical level-zero Super geometry, one or more analysis metadata descriptors, current frame index n, saved per-member displacement d, thscd1 and thscd2, and the current typed vector fields. Outputs: validated render geometry, scaled scene thresholds and a per-reference available flag. This operator does not produce pixels or modify properties.

## Creation geometry

The render clip is fixed-size, fixed-format GRAY or YUV420/422/440/444, with integer 8-16-bit or float32 samples, subject to host support. Its actual dimensions Wr,Hr are positive. Clip and Super have the same actual dimensions, sample type, bit depth, color family, subsampling and positive frame count Nv. Frame rate of the result comes from clip; no equal-frame-rate condition is imposed on auxiliary nodes. Every vectors node must provide at least Nv frames; its carrier's dimensions and pixel format are irrelevant.

Read each vector member's frame-zero metadata using Phase 1's [metadata-only validation](../../phase-1/analyse/kernel-vector-validation.md). Invalid creation metadata is an error. MetadataOnly is sufficient; frame-zero arrays are not required to exist. Save each member's effective d and the effective scalar fields described below.

For each analysis descriptor require:

- Working W,H, actual Wr,Hr, padding hp,vp and pel p equal the corresponding level-zero Super values.
- The luma block pair Bx,By is supported by [Super's block-size table](../../phase-1/super/plugin.md). Ox,Oy satisfy the public domain. Block sizes and overlaps are divisible by the render clip's chroma ratios for every processed plane.
- Nx,Ny are positive. Set sx=Bx-Ox, sy=By-Oy and WB=Nx*sx+Ox, HB=Ny*sy+Oy. Require Wr<=WB<=W and Hr<=HB<=H. Every visible output sample is covered; block footprints stay within the working image before displacement. W and H need not equal WB and HB.
- If AnalysisChroma is true, the render clip is YUV and the analysis ratios equal its ratios. If false, analysis ratios do not select render-plane sampling ratios. Sampling always uses the render clip's actual plane ratios.
- AnalysisBitsPerSample need not equal the render depth. AnalysisLevels is positive but does not specify which image level to render: these plugins always use Super level zero.

For Degrain, all vector members agree on all effective analysis scalars except DeltaFrame and Levels. Each member has its own saved d. Compensate accepts any effective signed-32 d, including zero. Degrain's pair constraints are defined by its plugin.

Super must expose its logical phase domains and all processed planes. The [sampling kernel](kernel-block-sampling.md) defines the additional whole-domain geometry check. Apply it at creation even if a scene threshold, a boundary or a zero user weight would prevent a particular reference from contributing. No producer identity or private payload on a vector carrier may override these public analysis values.

## Thresholds

Use the [scene-classification descriptor and equations](../../phase-1/sc-detection/kernel-scene-classification.md) without changing their rounding or strict comparisons. thscd1 is int64 in [0,16320]; thscd2 is converted to binary32 and must be finite in [0,100]. For Compensate use its sole member's creation descriptor; for Degrain use the common descriptor shared by all members.

For reuse in block thresholds define the binary64 factor

$$A=\operatorname{fl64}(B_xB_y/64),\quad C_f=\begin{cases}\operatorname{fl64}(1+\operatorname{fl64}(2/(r_xr_y)))&C\ne0\\1&C=0,\end{cases}$$
$$D=\operatorname{fl64}((2^{\min(16,b_a)}-1)/255),\qquad F=\operatorname{fl64}(\operatorname{fl64}(A C_f)D).$$

Here ba is analysis precision, C is the analysis chroma flag, and rx,ry are its ratios. Products of integer geometry values are exact before conversion to binary64. The divisions are floating divisions. For a nonnegative int64 argument z define

$$S(z)=\operatorname{trunc}(\operatorname{fl64}(\operatorname{fl64}(\operatorname{fl64}(z)F)+0.5)).$$

Reject an unrepresentable result before conversion. Each consuming plugin states its additional range restriction.

## Current field and temporal availability

Decode the current field using Phase 1's complete vector-data read. A malformed complete field or unrepresentable required arithmetic is an error, including when n+d is outside the clip. InvalidMetadata gives available=false. For valid metadata, all effective scalars except Levels must equal that member's saved values, including d; changes are frame errors even when arrays are unavailable. Levels may vary within its positive domain.

A matching MetadataOnly field gives available=false. For a Complete field apply scene classification with the saved descriptor; change=1 gives available=false. Otherwise

$$available=\mathbf1_{0\le n+d<N_v}.$$

This sequence distinguishes an ordinary absent reference from malformed data. n+d uses sufficiently wide checked arithmetic. Do not clamp the reference frame index, reverse the sign of d or retime the field. A zero d in Compensate addresses the current frame.

Only an available reference requires its Super[n+d] image. Every output evaluation still requires a valid clip[n] and current Super[n]. Used Super frames must match the creation geometry, logical phase domains and render sample format. A failed dependency retrieval or invalid memory view is a frame error; it is not silently converted into available=false. Input metadata/property maps and shared Super payloads remain immutable.

## Examples

- On a five-frame clip, saved d=-1 at n=0 gives available=false for an otherwise valid complete field. SAD=-1 in a complete array is still an error. At n=1 the same valid member refers to Super[0].
- With a matching 2x2 grid, T1=400,T2=2, SAD [400,401,900,0] has two bad blocks and remains scene-eligible; [401,401,900,0] does not. A missing array is unavailable, not four zero vectors.
- 10-bit analysis of a luma-only 8x8 block has F=1023/255. For z=400, S(z)=1605 even if the render video is 8-bit. With ba=32, F=257 and S(400)=102800.
- Wr=18, Bx=8, Ox=4, Nx=4 gives WB=20. W=24 is allowed if H and all other geometry conditions pass; W=19 or Nx=3 is not. The output still has 18 visible luma columns.
