# Flow

Returns a single-reference motion-displaced video. Parameters in interface order:

| Parameter | Default | Domain and behavior |
| --- | --- | --- |
| clip | Required node | Render format, timing, fallback pixels and output properties |
| super | Required node | Reference logical phase samples and applicable field parity |
| vectors | Required node | One public analysis field per output frame |
| time | 100.0 | Finite binary64 percentage in [0,100] |
| fields | False | Integer truth value; correction active only with pel>1 and odd DeltaFrame |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Converted to binary32, finite in [0,100] |
| tff | Omitted | Optional integer truth value; overrides parity when present |
| prefix | MVUtensils | String selecting Super and Analysis properties |

## Creation

Require fixed positive dimensions and a fixed GRAY or YUV420/422/440/444 render format, integer8..16 or float32, subject to host support. clip and Super have the same actual dimensions Wr,Hr, color family, sample type/depth, plane ratios and positive frame count Nv. Result frame count and rate come from clip; auxiliary frame rates need not match. vectors has at least Nv frames, with arbitrary carrier pixels/format.

Use the Phase 1 [metadata-only decoder](../../phase-1/analyse/kernel-vector-validation.md) on vectors[0]. InvalidMetadata is a creation error; MetadataOnly is sufficient. Save effective scalar fields including signed d=DeltaFrame; d=0 is allowed. Require:

- Analysis working W,H, actual Wr,Hr, padding hp,vp and pel p equal Super's logical level-zero description.
- Bx,By belong to [Super's supported block-size table](../../phase-1/super/plugin.md). Public overlap domains hold and block dimensions/overlaps are divisible by each processed render-plane ratio. Actual dimensions are divisible by those ratios.
- Wr<=WB=Nx*(Bx-Ox)+Ox<=W and Hr<=HB=Ny*(By-Oy)+Oy<=H. Analysis precision may differ from render precision. Levels is positive; rendering uses level zero.
- If AnalysisChroma is true, render video is YUV and its ratios match analysis ratios. If false, render chroma still uses render ratios and is still processed.

Use the creation descriptor and exact Phase 1 [scene-threshold equations](../../phase-1/sc-detection/kernel-scene-classification.md). Derive time t and validate [zero-displacement sampling](kernel-flow-sampling.md#admission-and-error-timing). Every processed Super plane must be present and expose its logical phase domains. fields=true with p=1 is accepted and inactive. No Compensate/Degrain whole-vector-domain sampling check applies to Flow.

## Frame evaluation

For output n, require clip[n] and vectors[n]. Decode the complete public field before applying temporal fallback. InvalidMetadata gives unavailable. Valid metadata must match all saved effective scalars except Levels; positive Levels may vary. A mismatch is a frame error even with absent arrays. Matching MetadataOnly gives unavailable. Malformed complete arrays, invalid vector bounds and negative SAD are errors, even when n+d is outside [0,Nv).

For a matching Complete field, available means scene-classifier change=0 and 0<=n+d<Nv. Compute the sum using sufficiently wide checked arithmetic. When unavailable, return clip[n]'s visible pixels and all properties exactly. No reference Super frame or field parity is semantically required by this fallback.

When available, require and validate Super[n+d] against the creation format, logical geometry, phase domains and storage contract. For active field correction, establish parity as [the dense-field kernel](kernel-dense-vector-field.md) specifies. If tff is omitted, also require Super[n] and validate its logical description/storage and `_Field`; when n+d=n the same immutable frame can serve both roles. Supplied tff computes parity from indices and needs no separate Super[n] dependency. No current Super image is otherwise required at evaluation. Always validate required parity at time=0.

Construct plane-specific [dense vectors](kernel-dense-vector-field.md), preflight all actual positions, then [sample the reference](kernel-flow-sampling.md). A dependency failure, missing payload, unsafe view, incorrect domain or failed actual coordinate check is a frame error, never an unavailable-reference fallback. Do not use carrier pixels or private carrier payloads as image sources.

## Output and examples

Output video description equals clip. Render Y for GRAY and all three planes for YUV, regardless of AnalysisChroma. Copy every property from clip[n] with unchanged type/count/value, including any preexisting Analysis, Super, range or scene properties. Add no diagnostic or vector-field properties. Generated writable pixels must not alias inputs. No persistent writable frame-order state is needed.

- Two GRAY8 8x8 frames, matching Super and one-block analysis, p=1,pad=4,d=1,zero vector,SAD=0: if clip[0] is all 5 and Super[1] all 80, output[0] is all 80 at time=100 and time=0. Output[1] falls back to clip[1]. Each output preserves its corresponding clip properties.
- With d=0 and a valid eligible field, nonzero vectors warp Super[n]. It is not necessarily a copy of clip[n].
- With p=2,time=50,d=1,fields=true,tff=true,n even, a constant zero field acquires Y displacement (0,1) before time scaling. Its final vertical offset is floor((128+128)/256)=1 pel unit: half a luma pixel. With fields=false the offset is zero.
- With an otherwise available field and omitted tff, missing required `_Field` is an error. With an unavailable field, the same missing parity is irrelevant and clip[n] is returned.
