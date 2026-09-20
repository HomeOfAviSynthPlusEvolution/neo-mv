# Compensate

Returns one video node under `clip`. Parameters are in interface order:

| Parameter | Default | Domain and behavior |
| --- | --- | --- |
| clip | Required node | Visible video description, fallback pixels and output properties |
| super | Required node | Current and reference logical image samples |
| vectors | Required node | One typed analysis field per output frame |
| thsad | 10000 | Nonnegative int64; block error threshold in reference 8-bit 8x8 units; scaled result must fit int64 |
| fields | False | Integer truth value; true requires pel>1 |
| time | 100.0 | Finite binary64 in [0,100]; quantized to t in [0,256] |
| thscd1 | 400 | Int64 in [0,16320]; scene bad-block threshold |
| thscd2 | 51.0 | Converted to binary32; finite percentage in [0,100] |
| tff | Omitted | Optional integer truth value; omission and explicit false have different field semantics |
| prefix | MVUtensils | String selecting Super and Analysis property names |

Missing optional arguments use these defaults. Reject malformed types. thsad is not saturated to int32. The [common numerical and memory contract](../README.md#common-contracts) applies.

## Creation

Establish [reference and render geometry](kernel-reference-availability.md), save the vector member's d, derive scene and block thresholds, and perform the [sampling admission checks](kernel-block-sampling.md#whole-domain-admission) for every block and render plane: the time-scaled reference map over the entire public vector domain at f=0, and the current-block map for all allowed f. Negative thsad, invalid time, fields=true with pel=1, unrepresentable arithmetic, inadequate coverage or a failed creation sampling check is a controlled creation error. A valid zero d is allowed. Actual nonzero field shifts on reference vectors are checked during frame evaluation.

The frame-zero field may be MetadataOnly. Its missing arrays do not bypass creation geometry checks. Neither the vector carrier's visible pixels nor an inherited private Super payload on that carrier supplies render samples.

## Frame dependencies and evaluation

For output n require clip[n], Super[n] and vectors[n]. Validate the current Super and decode the field using the reference-availability kernel. When available=false, output clip[n]'s visible pixels and all its properties exactly; do not read reference Super frames or require field-parity metadata for this fallback.

When available=true, require Super[n+d], validate its logical geometry/storage, establish applicable field parity, and validate every supplied vector's full reference footprint with the actual f before generating any blocks. A failed footprint is a frame error even if that block would select current Super by SAD. Then generate blocks with the compensation kernel and compose the visible output. Repeated references to the same node/frame may share an immutable frame. No result depends on evaluation of a previous output frame.

If a required dependency fails, report a frame error. Scene change and missing arrays select the documented fallback; corrupt complete arrays, changed valid metadata, invalid current Super or unsafe storage do not become a successful fallback.

## Output

Dimensions, sample format, frame count and frame rate equal clip. For GRAY render Y; for YUV render all three planes. Successful compensation uses current/reference Super samples, even if clip[n] has different pixel values. All output properties are copied from clip[n], with their existing types and element counts. Do not add, remove, relabel or recompute Analysis, Super, scene-change or diagnostic properties.

The whole-frame unavailable-reference fallback is distinct from the per-block s>=T selection of current Super samples. Writable generated frames cannot alias input sample storage. Allocation failure yields an error rather than a partial frame.

## End-to-end examples

- Two GRAY8 8x8 frames, matching one-block Super geometry, p=1, pad=4, current Super all 10, reference Super all 80, clip[n] all 5. A complete zero vector with SAD=99, thsad=100, thscd1=400 gives all 80; changing SAD to 100 gives all 10. A missing vector array gives all 5. Each output keeps exactly clip[n]'s properties.
- On a two-frame clip with d=1, output n=1 falls back to clip[1]. With d=0 and a valid eligible field, both sample sources refer to Super[n]; nonzero vectors still move within that image.
- YUV420 16x16, block 8x8, overlap 0, pad=3,p=2,time=100,fields=false fails creation because some public vectors map outside chroma phases. With time=0, the zero-displacement map passes that geometry check; valid-field and scene requirements remain in force.
