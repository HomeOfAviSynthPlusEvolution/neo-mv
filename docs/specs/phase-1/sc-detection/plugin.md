# SCDetection

`SCDetection(clip, vectors, thscd1=400, thscd2=51.0, prefix="MVUtensils") -> clip`

`clip` and `vectors` are required video nodes. `thscd1` is int64 in [0,16320], in 8-bit 8x8 reference-error units. `thscd2` is a floating percentage of blocks, converted to binary32 before the finite [0,100] check. `prefix` is a string used only for Analysis property names.

## Creation and dependencies

Read vectors[0] metadata without loading its arrays and establish the descriptor used by [scene classification](kernel-scene-classification.md): block size Bx,By, grid Nx,Ny, boolean chroma flag, ratios rx,ry and precision b. Missing/unusable creation metadata is a controlled creation error; an ordinary frame-0 metadata-only Analyse output is valid for this step. Reject invalid threshold arguments or unrepresentable derived values.

Frame n requests clip[n] and vectors[n], not clip[n+delta] or any Super. The clip is a carrier: its pixel format, dimensions and precision need not equal the analysis metadata. Preserve any variable format/dimension behavior that the host permits on clip. Each requested vectors[n] must exist; retrieval errors are frame errors.

## Per-frame behavior

Decode vectors[n] with [vector validation](../analyse/kernel-vector-validation.md). InvalidMetadata yields change=1. For readable valid metadata, require the descriptor fields listed above to match their creation values; a change is a frame error even when arrays are unavailable. Other metadata may vary within the validation domain. Matching MetadataOnly yields change=1; malformed complete data yields a frame error. Classify a Complete field using that descriptor. Thresholds may be cached or recomputed with identical results.

Take effective d from vectors[n]'s scalar DeltaFrame decoding, including zero when missing. Start output from clip[n]'s visible pixels and all its properties, then replace exactly one key with a one-element int64 value `change`:

- d>0: `_SceneChangeNext`.
- d<=0: `_SceneChangePrev`.

Do not prepend prefix to these keys. Preserve the other directional key exactly, including its prior type/count; do not create it when absent. Preserve all other clip properties. Do not copy unrelated vectors properties. Output video information, frame count/rate and visible pixels equal clip; no auxiliary images are generated.

No separate check of n+d against the clip length controls classification. A complete external vector field can be classified at a boundary; normal Analyse boundaries usually lack arrays and therefore produce change=1.

## Examples

With clip properties Prev=[7],Next=[9,8], current delta=2 and change=0, output keeps Prev=[7] and replaces Next with [0]. A zero delta instead selects Prev.

For a matching descriptor with N=4, arrays of counts 4 and 3 produce flag 1. Arrays both of count 4 with SAD=-1 fail the frame; they do not produce a successful flag. A missing current AnalysisPel makes metadata invalid and yields flag 1, provided the field was valid enough at creation to establish thresholds. A current valid grid of 3x2 after a creation grid of 2x2 fails the frame instead of classifying six entries against a four-block threshold.

Instances and concurrent requests must not modify shared input property maps or share mutable classification state.
