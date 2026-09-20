# Analysis data decoding and validation

Inputs: a typed property map, prefix, and a flag selecting metadata-only or vector-data reading. Output: decoded metadata plus one of InvalidMetadata, MetadataOnly, or Complete; Complete also contains row-major triples (vx,vy,SAD). Malformed complete data produces an error. The [data format](data-format.md) defines names, types, units and scalar domains.

## Scalar decoding

Read element zero of each scalar key. A missing, empty or wrongly typed scalar yields zero; ignore later elements. Except AnalysisChroma, clamp the read int64 to signed int32. AnalysisChroma uses full-int64 zero/nonzero truth semantics. Validate the resulting effective values against the data-format table. A failed domain check yields InvalidMetadata before reading arrays. Delta zero itself does not invalidate metadata.

Check all derived geometry, grid products and bounds before allocation or sampling. Required unrepresentable arithmetic is a controlled error. A metadata-only read stops after scalar validation and returns MetadataOnly for valid metadata; it does not inspect or certify arrays.

## Array decoding

For a vector-data read of valid metadata, let N=Nx*Ny. First compare both array element counts with N. A missing array or either count mismatch returns MetadataOnly before element-type or value checks. If counts match but either type is not integer, fail with a controlled malformed-array error.

Decode each packed vector as specified in the data format; SAD is its corresponding int64 value. For i=by*Nx+bx, set x0=hp+bx(Bx-Ox), y0=vp+by(By-Oy). Require

$$SAD_i\ge0,$$
$$-p x_0\le v_x<p(W+2hp-x_0-B_x),$$
$$-p y_0\le v_y<p(H+2vp-y_0-B_y).$$

Both upper bounds are exclusive. An empty displacement interval admits no Complete vector at that block. A negative SAD or out-of-range vector is a controlled data error; no diagnostic precedence is required when several violations coexist. Successful completion returns Complete even if every vector and error is zero.

This validates the public field, not every image consumer's supported block shape or sample region. A consumer still checks its own dimensions, format and logical sample access.

## Ownership and examples

The property map is read-only. Decoded output must remain valid for its declared lifetime; no borrowed property pointer may survive a mutation that invalidates it. A frame copy or reconstruction from equivalent typed values has identical meaning. Do not require private producer identity, a matching Super payload on the carrier, or an extra mandatory marker.

- With N=4, vector count 4 and SAD count 3 gives MetadataOnly, even if the short array has another type.
- Counts 4 and 4 with a float SAD array give a malformed-array error.
- W=H=16, hp=vp=4, block 8x8, overlap 0, p=2: the first block's X range is [-8,24). vx=23 passes; vx=24 fails.
- A missing AnalysisChroma becomes false and can leave metadata valid. A missing AnalysisPel becomes zero and makes metadata invalid.

Consumers: SCDetection and Recalculate; the field encoder must also emit data accepted by this validator. No SIMD implementation or host-specific parser is prescribed.
