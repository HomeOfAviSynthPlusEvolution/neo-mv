# MVU analysis-data contract

Typed output of Analyse and Recalculate, also accepted from external producers. [Validation](kernel-vector-validation.md) defines scalar decoding and array acceptance; [block error](kernel-block-error.md) defines how error values are computed.

## 1. Transport and naming

Analysis data is carried by host frame properties on a video frame. Integer values are signed 64-bit host integers, not byte strings or native C++ structures. Consumers MUST use property values rather than reinterpret a host integer-array buffer as a serialized struct.

Each property name is the exact concatenation of the user-selected `prefix` and the suffix below. The compatibility default is `MVUtensils`; no separator is inserted. For example, the default vector-array key is `MVUtensilsAnalysisVectors`. Consumers and external transformers MUST use the same prefix.

Producer scalar properties contain one integer each. The carrier's visible video format and dimensions do not determine the analysis format or geometry. In particular, an external vector scaler may change analysis geometry while leaving the carrier video unchanged.

## 2. Scalar fields

Let W,H be working image dimensions excluding padding; Wr,Hr the analyzed image dimensions; hp,vp the padding; Bx,By the block size; Ox,Oy the overlap; Nx,Ny the block-grid size; p the subpixel factor; and d the temporal displacement.

| Suffix | Meaning | Supported metadata domain |
| --- | --- | --- |
| `AnalysisWidth`, `AnalysisHeight` | W,H in luma pixels | Positive; W >= Wr and H >= Hr |
| `AnalysisRealWidth`, `AnalysisRealHeight` | Wr,Hr in luma pixels | Positive |
| `AnalysisHPad`, `AnalysisVPad` | hp,vp in luma pixels, on each side | Nonnegative |
| `AnalysisPel` | p; vector units per luma pixel | 1, 2, or 4 |
| `AnalysisLevels` | Number of analysis levels reported by the producer | At least 1; does not specify the number of exported vector arrays |
| `AnalysisChroma` | Whether chroma contributes to analysis | Producers write 0 or 1; consumers interpret nonzero as true |
| `AnalysisXRatioUV`, `AnalysisYRatioUV` | Luma-to-chroma spatial ratios | Each 1 or 2; 1,1 for GRAY |
| `AnalysisBlkSizeX`, `AnalysisBlkSizeY` | Bx,By in luma pixels | Each at least 2; function-specific support is additional |
| `AnalysisOverlapX`, `AnalysisOverlapY` | Ox,Oy in luma pixels | 0 <= Ox <= floor(Bx/2), likewise Y |
| `AnalysisNBlkX`, `AnalysisNBlkY` | Nx,Ny | Positive |
| `AnalysisDeltaFrame` | d; reference index is n+d for a vector frame at n | Any effective signed 32-bit value passes this scalar-domain check, including 0. Analyse and AnalyseMany produce nonzero d; Recalculate preserves its input member's creation-time d, including 0 |
| `AnalysisBitsPerSample` | Sample precision used to generate the error data | 8 through 16 for integer analysis, or 32 for float analysis |

Canonical geometry and temporal scalars in the supported domain are representable as signed 32-bit values. For noncanonical scalar input, apply the effective-value decoding rules in [validation](kernel-vector-validation.md) before validating this table. Derived products, offsets, allocation sizes, and frame indices MUST be checked before use. The array values themselves are not restricted to signed 32-bit range.

Block index i = by*Nx+bx, with bx varying fastest. Its unpadded top-left is (bx*(Bx-Ox), by*(By-Oy)). The grid covers Nx*(Bx-Ox)+Ox by Ny*(By-Oy)+Oy pixels. Membership in this metadata domain alone does not prove that a particular image-producing filter supports the block geometry or has complete visible coverage.

## 3. Vector and error arrays

`AnalysisVectors` and `AnalysisSAD` are host int64 arrays. A complete vector frame has exactly N=Nx*Ny entries in each, in identical row-major block order. Only the finest exported vector grid is present; `AnalysisLevels` does not imply concatenated grids for all pyramid levels.

Each vector element contains two signed 32-bit components in a 64-bit pattern. Define u32(v) = v modulo 2^32, taking the result in [0,2^32). Then

$$P=u32(v_x)+2^{32}u32(v_y).$$

The host int64 value is P if P < 2^63, otherwise P-2^64. On decoding, recover P modulo 2^64, split the low and high 32 bits, and interpret each half as a signed two's-complement integer. This is a numerical bit-field definition, not a CPU byte-order requirement.

The displacement in luma pixels is (vx/p, vy/p), applied to the corresponding location in reference frame n+d. Positive X points right; positive Y points down. Chroma addressing, field shifts, and temporal scaling are defined by the consuming operation; a generic parser MUST NOT silently apply them a second time.

`AnalysisSAD[i]` is a nonnegative int64 raw block-error value. It excludes search penalties. Despite its name, the luma contribution may be SATD when that analysis option is enabled; chroma contributions are SAD. There is no field in this profile identifying that option, so a consumer MUST NOT infer the generating metric from the property name alone.

Integer sample errors use the analysis sample scale, not automatic 8-bit normalization. For float analysis, each plane's nonnegative raw metric e is encoded separately before plane contributions are summed. The scalar reference encoding uses binary32 round-to-nearest-even, denoted fl:

$$
z=fl(e\times65535),\qquad
Q(e)=\begin{cases}
0 & z\le0 \\
4294967295 & z\ge4294967040 \\
\operatorname{trunc}(fl(z+0.5)) & \text{otherwise.}
\end{cases}
$$

The supported float domain requires finite intermediate values. The exported error is the luma contribution plus enabled chroma contributions, accumulated without 32-bit wraparound. Numerical tolerance for independently calculated metrics is separate from exact integer property transport.

## 4. Availability is distinct from zero motion

Analyse with an out-of-sequence reference writes metadata and removes any inherited vector/error arrays under the selected prefix. With an available reference, newly computed arrays replace both keys. Recalculate always generates both arrays after a successful frame computation. These cases do not reinterpret an absent field as measured zero motion.

Consumers MUST distinguish:

- complete, valid arrays, including all-zero motion;
- metadata without usable arrays;
- invalid array data requiring a controlled error.

With otherwise readable metadata, missing arrays or counts other than N mean no usable vector field. Do not invent missing entries. SCDetection reports a scene flag; Recalculate supplies zero motion as the input to its remapping and fresh error measurement. Complete arrays can still exceed scene thresholds.

## 5. Valid displacement bounds

For each block, let x0=hp+bx*(Bx-Ox), y0=vp+by*(By-Oy). The compatible per-block bounds are

$$-p x_0\le v_x<p(W+2hp-x_0-B_x),$$
$$-p y_0\le v_y<p(H+2vp-y_0-B_y).$$

Upper bounds are exclusive. Negative SAD and out-of-range vectors are errors, not ordinary unavailable motion. These checks do not authorize arbitrary later warps or malformed geometries; each consumer must also satisfy its own sampling-domain contract.

## 6. Mutation and interpretation

A third party may change geometry, vectors, or errors if it preserves their consistency and the consumer's support domain. Neo-mv MUST NOT require that a valid vector field was produced by its own Analyse implementation. Do not use private Super payloads inherited on a vector carrier to override its public Analysis properties.

Reading vectors and recomputing vector errors have different requirements. A motion-only consumer must interpret error values according to `AnalysisBitsPerSample`, even if its render video has a different supported depth. Recalculate requires its input analysis precision to equal its Super precision. Other geometry, plane, and format checks are stated by each consuming plugin.

## 7. Hand-checkable examples

- At p=2, vector (-3,2) means (-1.5,+1) luma pixels. Its pattern is `0x00000002FFFFFFFD`, transported as int64 12884901885.
- Vector (3,-2) has pattern `0xFFFFFFFE00000003`, transported as int64 -8589934589. The negative property value is valid packed data.
- Nx=7 and Ny=3 require 21 entries in both arrays, even when `AnalysisLevels` is greater than 1.
- With valid metadata and bounds that include zero for every block, 21 zero vectors with 21 zero errors represent valid zero motion. Absent arrays are not equivalent to it.
- For W=32, hp=4, Bx=8, p=2 and bx=0, the X range is [-8,56). A value of 55 passes this bound; 56 does not.
- For float-plane metric values, Q(0)=0, Q(0.5)=32768, and Q(1)=65535. Two enabled planes each with raw error 0.5 contribute 65536 after separate encoding; encoding their combined raw error 1 would instead give 65535 and is not the specified per-plane procedure.
