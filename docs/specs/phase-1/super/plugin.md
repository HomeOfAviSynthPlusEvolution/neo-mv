# Super

## Interface

`Super(clip, blksize, overlap, pad=[16,16], onelevel=False, sharp=2, rfilter=1, pel=2, pelclip=omitted, prefix="MVUtensils") -> clip`

Arguments appear in registration order. `clip` and `pelclip` are video nodes; `prefix` is a string. `blksize`, `overlap` and `pad` are integer arrays with int32 elements. Other controls are int32 values with boolean semantics where indicated.

| Parameter | Domain | Effect |
| --- | --- | --- |
| clip | Required fixed-size, fixed-format GRAY/YUV, integer 8-16 or float32 | Source of visible pixels and integer pyramid samples |
| blksize | Required; one or two elements; allowed pairs below | Determines working coverage and pyramid depth, not a resampling kernel size |
| overlap | Required; each 0 through floor(block/2), aligned to chroma ratios | Changes grid steps and working coverage |
| pad | Each strictly positive; default 16,16 | Border size in luma pixels; chroma padding uses floor division |
| onelevel | Boolean, default false | Forces exactly one level |
| sharp | 0,1,2; default 2 | Selects built-in half-sample interpolation |
| rfilter | 0,1,2; default 1 | Selects reduction between levels |
| pel | 1,2,4; default 2 | Number of phase steps per pixel along each axis at level zero |
| pelclip | Optional; see below | Replaces generated non-integer level-zero phases |
| prefix | Default MVUtensils | Names the Super data on output frames |

Allowed block pairs: 4x4, 8x4, 8x8, 16x2, 16x8, 16x16, 32x16, 32x32, 64x32, 64x64, 128x64, 128x128. One-element axis arrays replicate their value. If the host accepts an explicit empty axis array, use the pair fallback: blksize 8,8; overlap 0,0; pad 16,16. Omission of required arguments is still an error. More than two elements is an error.

## Frame computation

1. Derive all dimensions and levels using [geometry](kernel-geometry.md).
2. Extend each original plane to the level-zero working/padded extent using [border extension](kernel-border-extension.md).
3. Produce smaller integer levels using [pyramid reduction](kernel-pyramid-reduction.md), extending every resulting level.
4. Produce level-zero fractional phases using [subpixel interpolation](kernel-subpixel-interpolation.md), or the specified external samples.

Output frame n has exactly the source's visible pixels, dimensions, format, frame count and frame rate. Request clip[n] and, only when pelclip is used, pelclip[n]. Auxiliary generation never replaces visible pixels. Frame requests may occur concurrently or in any order; they must yield the same logical samples.

## Output data

Copy source properties, then replace the following scalar properties with one int64 value each. Names are `prefix` followed directly by the suffix. W0,H0,L and per-plane geometry are defined by the geometry kernel.

| Suffix | Value |
| --- | --- |
| SuperWidth, SuperHeight | W0,H0 |
| SuperRealWidth, SuperRealHeight | Actual source W,H |
| SuperHPad, SuperVPad | Luma padding px,py |
| SuperPel, SuperLevels | pel,L |
| SuperChroma | 0 for GRAY, 1 for YUV |
| SuperXRatioUV, SuperYRatioUV | Global rx,ry, or 1,1 for GRAY |
| SuperBitsPerSample | Source bit depth, including 32 for float |
| SuperBlkSizeX, SuperBlkSizeY | Effective Bx,By |
| SuperOverlapX, SuperOverlapY | Effective Ox,Oy |

Attach the corresponding logical plane/level/phase data for use by this library's consumers. The payload's physical arrangement is private. The payload must remain alive and readable after supported host frame copies and concurrent reads. If input already contains a Super payload under this prefix, replace the selected payload with the newly generated one; do not let stale inherited samples override new metadata. Preserve unrelated properties and other prefixes. Unused storage has no numerical output value.

Consumers must obtain every defined auxiliary sample and its geometry through the library's Super interface, without relying on a process-global producer pointer. The logical data, including interpolation and padding values, is part of the output even though its storage is private.

## External pelclip

The supplied node must have fixed dimensions and the same sample format as clip, even for pel=1. For pel=1 it is otherwise unused: do not request its frames or require its dimensions/frame count to match a scaled clip.

For pel=2/4, require equal frame count and actual dimensions pel*W by pel*H. A mismatch in either dimension or frame count is a creation error. Integer phase samples always come from clip. All nonzero phases come from pelclip; `sharp` does not change them, while `rfilter` still controls smaller levels derived from clip.

## Errors and example

Reject unsupported formats, invalid enums or block/overlap/padding, dimensions smaller than a block, nonpositive derived dimensions, unrepresentable sizes, or an interpolation/reduction request outside a kernel's declared read domain. Validate known geometry at creation. Invalid requested-frame data and non-finite computations fail during frame evaluation. Do not read undefined quarter-phase edges to manufacture a successful result.

GRAY8 input 18x10 with block 8x8, overlap 4x4, pad 2, onelevel=true and pel=1 yields visible 18x10 output, working size 20x12 and one 24x16 padded integer phase. With source corners 7 and 99, auxiliary positions (0,0) and (2,2) are 7, and (23,15) is 99.
