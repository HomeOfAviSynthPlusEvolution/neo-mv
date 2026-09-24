# Super

Builds image pyramids, extended borders, and subpixel sampling data for motion analysis and compensation. Returns a clip carrying this auxiliary data while preserving the visible image.

## Calling the function

Use `core.neo_mv.Super` in VapourSynth and `neo_mv_Super` in AviSynth. Both interfaces use the same parameter order:

```text
Super(clip, blksize, overlap [, pad, onelevel, sharp, rfilter, pel, pelclip, prefix])
```

The brackets above denote optional arguments, not a literal script array. The first three arguments are required; pass optional arguments by name. The return type is a VapourSynth `VideoNode` or an AviSynth `clip`.

## Parameters

| Parameter | Type | Default | Values and effect |
| --- | --- | --- | --- |
| `clip` | Clip | Required | Fixed dimensions and format, with at least one frame. Accepts planar GRAY/YUV with 8–16-bit integer or 32-bit float samples. Each YUV chroma subsampling ratio must be 1 or 2. |
| `blksize` | Integer axis array | Required | `[block width, block height]` in luma pixels. Determines working-image block coverage and pyramid level count. Supported pairs are listed below. |
| `overlap` | Integer axis array | Required | `[horizontal overlap, vertical overlap]` in luma pixels. Each value must be between zero and half the corresponding block dimension, inclusive, and chroma aligned. |
| `pad` | Integer axis array | `[16, 16]` | `[horizontal padding, vertical padding]` in luma pixels. Both values must be positive. The horizontal value applies to each of the left and right borders; the vertical value applies to each of the top and bottom borders. |
| `onelevel` | Boolean | `false` | Generate only level 0 when true. Otherwise calculate the level count from image dimensions, blocks, and padding. |
| `sharp` | Integer | `2` | Built-in subpixel interpolation: `0` uses two-sample averaging, `1` a four-tap filter, and `2` a six-tap filter. Only 0–2 are accepted. This does not sharpen the visible output. |
| `rfilter` | Integer | `1` | Pyramid reduction: `0` uses a 2×2 average, `1` a separable four-tap filter, and `2` a separable six-tap filter. Only 0–2 are accepted. |
| `pel` | Integer | `2` | Subpixel precision: `1` for integer pixels, `2` for half pixels, and `4` for quarter pixels. Only 1, 2, and 4 are accepted. |
| `pelclip` | Clip | Omitted | An externally enlarged image supplying fractional sampling phases instead of built-in interpolation. Format, size, and frame-count requirements are listed below. |
| `prefix` | String | `"MVUtensils"` | Prefix for output property names. Subsequent functions reading this Super must use the same prefix. An empty string is allowed; NUL characters are not. |

Write Boolean values as `True`/`False` in VapourSynth scripts and `true`/`false` in AviSynth scripts.

### Axis arrays and block sizes

`blksize`, `overlap`, and `pad` follow these rules:

| Input | Meaning |
| --- | --- |
| `8` or `[8]` | Use 8 for both axes. |
| `[16, 8]` | Use 16 horizontally and 8 vertically. |
| `[]` | Use the fallback: `[8, 8]` for `blksize`, `[0, 0]` for `overlap`, and `[16, 16]` for `pad`. |
| More than two elements | Error. |

**An empty array is different from an omitted argument.** You must still supply `blksize` and `overlap`, even though explicit empty arrays have fallback values.

Supported block sizes are `4×4`, `8×4`, `8×8`, `12×12`, `16×2`, `16×8`, `16×16`, `24×24`, `32×16`, `32×32`, `48×48`, `64×32`, `64×64`, `128×64`, and `128×128`. Width and height are not interchangeable.

The input width and height must be at least the corresponding block dimensions. YUV image dimensions, block dimensions, and overlaps must be divisible by the chroma subsampling ratio on each axis. For example, YUV420 requires even block dimensions and overlaps on both axes. Image dimensions do not have to be multiples of the block dimensions.

### External subpixel images

When `pelclip` is supplied and `pel>1`:

- Its color format, sample type, and bit depth must match `clip`.
- Its width and height must each be `pel` times the original, with the same number of frames.
- Output frame n uses frame n from both inputs. The caller is responsible for temporal correspondence; no frame-rate conversion takes place.
- Integer-position samples and coarse pyramid levels still come from `clip`. Only fractional phases come from `pelclip`. `sharp` does not generate those external phases, but must still be in 0–2.

With `pel=1`, a supplied `pelclip` must still have supported fixed dimensions, a nonzero frame count, and the same format. Its dimensions need not satisfy the enlargement relation and its frame count need not equal the original. Its frames are not requested.

## Return value and frame properties

Returns one clip with the same width, height, format, frame count, and frame rate as `clip`, preserving its visible pixels. AviSynth also forwards audio and parity from `clip`.

Auxiliary images are carried by frame properties. Prepend `prefix` to every name below; for example, the default full name of `SuperPel` is `MVUtensilsSuperPel`.

| Property suffix | Contents |
| --- | --- |
| `SuperWidth`, `SuperHeight` | Level-0 working dimensions, excluding surrounding padding; these may exceed the visible dimensions. |
| `SuperRealWidth`, `SuperRealHeight` | Original visible dimensions. |
| `SuperHPad`, `SuperVPad` | Luma padding. |
| `SuperPel`, `SuperLevels` | Subpixel precision and pyramid level count. |
| `SuperChroma`, `SuperXRatioUV`, `SuperYRatioUV` | Whether chroma is present, and its subsampling ratios. |
| `SuperBitsPerSample` | Sample bit depth. |
| `SuperBlkSizeX`, `SuperBlkSizeY`, `SuperOverlapX`, `SuperOverlapY` | Block dimensions and overlaps. |
| `NeoMVSuperDescriptorV1`, `NeoMVSuperPlanesV1` | Description data and auxiliary frame references consumed by subsequent functions. Do not manually modify or remove them. |

Existing Super data with the same prefix is replaced; other frame properties are preserved. Removing auxiliary properties later can leave a clip that displays correctly but is no longer a valid Super input.

## Minimal examples

Replace the plugin path with your local path. These examples use a blank source and can run without an additional video source plugin.

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, format=vs.YUV420P8)
super_clip = core.neo_mv.Super(clip, blksize=[8, 8], overlap=[4, 4])
super_clip.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, pixel_type="YV12")
super_clip = neo_mv_Super(clip, blksize=[8, 8], overlap=[4, 4])
return super_clip
```

Both examples use the defaults `pad=[16,16]`, `pel=2`, `sharp=2`, and `rfilter=1`. The output remains the original blank image, with auxiliary data attached to its frames. Pass `super_clip` to functions such as `Analyse`.

## Restrictions and common errors

| Situation | Result or remedy |
| --- | --- |
| Omitted `blksize` or `overlap` | Required-argument error. Supply a value or an explicit empty array. |
| RGB, an alpha format, or unsupported chroma subsampling | Format error. Convert to supported planar GRAY/YUV first. |
| Unsupported block pair, an image smaller than its block, overlap exceeding half a block, or zero padding | Super geometry error. |
| Chroma axes not aligned | Geometry alignment error, such as odd overlaps with YUV420. |
| A padded plane too small for built-in interpolation | `sharp=0/1/2` requires each padded plane's width and height to be at least 2/4/6, respectively. |
| Mismatched `pelclip` format, or mismatched size/frame count when `pel>1` | Creation fails. Prepare the input according to the external subpixel image requirements. |
| Unrepresentable dimensions, insufficient readable samples for reduction, or insufficient memory | Creation or frame retrieval fails. Positive padding alone does not make every extreme geometry valid. |

## Computation

See [How Super computes its output](../../knowledge/en/super.md) for working dimensions, level counts, filter coefficients, border handling, and rounding.

[API index](README.md)
