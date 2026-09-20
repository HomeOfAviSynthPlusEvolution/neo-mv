# VectorLengthMask

Returns a grayscale vector-magnitude visualization. Parameters in interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| vectors | Required node | Public analysis fields; carrier pixels are ignored |
| ml | 100.0 | Converted to binary32, finite and positive; magnitude normalization |
| gamma | 1.0 | Converted to binary32, finite and nonnegative |
| time | 100.0 | Finite binary64 percentage in [0,100]; validated but unused by this magnitude operator |
| scval | 0.0 | Converted to binary32; unavailable/scene-change fill sample |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Converted to binary32, finite in [0,100] |
| prefix | MVUtensils | String selecting public analysis property names |

## Creation and evaluation

Apply the [shared mask contract](kernel-mask-input.md), validate geometry, establish the output format, scene thresholds and fallback value, and validate the magnitude operator's constants. Frame-zero arrays are not required for creation. Output n requires only vectors[n]. Decode it before choosing an eligible path or fallback. Missing arrays, invalid current metadata and scene change select scval; malformed complete data, changed valid metadata and failed dependency retrieval are frame errors.

For eligible input, run the [magnitude operator](kernel-vector-length.md) and then [grid resampling](kernel-grid-resampling.md). For ineligible input, fill the visible plane with scval's converted sample. Both paths use the creation output description. Unsupported host output precision is a creation error, not an automatic conversion.

## Output properties

Construct the result's property map with exactly one public entry: `_Range`, integer, one element with value 1, meaning full sample range in this interface. No input frame properties, Analysis arrays, Super data, color matrix tags or scene flags are copied. The key and value are fixed; do not substitute `_ColorRange` or derive this value from a host enumeration. A host may manage its own non-property frame bookkeeping.

This output/property rule and the shared mask contract also apply to SADMask and OcclusionMask. Writable output storage must be independent of all inputs. Frame order and simultaneous filter instances must not affect pixels or properties.

## End-to-end examples

- A two-frame GRAY8 carrier may contain valid 10-bit analysis metadata for Wr=Hr=8, Bx=By=8, overlap zero, Nx=Ny=1, p=1, W=H=8 and pad=8. With vectors (4,0), SAD=0, ml=8,gamma=2, the output is two GRAY10 8x8 frames of 255, because Q(1023/4)=255. Carrier format and pixel values are irrelevant. Each result has only `_Range=[1]` in its property map.
- For the same metadata with a missing array and scval=20.5, every output sample is 21. Retaining a carrier `_SceneChangePrev` property would violate the output contract.
