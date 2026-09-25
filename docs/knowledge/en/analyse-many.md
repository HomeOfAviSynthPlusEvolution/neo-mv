# AnalyseMany: independent analyses at multiple distances

## 1. What the function computes

`AnalyseMany` runs [Analyse](analyse.md) independently for several positive and negative temporal offsets and returns an array of video nodes. It neither combines distances into one frame nor introduces another motion estimation algorithm.

## 2. Objects and notation

`R=radius` is the number of distance groups; `D=delta` is the positive distance step. Here delta must be positive, unlike Analyse's signed nonzero offset. Output indices start at zero; each pair contains the positive offset first, then the negative one.

## 3. Overall calculation

Validate R, D and the result length, construct member arguments from nearest to farthest, independently create every analysis node, then return the complete `2R` nodes.

## 4. Step-by-step calculation

For `r=1…R`:

$$O_{2(r-1)}=Analyse(super,delta=rD,\theta),$$
$$O_{2(r-1)+1}=Analyse(super,delta=-rD,\theta).$$

Here θ includes all other arguments, preserving whether each was omitted. Omitted `tff` still means reading the actual frame's `_Field`, not `tff=false`. Omitted `pzero` still derives its default from effective `pnew` under Analyse's rules.

Each member retains the full source frame count and rate. Its frame n uses `super[n]` as the carrier and `n±rD` as the reference. Estimation, visible pixels, and output properties all follow Analyse. Members may share immutable Super data, but not mutable analysis results.

`metric` is passed unchanged to every member: `"sad"` by default, or `"satd"` / `"dct"` under Analyse's restrictions. Each member uses the same coefficient quantization and error rules; errors are not combined across members.

## 5. A complete numerical example

`radius=3,delta=2` produces offsets `[2,-2,4,-4,6,-6]`. In a 20-frame sequence, frame 8 refers to `[10,6,12,4,14,2]`, all valid.

At frame 0, positive members still analyze frames 2, 4, 6. Negative references are out of range, so those members retain scalar analysis metadata but remove vector/error arrays. The result still contains six 20-frame nodes; unavailable members are neither removed nor reordered.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `radius` | 1; positive after int32 saturation | Creates `2R` members; precedes `prefix` in the interface |
| `delta` | 1; positive after int32 saturation | Distance increment |
| Other parameters | Same as [Analyse](analyse.md) | Passed unchanged, including omission state |

## 7. Boundaries, missing data, and errors

Every `rD` must fit a signed 32-bit offset, and `2R` must be representable by the host and result array. A member creation failure fails the whole call and identifies the zero-based failing member; it does not return a shorter success array. Frame errors belong to the requested member/frame.

Negative delta is invalid, not a way to reverse the array. Each member independently decides reference availability.

## 8. Precision and determinism

Members have no numerical feedback between them. Concurrency and request order do not change member results. Integer products and array lengths are checked before allocation, not allowed to overflow into offsets.

[Back to the English index](README.md)
