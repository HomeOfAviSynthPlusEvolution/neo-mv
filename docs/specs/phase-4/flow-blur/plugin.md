# FlowBlur

Produces motion blur by averaging samples along two motion trajectories in the current Super image. Output video description, frame count and frame rate equal clip. Parameters in interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| clip | Required node | Render description, output properties and unavailable-pair fallback |
| super | Required node | Current logical image samples |
| vectors | Required two-node array | Ordered [bw,fw], positive/negative paired distance |
| blur | 50.0 | Convert to binary32; finite in [0,200] |
| prec | 1 | Int64 converted with sat32; effective value must be at least1 |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Convert to binary32; finite in [0,100] |
| prefix | MVUtensils | String selecting public properties |

At creation apply [pair geometry, direction and scene thresholds](../flow-inter/kernel-pair-input.md#creation), and compute the [blur coefficient](kernel-trajectory.md). sat32(x)=min(2147483647,max(-2147483648,x)); apply it before checking prec. There is no ml, blend, extramask, fields or tff argument. Frame-zero arrays are not required for creation.

## Frame evaluation

For output n require clip[n]. The selected fields are B=bw[n-d] and F=fw[n+d]. First test both indices against [0,Nv). If either is outside, copy clip[n]'s visible pixels and properties without requiring any vector field or Super frame. This boundary selection is specific to FlowBlur; do not substitute bw[n],fw[n].

When both indices are in range, read and validate both fields using [selected-field eligibility](../flow-inter/kernel-pair-input.md#reading-a-selected-field). Validate the second field even if the first is ordinarily ineligible. A malformed complete field or changed valid metadata is an error. If either field is ineligible, copy clip[n]; Super is not required by this fallback.

When both fields are eligible, require Super[n] with its creation format/geometry, logical domains and valid views. There is no image dependency on Super[n-d] or Super[n+d]. Generate both plane-specific dense fields, preflight every actual trajectory point, read samples and apply [ordered averaging](kernel-sample-average.md). Any dependency, storage, domain or numeric failure in this branch is a frame error, not a copy fallback. blur=0 does not skip eligibility or required Super validation: a usable pair then copies integer-phase Super[n] pixels.

## Output and examples

Process every actual render plane, regardless of AnalysisChroma. All properties come from clip[n] with unchanged type/count/value in every branch. Add no motion, mask, duration or scene properties. Inputs remain immutable and generated writable pixels cannot alias them. Repeated frame requests have no mutable history requirement.

- Three GRAY8 frames with matching 8x8 one-block analysis and Super,pad=4,p=1,d=1. Output n=1 uses bw[0],fw[2]. With valid zero vectors,SAD=0,clip[1]=5,Super[1]=80, output is80 for blur=0 and blur=200. At n=0 or2 the output is the original clip frame, without a Super dependency.
- For interior n=1 and the same geometry, use constant F=(2,0),B=(-2,0),blur=200,prec=1. At a position whose ordered Super samples are [10,20,30,0,1], output is12. Properties still come from clip[1].
- blur=0 with one matching absent array and another valid field copies clip[n]. Replacing the second with complete negative SAD produces an error. prec=0 is a creation error; prec=9223372036854775807 converts to2147483647 and is accepted, giving no extra taps for any supported dense vector.
