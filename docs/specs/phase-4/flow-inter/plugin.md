# FlowInter

Produces a video at one fixed relative time between each left frame and its selected right frame. The output description, frame count and frame rate equal clip. Parameters in interface order:

| Parameter | Default | Domain and meaning |
| --- | --- | --- |
| clip | Required node | Render description, property source and fallback pixels |
| super | Required node | Motion-path image samples |
| vectors | Required two-node array | Ordered [bw,fw], positive/negative matching delta |
| time | 50.0 | Convert to binary32; finite in [0,100] |
| ml | 100.0 | Convert to binary32; finite positive occlusion normalization |
| blend | True | Integer truth value; blend original images when the main pair is unavailable |
| thscd1 | 400 | Int64 in [0,16320] |
| thscd2 | 51.0 | Convert to binary32; finite in [0,100] |
| prefix | MVUtensils | String selecting Super and Analysis properties |

At creation apply [pair geometry and scene thresholds](kernel-pair-input.md), validate [occlusion constants](kernel-dense-occlusion.md), and compute

$$t=\operatorname{trunc}(\operatorname{fl32}(\operatorname{fl32}(time\,256)/100)).$$

This binary32 parameter calculation differs from Flow's binary64 time conversion. Frame-zero arrays are not needed for creation. time endpoints, blend=false and a clip shorter than d do not waive creation checks.

## Frame evaluation

For output n in [0,Nv), l=n,r=n+d. Require clip[l] for properties. Apply the pair-selection sequence. FlowInter always enables extra fields; there is no extramask argument. When the main pair is eligible, build [dense motion and occlusion](kernel-dense-occlusion.md), choose basic/extra mode, validate required Super frames and preflight [all actual positions](kernel-bidirectional-sampling.md), then [compose pixels](kernel-pixel-composition.md).

If l/r are out of range or a main field is ineligible, blend=true uses [ordinary blending](kernel-fallback-blend.md) of clip[l] and clip[min(r,Nv-1)] with t. Both are required, even when their indices coincide. blend=false copies clip[l]'s visible samples. No reference Super or extra field is required by fallback. In-range corrupt main data and required extra/image errors remain errors as specified by the pair kernel.

There is no endpoint-copy shortcut at t=0 or t=256: selected vector validation, extra-field selection, position preflight and arithmetic still apply. Whole-frame fallback at an endpoint also follows blend, rather than being unconditionally changed into a copy.

## Properties and examples

Copy all properties from clip[l] with unchanged types, counts and values in every branch. Do not add masks, motion arrays, scene flags or duration changes. Generated pixels cannot alias input sample storage; a logical exact-copy branch may share immutable pixels.

- Matching GRAY8 actual/working 8x8,one 8x8 block,overlap=0,pad=4,p=1,d=1,three frames. At n=0 take complete zero main vectors with SAD=0, matching absent extra arrays, Super[0]=10 and Super[1]=21, time=50. Output is all 15 with clip[0]'s properties, even if clip[0]'s pixels differ. Float render with the same integer-analysis metadata and float Super values gives 15.5.
- At the last frame, blend=true blends the last original frame with itself. Integer values are unchanged; float arithmetic follows the fallback formula. blend=false copies it exactly.
- At n=0,time=0, a malformed in-range main field is an error. Complete main vectors with scene change instead use the configured fallback. This is deliberately different from FlowFPS's endpoint branch.
