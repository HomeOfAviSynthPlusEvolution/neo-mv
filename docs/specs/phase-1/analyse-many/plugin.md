# AnalyseMany

Inputs are all [Analyse parameters](../analyse/plugin.md), in the same order, with `radius` inserted immediately before `prefix`. `radius` defaults to 1; `delta` defaults to 1 and is a positive distance step here. Both use int32 saturation and must be positive. All other defaults, omitted-value distinctions and parameter effects are exactly those of Analyse. Returns a video-node array under `clip`.

Let R=radius,D=delta. Output length is 2R. For r=1 through R,

$$O_{2(r-1)}=Analyse(super,delta=rD,\theta),$$
$$O_{2(r-1)+1}=Analyse(super,delta=-rD,\theta),$$

where theta contains every other original argument, preserving omission of optional parameters such as tff and pzero. These equalities specify each member's video information, pixels, properties and numerical results; they do not require identical internal node objects.

Members are independent video sequences, not interleaved frames or concatenated vector arrays. Each retains the source frame count/rate and uses super[n] as its carrier. End-of-sequence reference availability follows Analyse separately for each signed delta. Never remove a member because its current frame lacks a reference.

Require 2R and all rD values to be representable in the host/output and signed-32 delta domains, and enough resources for the complete result. If any member cannot be created, return a controlled whole-call error, not a successful short array, and identify a failing member by index. No ordering between simultaneous independent errors or exact error prefix is required. Frame-time errors remain associated with the requested member/frame.

No new mathematical kernel is required. Concurrent members must not accidentally share writable results; reuse of immutable inputs is allowed.

Examples: R=3,D=2 gives deltas [2,-2,4,-4,6,-6]. At n=8 the references are [10,6,12,4,14,2]. For R=2,D=1,n=0 on a ten-frame source, all four outputs have ten frames; members at delta -1 and -2 have metadata but no selected-prefix vectors/SAD at n=0, including when such arrays were inherited from the input. A negative D is an error, not a direction-order switch.
