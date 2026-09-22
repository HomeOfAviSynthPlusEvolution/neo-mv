# Stabilization diagnostics

Inputs are the completed base frame, final n,b and final current Q before per-plane scaling or translation clamping. When info=true, convert Q to motion once more with forward=true,aspect a,center (cx,cy). This conversion is required even for a scene-start branch. Conversion failure is a frame error. Neighbor-layer maps do not enter this diagnostic.

Replace `DepanStabilise_info` with one UTF-8 data value, using the fixed-point formatting, signed-zero, nearest-even decimal rounding and 127-byte truncation rules from [Phase 5 diagnostics](../../phase-5/depan-analyse/kernel-diagnostics.md).

If b!=n:

`frame=<n> base =<b> dx=<dx:2> dy=<dy:2> rot=<r:3> zoom=<z:5>`

If b=n:

`frame=<n> BASE!=<b> dx=<dx:2> dy=<dy:2> rot=<r:3> zoom=<z:5>`

The space before the lower-case equals sign and the exclamation mark before the upper-case equals sign are literal. No trailing newline or NUL is stored. Do not export the correction under the five Depan motion properties: those retain clip[n]'s values if present.

Apply the Phase 5 host text-renderer contract to this key only. For VapourSynth invoke text.FrameProps(clip=base,props=["DepanStabilise_info"]) with all other arguments omitted. Missing or rejected renderer support is a creation error; rendering failure is a frame error. On another host require equivalent rendering support or reject info=true. Pixel comparisons of info=true bind both sides to the same renderer/version. info=false leaves inherited diagnostics unchanged and requires no renderer.

## Examples

- For a direct final Q=(tx=2,ty=+0,u=1,v=+0,w=+0,h=1),a=1,center=(24,24),n=4,b=0, the text is `frame=4 base =0 dx=2.00 dy=0.00 rot=-0.000 zoom=1.00000`. The negative zero follows -atan(+0).
- For the same Q,n=b=4, the prefix is `frame=4 BASE!=4`; the remaining numeric fields are unchanged.
- With info=false and an inherited DepanStabilise_info="earlier", keep that string exactly. With info=true, replace it with the new diagnostic before rendering.
