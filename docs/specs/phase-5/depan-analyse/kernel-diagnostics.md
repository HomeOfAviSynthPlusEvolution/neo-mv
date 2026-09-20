# Depan diagnostics and text rendering

Inputs are a completed base frame, frame indices, parameters and the computed motion/fit scalars. Output is that frame with a diagnostic property followed, when info=true, by the host text renderer. info=false neither constructs new diagnostics nor removes an inherited diagnostic key.

## Strings

Use ASCII characters encoded as one UTF-8 data value. Format floating values after exact promotion from binary32, using fixed-point decimal, a period decimal separator, no thousands separators, no leading plus sign and the decimal places below. Round to nearest decimal with ties to even; preserve the sign of a negative zero, including a negative value that rounds to zero. Integer indices have ordinary decimal spelling. Truncate the completed byte sequence to its first 127 bytes if necessary. No trailing newline or terminating NUL is part of the property data.

DepanAnalyse replaces `DepanAnalyse_info` with:

`fn=<n> iter=<iter> error=<E:3> dx=<dx:2> dy=<dy:2> rot=<r:3> zoom=<z:5> bad=<bad>`

Here `:p` denotes exactly p decimal places, not literal output. bad=0 for valid motion and 1 for invalid. Use the exported motion values, including any field correction; invalid motion uses the standard tuple. E and iter come from the fit, even for a rejected estimate.

DepanCompensate replaces `DepanCompensate_info` on its render branch with:

`offset=<offset:2>, <s> to <n>, dx=<dx:2>, dy=<dy:2>, rot=<r:3> zoom=<z:5>`

Convert the final composed map after field correction back to motion using its aspect, center and forward flag, before per-plane scaling or translation clamping. Conversion failure is a frame error when info=true. A bypass creates no new diagnostic property, but retains any inherited one.

## Renderer contract

With info=true, apply the host's property-text renderer to the completed base node, selecting only this plugin's diagnostic key. For VapourSynth this is `text.FrameProps(clip=base, props=[key])`, with every other argument omitted so that the installed renderer supplies its defaults. The renderer's resulting pixels and property map are the output. The wrapper applies on bypass frames too, even when their diagnostic property is absent or inherited. Missing renderer capability or a rejected invocation is a controlled creation error; rendering failure is a frame error. Never silently reduce info=true to a property-only operation.

This is an external rendering dependency. The text renderer need not be reimplemented as a motion kernel. On another host, an adapter must supply equivalent property-text rendering semantics, or report info=true unsupported. Pixel comparisons of this mode bind both sides to the same renderer/version and arguments; there is no host-independent bitmap/font definition here. info=false has no text-renderer dependency and has fully specified motion/image output.

## Examples

- Ineligible analysis at n=0 with error=15 generates `fn=0 iter=0 error=30.000 dx=0.00 dy=0.00 rot=0.000 zoom=1.00000 bad=1` before rendering.
- For compensation from s=4 to n=5, offset=1, center (8,8), a=1, forward=true and a final map tx=2,ty=+0,u=h=1,v=w=+0, the text is `offset=1.00, 4 to 5, dx=2.00, dy=0.00, rot=-0.000 zoom=1.00000`. The negative zero follows theta=-atan(+0), not a formatting error.
- info=false preserves an inherited string under the diagnostic key unchanged, even though it describes an earlier operation.
