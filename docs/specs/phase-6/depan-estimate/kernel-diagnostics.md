# Estimate diagnostics and text rendering

Inputs are a completed output frame after optional correlation display, output index n, final motion, and current basic confidence Tn. For info=true, replace `DepanEstimate_info` with one UTF-8 data value:

`fn=<n> dx=<dx:2> dy=<dy:2> zoom=<z:5> trust=<Tn:2> bad=<bad>`

Use one space between fields. bad=0 for final good motion and 1 otherwise. The decimal counts denote fixed-point formatting, not literal characters. Use the [Phase 5 formatting contract](../../phase-5/depan-analyse/kernel-diagnostics.md#strings): exact promotion from binary32, nearest-even decimal rounding, period separator, signed-zero preservation and truncation to the first 127 bytes, without a newline or terminating NUL. Use final reset motion on a bad frame but retain its basic Tn. Rotation is not printed. Frame zero prints trust=0.00.

Then invoke the host property-text renderer selecting only `DepanEstimate_info`. On VapourSynth this is text.FrameProps with clip=the completed base output and props=["DepanEstimate_info"], all other arguments omitted. The resulting pixels and properties are the public output. With show=true, render text after the correlation display; the text may cover it. The [shared renderer contract](../../phase-5/depan-analyse/kernel-diagnostics.md#renderer-contract) governs missing capability, creation/frame failures and host-dependent text pixels. info=false has no renderer dependency and preserves any inherited same-named diagnostic property.

## Examples

- n=0 always has final reset motion and Tn=0, so the text is `fn=0 dx=0.00 dy=0.00 zoom=1.00000 trust=0.00 bad=1`.
- Final dx=1,dy=0,z=1,Tn=88.00879669189453 at n=2 gives `fn=2 dx=1.00 dy=0.00 zoom=1.00000 trust=88.01 bad=0`.
- A temporal rejection with Tn=5 prints zero motion, zoom 1.00000, trust 5.00 and bad=1. It does not print the discarded displacement or reset the displayed confidence to zero.
