# Neighbor confidence and final motion

Input is the basic result at n in a clip of F>=1 frames, and basic confidences at existing immediate neighbors. Output is the final (dx,dy,r,z,good) tuple and the unchanged current basic confidence for diagnostics. The basic records come from [window combination](kernel-two-window-motion.md), before this temporal rule is applied. Never use a neighbor's final decision recursively.

Let t be the configured trust limit. Start good=basic.good. Set it false if either condition holds:

- n>0 and Tn<fl32(t*2) and Tn<fl32(0.5*T[n-1]).
- n+1<F and Tn<fl32(t*2) and Tn<fl32(0.5*T[n+1]).

All comparisons are strict. Neighbor good flags do not suppress their confidence; a rejected zoom can still contribute its retained confidence. No nonexistent neighbor is invented. Frame zero's basic confidence is zero, so it cannot by itself cause the backward comparison to reject a later nonnegative confidence.

If the final good is false, return exactly (dx,dy,r,z,good)=(+0,+0,+0,1,0). Otherwise preserve basic dx,dy,z and set r=+0,good=1. Do not alter Tn after a temporal rejection. [Motion property transport](../../phase-5/depan-analyse/kernel-motion-properties.md) supplies exact float64 promotion of binary32 values and one-element output counts.

## Required observations

To produce frame n, obtain basic records for indices max(0,n-1), n and min(F-1,n+1), deduplicating repeated indices. Each record j analyzes source frames max(0,j-1) and j. The resulting source-frame set lies between max(0,n-2) and min(F-1,n+1), inclusive. This dependency is on input data, never on earlier output requests. All these basic records are required even if the current record is already bad; their correlation or parity errors remain errors. Caches may reuse identical immutable calculations.

With fields=true, each required basic index j needs parity from clip[j] unless explicit tff provides it. A frame used only as a preceding image does not separately need parity. Display and text rendering are required only for the requested output n, not for neighboring basic records.

## Examples

- t=4,Tn=5,T[n-1]=20 gives 5<8 and 5<10, so good becomes false. Tn=8 instead fails the first strict test; Tn=5,Tneighbor=10 fails the second. Neither equality causes rejection.
- n=1 with T0=0 has no backward rejection for nonnegative T1, but an actual T2 can trigger the forward test.
- F=5,n=2 requires basic records 1,2,3 and source images 0,1,2,3. With fields=true only source frames 1,2,3 need parity. F=1 requires only basic record 0 and source frame 0, and final motion is invalid.
- A basic good=true result (dx,dy,z)=(1,0,1) that fails a neighbor comparison exports (0,0,0,1,0), while its diagnostic trust remains Tn.
