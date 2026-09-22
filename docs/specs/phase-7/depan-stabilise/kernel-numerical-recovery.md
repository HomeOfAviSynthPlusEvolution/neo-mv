# Inertial numerical recovery

This contract applies to method=0 with b<n, after successful parameter normalization, motion decoding, cumulative-map construction and the required cumulative-map inverses. It describes recovery from overflow in inertial smoothing and propagation of the resulting non-finite values into the current correction. It does not define a general exception-to-image conversion.

## Arithmetic boundary

Within inertial smoothing, allow a finite binary32 operation to overflow to signed infinity. Continue the prescribed expressions with IEEE binary32 infinity and quiet-NaN semantics through optional inertial adaptive zoom, construction of the raw current correction, its conversion to motion, and end taper. An invalid operation caused by these non-finite operands produces NaN. Do not trap the first overflowing intermediate, clamp it to the largest finite value, change precision, or immediately replace the correction. NaN payload and sign are not observable outputs of this contract.

Finite operations retain the phase's independent rounding and operation order. Comparisons with NaN are unordered: ordered comparisons and equality are false, while != is true. Absolute value of infinity is positive infinity; absolute value of NaN remains NaN. Elementary functions accept propagated non-finite operands with their IEEE mathematical classifications: atan(+/-infinity)=+/-pi/2 rounded to binary32, exp(-infinity)=+0, exp(+infinity)=+infinity, log(+infinity)=+infinity, sqrt(+infinity)=+infinity; NaN arguments, sqrt/log of negative infinity and sin/cos of infinity give NaN. Existing explicitly selected comparisons, including the nonpositive-scale replacement and adaptive upper bound, still apply. Do not discard the whole result solely because an intermediate was non-finite: its eventual correction tuple determines recovery.

This allowance does not admit non-finite public arguments or motion properties. It does not waive an independent zero divisor, negative square-root argument or invalid cumulative inverse with finite operands; in particular, tzoom=0 still fails when its reciprocal is required. It does not extend to method=1, neighbor-map construction, diagnostics or pixel-coordinate arithmetic. Those retain their existing checked domains. Overflow produced by a finite correction-limit expression itself also retains the existing frame-error rule.

## Recovery decision and output

The correction-limit operator admits the possibly non-finite motion tuple from the boundary above. At each component's existing ordered limit step, test that component for finiteness before its magnitude comparison. A non-finite component triggers the same whole-tuple reset as a negative hard limit: motion becomes (dx=+0,dy=+0,r=+0,z=z0) and b becomes n. Continue the remaining component limit steps. This test is independent of whether the corresponding public limit is positive, zero or negative. A finite large correction follows the ordinary finite limit rules; there is no additional divergence-magnitude threshold.

Recovery replaces the current correction, not the data clip's motion properties. It does not edit a decoded good-motion flag, create a persistent scene boundary or alter another output frame's interval. Rebuild the final current map through the ordinary motion conversion. The resulting map must be finite before neighbor selection and rendering; any subsequent failure is still a controlled frame error.

The recovery scale is the reciprocal initial zoom z0, not necessarily 1. It is not the computed adaptive scale. End taper has already acted on the incoming correction and does not subsequently taper the reset tuple. Remaining limit steps still apply, as in an ordinary hard reset.

Render the recovered map with the normal public parameters. prev>0 still creates its requested layer; because b=n, that previous pair is (n,Q). next>0 retains its ordinary next-motion decoding and selection and can use later images. No future motion is marked invalid by recovery. Current subpixel, first-layer mirror/blur, plane geometry, Preserve behavior and property inheritance remain unchanged. Therefore recovery is not a generic copy of clip[n], even when z0=1; identity resampling and neighboring layers can affect edge pixels.

With info=true, diagnostics describe the final recovered correction and use the b=n `BASE!` form. With info=false, inherited properties, including existing diagnostic strings, remain unchanged. Required data, image, allocation and text-rendering failures are not swallowed by recovery.

Every output is determined by its own required public inputs. A recovery for output n neither resets nor seeds the smoothing of a later requested frame. Different request orders, repeated requests and independent instances must give the same per-frame result. Recovery does not promise temporal continuity or successful stabilization during a numerically unstable motion interval.

## Behavioral cases

- For a 48x48 GRAY8 clip at 24000/1001 fps with at least eight frames, valid motion (20,0,0,1) after synthetic frame zero, default parameters except dxmax=+10 or -10, output n=7 recovers. With default rendering and initzoom=1 its visible pixels equal clip[7]. The same input does not authorize an earlier error at the first overflowing smoothing operation.
- Keep that fixture's later motions valid, extend it to 12 frames, and replace only data[7]'s validity by invalid in a separate control. At n=7 the recovery output matches that control's rendered result for modes 0,1,2 with default parameters, and separately with initzoom=1.2, addzoom=true, fitlast=4, prev=2, next=2, or prev=2,next=2,mirror=15,blur=3. This is an output comparison; the original data remains valid. Making every later motion invalid is a different control and can change next-layer pixels.
- At the limit-operator interface, a non-finite dx resets the whole tuple for either sign of dxmax. A finite dx that triggers an ordinary hard reset can remove a non-finite later component before that component is visited. The ordered whole-tuple rule governs both cases.
- Malformed visited motion, a required tzoom=0 reciprocal, an invalid cumulative inverse, and a failure during next-layer selection remain errors. None is an instruction to return an unmodified source frame.

These cases define recovery behavior, not a pixel tolerance. General reference equivalence outside the arithmetic boundary above, or for other non-finite arithmetic sites, is not asserted by this contract.
