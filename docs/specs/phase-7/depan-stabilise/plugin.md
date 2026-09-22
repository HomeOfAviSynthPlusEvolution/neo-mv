# DepanStabilise

Returns one video node under `clip`. Inputs appear in the following order. Float means a finite binary64 argument rounded to finite binary32 before domain checking. Integer scalars saturate to signed int32 before validation; booleans use the original int64 zero/nonzero truth. Read element zero only; a supplied empty or wrongly typed value is an error. Defaults apply only when omitted.

| Argument | Type | Default | Domain and behavior |
| --- | --- | --- | --- |
| clip | Video node | Required | Constant GRAY or YUV420/422/444 integer 8..16-bit images, known positive frame rate |
| data | Video node | Required | At least F frames; public Depan properties are consumed, pixels are not |
| cutoff | Float | 1 | >0; temporal radius/lookback and inertial response |
| damping | Float | 0.9 | Any finite value; inertial damping, negative allowed |
| initzoom | Float | 1 | >0; reciprocal is initial sampling scale |
| addzoom | Boolean integer | 0 | Enable method-specific adaptive zoom |
| prev | Integer | 0 | >=0; previous fill search extent |
| next | Integer | 0 | >=0; next fill search extent |
| mirror | Integer | 0 | 0..15; top=1,bottom=2,left=4,right=8, first layer only |
| blur | Integer | 0 | >=0; horizontal reflected-border averaging extent |
| dxmax | Float | 60 | Finite; method-0 nonlinear factor and translation limit |
| dymax | Float | 30 | Finite; method-0 nonlinear factor and translation limit |
| zoommax | Float | 1.05 | Finite; normalized method-0 scale limit |
| rotmax | Float | 1 | Finite; method-0 nonlinear factor and rotation limit |
| subpixel | Integer | 2 | 0 nearest,1 bilinear,2 bicubic for current layer |
| pixaspect | Float | 1 | >0; aspect before fields division |
| fitlast | Integer | 0 | Any signed int32; positive end taper for method 0 |
| tzoom | Float | 3 | >=0; adaptive zoom time/window parameter |
| info | Boolean integer | 0 | Diagnostic property plus host text rendering |
| method | Integer | 0 | 0 inertial,1 symmetric window |
| fields | Boolean integer | 0 | Divide effective aspect by 2 |

Negative correction limits are not rejected; they trigger the specified whole-correction reset when exceeded. Zero translation/rotation limits are active limits. Method 1 still validates all public argument types/domains and shared normalization, but does not apply inertial coefficients, correction limits or fitlast. There is no tff argument or field-parity read.

## Construction and frame behavior

Require positive dimensions, F in [1,2147483647], positive frame-rate numerator and denominator and host-supported format. Each plane requires Wp,Hp>=1 for modes 0/1, and Wp>=1,Hp>=2 for mode 2. Apply [parameter normalization](kernel-parameters.md); derived-value failures are creation errors. Data need not share clip's format, dimensions or frame rate. Its image samples and extra frames beyond F are unused. Unsupported host formats, required text support failure and insufficient data length are creation errors.

For output n, perform [interval selection](kernel-motion-interval.md) and [cumulative motion](kernel-cumulative-motion.md). Method 0 uses [inertial smoothing](kernel-inertial-smoothing.md), optional [adaptive zoom](kernel-inertial-zoom.md), and [taper/limits](kernel-correction-limits.md), with the explicit b=n bypass. Method 1 uses [window smoothing](kernel-window-smoothing.md) and its required motion round trip. Select [neighbor layers](kernel-border-selection.md), then render [previous, next and current](kernel-layer-composition.md) in that order. Apply [diagnostics](kernel-diagnostics.md) if requested.

Preserve clip's dimensions, format, frame count and frame rate. The base output copies all properties of clip[n], including Depan motion keys, scene tags, range/color metadata and any diagnostic keys. Only info=true replaces DepanStabilise_info and invokes the external renderer, whose output is final. There is no new exported correction-vector buffer or correction-motion property tuple. Pixels always pass through the specified renderer even for identity or invalid motion; no generic frame-copy shortcut bypasses border behavior.

Malformed visited motion properties, invalid required transform arithmetic, divergent smoothing, zero adaptive normalization, unavailable required frames and allocation failure are controlled frame errors. Never skip a failing candidate, silently enlarge padding, interpolate missing properties or depend on the prior request. A parameter value accepted at construction can fail on a particular frame when its otherwise-unused arithmetic branch becomes necessary.

## Required behavioral cases

- Synthetic frame-zero motion, scene cuts at both interval edges, asymmetric clip ends and symmetric interval shrinkage.
- Inertial first frame after a cut, with both addzoom settings and initzoom!=1; finite divergence/error and negative hard-reset limits.
- Window radius one, fractional weights, separate diagonal averaging, adaptive radius zero and cumulative-map self-composition.
- fitlast positive/zero/negative, limit equality, two-pass translation limit and one-pass scale/rotation limit.
- Previous/next winning image different from the endpoint map; invalid next motion overriding the best index; malformed later next-range data.
- Multiple layers, real zero samples versus Preserve, first-layer mirror only, all three sampling classes/modes, 420/422 chroma conversion and unequal strides.
- Inherited properties, signed-zero diagnostics, fields without `_Field`, random request order and simultaneous independent instances.
