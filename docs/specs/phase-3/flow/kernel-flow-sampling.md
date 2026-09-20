# Dense single-reference sampling

Inputs are visible plane geometry, dense integer Vx,Vy in plane-pel units, pel p, time coefficient t in [0,256], and a reference Super level-zero plane with its logical phases. Output has the same sample type and visible dimensions as that render plane. Time quantization is

$$t=\operatorname{trunc}(\operatorname{fl64}(\operatorname{fl64}(time\,256)/100)),$$

where time is finite binary64 in [0,100]. This operator does not select the reference frame and does not interpolate between two video images.

## Coordinate map

For each visible plane pixel x,y compute exact integers

$$a_x=px+\left\lfloor\frac{V_x(x,y)t+128}{256}\right\rfloor,\qquad a_y=py+\left\lfloor\frac{V_y(x,y)t+128}{256}\right\rfloor.$$

Split each total coordinate by floor division:

$$q_x=\lfloor a_x/p\rfloor,\quad \alpha_x=a_x-pq_x,\qquad q_y=\lfloor a_y/p\rfloor,\quad \alpha_y=a_y-pq_y.$$

For this plane's actual level-zero Super padding hP,vP, copy the single stored sample

$$out(x,y)=P_{\alpha_x,\alpha_y}(h_P+q_x,\ v_P+q_y).$$

The phases and their defined domains follow [Super interpolation](../../phase-1/super/kernel-subpixel-interpolation.md). Do not interpolate again, apply a SAD weight, average overlapping blocks or blend with clip[n]. Copy float32 samples without numeric conversion, preserving their stored representation; Flow introduces no normalization or clamping of image values. This is distinct from the finite numeric requirements on computed mask scores.

## Admission and error timing

At creation require the declared logical phases, all render planes, valid fixed geometry and a sampleable zero-displacement integer-phase position for every visible output sample. This checks t=0,f=0 coordinates. No all-candidate search-domain admission from Analyse, Recalculate, Compensate or Degrain applies here: Flow samples a derived dense field rather than every possible public block vector.

For an available reference, after validating the complete public field, applying actual field correction and generating all dense components, preflight every actual output sampling coordinate in every processed plane. All must belong to the selected phase's defined logical domain. Perform this preflight before copying any reference image samples or publishing output. Failure is a controlled frame error. It is forbidden to clamp a sampling position, skip a pixel, increase padding, synthesize a missing edge, switch phases or fall back to clip[n] to conceal the failure.

This rule covers chroma floor conversion, dense-grid interpolation, time rounding, field shifts and built-in quarter-phase edges. Public array bounds alone do not establish safety. Frame geometry/domain changes, missing required payloads and unsafe storage views are errors. An unavailable reference takes the plugin's fallback before dense sampling; it does not require hypothetical displaced footprints to be safe.

Only visible output pixels are sampled. Distinct planes and outputs may have different valid strides. Inputs remain immutable and generated output cannot alias their sample storage.

## Examples

- t=128 gives offsets 1,0,-1 for V=1,-1,-3 respectively. These are halfway-toward-positive-infinity results, not truncation toward zero or floor without the bias.
- p=2,x=0,Vx=-3,t=256 gives ax=-3,qx=-2,alphaX=1. With hP=2 the read is phase 1 column 0. With hP=1 it is column -1 and the requested frame fails.
- At time=0 every offset is zero; the result uses the reference's integer-phase visible samples. Current pixels are not selected merely because time is zero.
- YUV420,Wr=Hr=W=H=16,8x8 blocks,overlap=0,pad=3,p=2,one level,fields=false,time=100: zero vectors pass creation and actual sampling in all planes. If the top-left vector is vx=-6,vy=0 and all others are zero, that vector is inside its public bounds. At the first chroma output pixel, edge extension retains small Gx=-3, so ax=-3,qx=-2,hP=1 gives column -1: the frame fails before sampling. With pad=4 the same field is sampleable. With pad=3 and time=0 it is also sampleable because its effective offsets are zero.
- At p=4, a built-in phase with alphaX=3 excludes its final logical column. A request to that column fails even if its allocation contains bytes there. The same coordinate is allowed on a supplied external phase whose declared defined domain includes it.
