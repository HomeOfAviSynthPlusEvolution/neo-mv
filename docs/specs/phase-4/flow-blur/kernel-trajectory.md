# Motion-blur trajectory samples

Inputs are dense per-plane integer fields F and B, positive int32 prec, pel p, visible plane geometry and blur coefficient t in [0,256]. Outputs are an ordered list of sample coordinates and sample count for each visible pixel. All positions refer to one current Super level-zero image. The names F and B denote two trajectory directions, not two image sources.

Generate each dense field using [Phase 3 dense vectors](../../phase-3/flow/kernel-dense-vector-field.md) with field correction f=0: public validation, signed16 saturation, per-plane floor conversion, then exact grid resampling. There are no occlusion masks and no ml parameter. For finite binary32 blur in [0,200] define at creation

$$t=\operatorname{trunc}(\operatorname{fl32}(\operatorname{fl32}(blur\,256)/200)).$$

## Trajectory coordinates

At each visible plane pixel x,y let q=(px,py). For each dense vector G in the ordered directions F then B, form exact integers

$$z_G=tG,\qquad a_G=\max(|z_{Gx}|,|z_{Gy}|),\qquad m_G=\left\lfloor\frac{\operatorname{trunc}(a_G/prec)}{256}\right\rfloor.$$

If mG=0, that direction contributes no additional samples. If mG>0, define componentwise eG=trunc(zG/mG). For i=1 through mG, in increasing order, use

$$q_{G,i}=q+\left(\left\lfloor i e_{Gx}/256\right\rfloor,\left\lfloor i e_{Gy}/256\right\rfloor\right).$$

The full list is center q, then all F samples in ascending i, then all B samples in ascending i. Repeated coordinates remain repeated contributions. First truncate the step, then multiply it by i; using floor(i*zG/(256*mG)) can change sample positions. Negative step division truncates toward zero, while coordinate division floors.

Dense components lie in [-32768,32767], hence each mG<=32768 and the total count 1+mF+mB<=65537. Products and absolute values must be computed without signed overflow. prec is not a minimum number of taps; a larger prec can eliminate a direction's additional samples.

## Domain checks

Use [the phase-point map](../flow-inter/kernel-bidirectional-sampling.md) to split each generated point into phase and padded coordinates, without applying that kernel's interpolation-time map. Require the center and every trajectory point to belong to their logical phase domains for every visible sample in every render plane. Preflight them all before reading image samples or publishing a generated frame. A bounds proof can replace explicit coordinate enumeration if equivalent for the actual field.

Do not clamp, shorten a trajectory, omit repeated points, synthesize quarter-phase edge values or fall back to clip to conceal an invalid point. Creation checks only the fixed geometry and center positions. The entire actual trajectory is checked at frame time, including at blur=0, when the list contains only centers. An unavailable input pair is handled before this operator.

## Examples

- p=1,blur=200,t=256,prec=1,F=(2,0),B=(-2,0) gives mF=mB=2. The ordered X offsets are [0,1,2,-1,-2]. All samples come from the current Super image.
- For those vectors with blur=50,t=64, both counts are zero. With blur=200,prec=3, both counts are also zero. Only the center remains.
- Take t=1,F=(-257,514),prec=1. Then zF=(-257,514),mF=2,eF=(-128,257); additional offsets are (-1,1),(-1,2). Directly dividing the untruncated path at the last point would incorrectly use X=-2.
- YUV420,actual/working16x16,8x8 blocks,no overlap,p=2,pad=3,blur=200,prec=1: take a constant public F=(-6,0),B=(0,0) field with zero SAD. All vectors satisfy their public block bounds. At the first chroma output pixel, dense F=(-3,0),mF=3, with extra plane-pel X offsets -1,-2,-3. The last point gives qx=floor(-3/2)=-2 and hP=floor(3/2)=1, hence padded column -1. The frame fails before sampling. Changing padding to4 makes these fields' full visible trajectories sampleable in all planes. Keeping padding3 and setting blur=0 also gives safe center-only trajectories, after the ordinary input-eligibility checks.
