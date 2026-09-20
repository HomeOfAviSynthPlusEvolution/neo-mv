# Dense motion and occlusion fields

Inputs are eligible main fields B with DeltaFrame=+d and F with DeltaFrame=-d, optional eligible BB/FF, render plane geometry, pel p, interpolation time t in [0,256], and ml. Outputs are dense signed displacement components and dense integer masks mB,mF in [0,255] for every visible sample of each render plane. They are intermediate mathematical data, not output frame properties.

## Motion grids

Generate each needed B,F,BB,FF plane field using [Phase 3 dense vectors](../../phase-3/flow/kernel-dense-vector-field.md), with field correction f=0. Thus clip each public component to [-32768,32767], floor-divide by that plane's ratio, then apply the exact [integer grid resampler](../../phase-3/vector-length-mask/kernel-grid-resampling.md) with plane-specific B,O,Wr,Hr. Do not time-scale before this resampling. All source fields must have passed public validation before saturation.

## Occlusion grids

At creation convert ml to binary32; require finite ml>0. Define f=fl32(1/ml), require finite f and finite required intermediate values 80f, ax,ay from the [occlusion kernel](../../phase-3/occlusion-mask/kernel-occlusion.md). Use luma block steps and pel in these constants, regardless of render precision or plane ratios. This is required even when every output might bypass interpolation.

Use that occlusion event kernel with these exact inputs:

| Grid | Input vectors | Signed direction | Event time coefficient | Maximum M | Gamma |
| --- | --- | --- | --- | --- | --- |
| GB | Public unsaturated B components | +d | 256-t | 255 | 1 |
| GF | Public unsaturated F components | -d | t | 255 | 1 |

Event differences, interval orientation, inclusive bounds and empty-interval behavior are unchanged. Convert every finite score to trunc(min(L,255)) before combining contributions by maximum. Keep the resulting small grids as exact integers in [0,255]. Do not derive events from saturated, time-scaled or chroma-divided components. No events are generated from BB/FF.

For each actual render plane, resample the same luma-derived GB and GF small-grid values with that plane's Bx/rx,By/ry,Ox/rx,Oy/ry and visible Wr/rx,Hr/ry. This produces mB,mF. Chroma changes spatial scaling only: do not recompute convergence amplitudes or event intervals using the chroma block steps. The final masks remain in [0,255] for integer8, integer16 and float render images alike; 255 is not 256.

## Examples

- Nx=2,Ny=1,4x4 luma blocks,no overlap,p=1,ml=80,t=256: F with horizontal vectors [4,0] gives GF=[255,0]. On an eight-column Y plane, mF=[255,255,223,159,96,32,0,0]. On a four-column chroma plane with rx=2, mF=[255,191,64,0]. These are unchanged when render depth changes from 8 to 16 or float.
- With the same B vectors [4,0] and t=256, B's event coefficient is zero, but GB=[0,255], not an all-zero grid. Event time zero does not remove convergence events.
- A constant vector field has no convergence event and produces mask zero. With a luma component -1 and rx=2, the dense motion starts from small component -1, whereas its occlusion differences still use the original -1.
