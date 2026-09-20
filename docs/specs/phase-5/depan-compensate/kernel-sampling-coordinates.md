# Plane transforms and finite-precision coordinates

Inputs are a finite six-coefficient map `(tx,ty,u,v,w,h)`, a plane description and interpolation mode. Outputs are a plane map, its numeric class and coordinates for each output pixel. Apply the [Phase 5 numeric rules](../README.md#common-contracts). Integer plane samples have nominal precision b=8..16 and maximum M=2^b-1. All visible source samples must lie in [0,M]. Source and output have the same positive plane dimensions W,H. Modes 0 and 1 support W,H>=1; mode 2 requires W>=1,H>=2. A smaller height for mode 2 is rejected at plugin creation because the near-edge rule can require the next row. A width below four is supported: it simply has no complete cubic footprint. No unsafe tap access or inferred extension is allowed.

## Plane conversion

For luma or GRAY, use the input map. For chroma, use the following map coefficients, rounded after each operation:

| Format | tx | ty | u | v | w | h | Plane blur |
| --- | --- | --- | --- | --- | --- | --- | --- |
| YUV444 | tx | ty | u | v | w | h | blur |
| YUV422 | tx/2 | ty | u | v/2 | w*2 | h | floor(blur/2) |
| YUV420 | tx/2 | ty/2 | u | v | w | h | floor(blur/2) |

Both chroma planes use the same coefficients; their memory strides may differ. No chroma-location property introduces an additional offset. Plane W,H come from actual subsampling dimensions, not from a ceil guess for an unsupported clip format.

After this conversion, define L=fl32(W+H+64), with the integer sum formed exactly. Clamp tx and ty independently to [-L,L]. Do not clamp the other four coefficients or apply these clamps to diagnostic motion. The input coefficients must be finite before clamping. Then select the numeric class:

- T: u=h=1 and v=w=0 (translation).
- Z: v=w=0, otherwise (axis-aligned scale/translation).
- R: all other maps.

These exact comparisons determine visible rounding and edge behavior. They are part of the contract, even when two maps are geometrically very close.

## Coordinates by class

Output x,y are integer coordinates, starting at zero. Integer operands in floating expressions convert to binary32 before arithmetic.

For T, set Y=fl32(ty+fl32(y)). Nearest uses i=x+floor(fl32(tx+0.5)) with exact integer addition, and j=floor(fl32(Y+0.5)). Bilinear/bicubic use i=x+floor(tx), fx=fl32(tx-fl32(floor(tx))), j=floor(Y), fy=fl32(Y-fl32(j)). Bicubic's near-edge row rule instead uses the constant `fyT=fl32(ty-fl32(floor(ty)))`.

For Z, independently compute X=fl32(tx+fl32(u*fl32(x))) and Y=fl32(ty+fl32(h*fl32(y))). Nearest uses floor(fl32(X+0.5)) and floor(fl32(Y+0.5)). Other modes use their floors and binary32 fractional differences.

For R with nearest or bilinear, start each row with X0=fl32(tx+fl32(v*fl32(y))) and Y0=fl32(ty+fl32(h*fl32(y))). The coordinate at column x is defined recursively by X(x+1)=fl32(Xx+u) and Y(x+1)=fl32(Yx+w). Do not replace these rounded recurrences with one multiplication. No recurrence beyond the last visible coordinate is required. Nearest uses i=trunc(fl32(Xx+0.5)), j=trunc(fl32(Yx+0.5)), including for negative coordinates. Bilinear uses floor and fractional differences; equivalently truncate, subtract, and for a negative fraction decrement the integer and add 1 to the fraction, with each float operation rounded.

For R with bicubic, independently compute

$$X=\operatorname{fl32}(\operatorname{fl32}(t_x+\operatorname{fl32}(u\operatorname{fl32}(x)))+\operatorname{fl32}(v\operatorname{fl32}(y))),$$
$$Y=\operatorname{fl32}(\operatorname{fl32}(t_y+\operatorname{fl32}(w\operatorname{fl32}(x)))+\operatorname{fl32}(h\operatorname{fl32}(y))).$$

Use floor and binary32 fractional differences. Fractions may round to exactly 1 for a tiny negative coordinate; interpolation quantizers therefore include their endpoint coefficient.

All actual coordinates and required rounding intermediates must be finite. Integer coordinates and subsequent mirror/blur arithmetic are exact; a particular int32 temporary overflowing is not permission to reject a finite supported map. Resolve logical borders before forming memory addresses. Out-of-image finite coordinates are normal border inputs, not errors. A non-finite actual coordinate is a frame error; do not turn it into border color.

## Examples

- T with tx=-0.75 gives nearest i=-1 at x=0. R at X=-0.75 gives i=0 because trunc(-0.25)=0. Unifying these nearest rules changes output.
- A luma map with tx=4,ty=2,v=0.25,w=-0.25 becomes tx=2,ty=1,v=0.25,w=-0.25 on 420; on 422 it becomes tx=2,ty=2,v=0.125,w=-0.5.
- A 4x4 plane uses L=72. tx=1000 clamps to 72 before coordinate evaluation. The same input map has a different translation clamp on a differently sized plane.
- A 6x8 YUV420 clip has a 3x4 chroma plane and is supported, but chroma mode 2 uses only its edge rules. A 2x2 YUV420 clip has 1x1 chroma: modes 0/1 are supported and mode 2 is rejected. A 2x4 YUV420 clip has 1x2 chroma and is supported in all three modes.
