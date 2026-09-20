# Nearest-sample compensation

Inputs are a source plane, a finite plane map, mirror bits, blur and border B. Output is a same-sized integer plane. Use [coordinates](kernel-sampling-coordinates.md) for mode 0 and [border operators](kernel-border-sampling.md). Samples are copied without arithmetic when directly selected.

For T or Z, reflect j vertically once. If the resulting row is invalid, output B. On a valid row, read P(i,j) for 0<=i<W; otherwise use ordinary horizontal edge fill, including its blur and per-column clamp.

For R, use the original nearest i,j first. If both are valid, read P(i,j). Otherwise use general two-axis reflection and its sample-or-B result. R never uses horizontal blur or horizontal clamp. Do not reflect a fractional coordinate before integer rounding.

## Examples

- A translation tx=2,ty=0,mirror=0 takes output (0,0) from source (2,0). Columns whose translated i>=W receive B.
- On a valid row [0,10,20,30,40], T with tx=-2 at x=0,left enabled,blur=2 gives 15. An R map reaching i=-2 on the same row gives column 2's value 20, regardless of blur.
- With mirror=0, T at X=-0.75 selects border at x=0, while R at X=-0.75,Y=1 selects P(0,1), because its nearest rounding truncates.
- On a 4x4 plane, identity with any mirror/blur produces an exact image copy in this mode; all coordinates are already in range. At very large dimensions the specified binary32 coordinate rounding still applies; identity is not a general permission to bypass it.
