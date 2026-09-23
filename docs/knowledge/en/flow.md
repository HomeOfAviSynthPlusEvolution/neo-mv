# Flow: from block vectors to per-pixel sample positions

## 1. What the function computes

`Flow` expands block vectors into a displacement at each visible pixel, then samples one reference Super image. It neither averages overlapping blocks, weights images by SAD, nor blends two frames.

## 2. Objects and notation

Current frame n, reference n+d, pel p, time coefficient `t=trunc(fl64(fl64(time·256)/100))`. Each rendering plane uses its own ratios rx, ry; dense Vx, Vy are in that plane's pel units.

## 3. Overall calculation

Validate geometry, decode vectors, and check scene/temporal availability. If unavailable, copy clip. Otherwise obtain reference Super and required parity, construct small plane-specific vector grids, resample dense fields, validate every actual sample position, then copy phase samples.

## 4. Step-by-step calculation

### 4.1 Field shift, saturation, and chroma conversion

Field correction applies only for fields=true, p>1, odd d. Current top/reference bottom gives f=p/2, the reverse −p/2, otherwise zero. Parity comes from explicit tff or the two Super frames' `_Field`.

First calculate `cx=clip(vx,-32768,32767),cy=clip(vy+f,-32768,32767)`, then plane grids:

$$G_x=\lfloor c_x/r_x\rfloor,\qquad G_y=\lfloor c_y/r_y\rfloor.$$

Saturation does not waive validation of raw public vectors. Negative chroma displacements floor: at rx=2, −1 becomes −1 and +1 becomes 0. Build each plane's grid independently, rather than downsampling a dense luma field.

### 4.2 Dense fields

Apply [integer grid resampling](shared/grid-resampling.md) separately to Gx, Gy with the current plane's blocks, overlap, and real dimensions. Preserve Q14 coefficients, geometry-selected axis order, and separate rounding on both axes.

Luma grid `[0,4]` for two nonoverlapping width-4 blocks gives `[0,0,1,2,3,4,4,4]`. With rx=2, first form chroma grid `[0,2]`, then its four samples `[0,1,2,2]`.

### 4.3 Time scaling and phase selection

For plane pixel x, y:

$$a_x=px+\left\lfloor\frac{V_x(x,y)t+128}{256}\right\rfloor,\quad
a_y=py+\left\lfloor\frac{V_y(x,y)t+128}{256}\right\rfloor.$$

Split `qx=floor(ax/p),αx=ax-pqx`, likewise Y, then copy reference phase `(αx,αy)` at `(hP+qx,vP+qy)`. Time half-ties round toward positive infinity: t=128 with V=1, −1, −3 gives offsets 1, 0, −1.

Super already contains interpolated phases; Flow does not perform another pixel bilinear interpolation. time=0 selects reference integer phases, not the current frame.

## 5. A complete numerical example

Two GRAY8 frames, one `8×8` block, p=1, pad=4, d=1, zero vectors/SAD and passing scene checks. clip[0] is constant 5; Super[1] constant 80.

Both small and dense fields are zero. Output frame 0 samples matching reference integer positions and is all 80 for time=100 or 0. Frame 1 lacks a reference and falls back to clip[1]. Each output inherits its own clip properties.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,super,vectors` | Required | Output, reference phases, public vectors |
| `time` | 100.0; finite 0–100 | Dense displacement scale |
| `fields` | false | Field shift for p>1 and odd d |
| `thscd1,thscd2` | 400, 51.0 | Reference availability |
| `tff` | Omitted | Parity source |
| `prefix` | `MVUtensils` | Data names |

## 7. Boundaries, missing data, and errors

Geometry/descriptor matching follows the relevant [block-rendering rules](shared/block-rendering.md), but Flow does not validate every hypothetical public-domain sample at creation. It validates visible zero-displacement positions then, and all actual positions after dense-field generation per frame, before reading reference pixels.

Out-of-bounds positions or undefined quarter-phase edges fail. They cannot be clamped, changed to another phase, or hidden by copying clip. Normal unavailable-reference fallback requires no hypothetical parity.

fields=true, p=1 is valid and enables no shift. Property-derived parity additionally needs current Super; explicit tff can derive it from indices and usually avoids that request. time=0 still validates required parity. d=0 is valid.

## 8. Precision and determinism

Order is field correction, 16-bit saturation, chroma floor, spatial resampling, then temporal rounding. Reference float32 samples are copied bitwise without normalization/clipping. Clip properties remain intact; no dense vector field is exported.

[Back to the English index](README.md)
