# Block vectors to dense plane displacement

Inputs are a Complete scene-eligible public vector field, pel p, saved displacement d, fields, optional tff, applicable current/reference parity, and render-plane ratios rx,ry. Output is a pair of signed integer displacement fields at every visible sample of that plane. Y uses 1,1. These components remain in plane-pel units. The public vector bounds must already have passed validation; saturation below is a numeric transformation, not a way to accept malformed public data.

## Field correction

The correction is active exactly when fields=true, p>1 and d is odd. Otherwise f=0 and parity metadata is ignored, including fields=true with p=1. If active and tff is supplied, top(k)=bool(tff) XOR (k is odd). If active and tff is omitted, read element zero of integer `_Field` on both Super[n] and Super[n+d]; nonzero means top. Missing, empty or wrongly typed parity is a frame error; later elements are ignored. Define

$$f=\begin{cases}p/2&top(n)\land\neg top(n+d),\\-p/2&\neg top(n)\land top(n+d),\\0&\text{otherwise}.\end{cases}$$

Validate required parity even when time=0. There is no parity read for a whole-frame unavailable-reference fallback. This correction applies before displacement saturation, chroma conversion, spatial resampling and time scaling.

## Small plane grid

For each vector form exact integer values

$$c_x=\min(32767,\max(-32768,v_x)),\qquad c_y=\min(32767,\max(-32768,v_y+f)),$$
$$G_x(b_x,b_y)=\lfloor c_x/r_x\rfloor,\qquad G_y(b_x,b_y)=\lfloor c_y/r_y\rfloor.$$

The addition vy+f must not overflow before clipping. Use render-plane ratios, regardless of AnalysisChroma. Require Bx,Ox divisible by rx and By,Oy divisible by ry. This creates a grid of signed plane displacements with the same Nx,Ny for every plane. Do not first spatially resample the luma field and then downsample it for chroma.

Invoke [integer grid resampling](../vector-length-mask/kernel-grid-resampling.md) independently on Gx and Gy, using Bx/rx,By/ry,Ox/rx,Oy/ry and visible Wr/rx,Hr/ry. Its result is Vx(x,y),Vy(x,y), rounded only once per component with halfway results toward positive infinity. The same field can serve U and V when their geometry agrees. Writable temporary grids must not mutate input vectors or shared state.

## Examples

- With f=0 and rx=2, vx=-1 becomes -1; vx=1 becomes 0. Truncation toward zero is incorrect for the negative case.
- vx=40000 saturates to 32767, then rx=2 gives 16383. This kernel example presumes a sufficiently large public vector bound; it does not waive that bound. vy=32767,f=2,ry=1 remains 32767.
- p=2,d=1,fields=true,current top,reference bottom gives f=1. A constant (0,0) field becomes dense Y (0,1); at ry=2 it becomes dense chroma (0,0). With current bottom and reference top, f=-1 and dense chroma vertical displacement is -1.
- A constant grid stays constant. For a horizontal small grid [0,4], Nx=2,Ny=1,Bx=By=4,overlap=0,Wr=8,Hr=4, every dense Y row is [0,0,1,2,3,4,4,4]. At rx=2,ry=1 the small horizontal grid is [0,2] and every four-column chroma row is [0,1,2,2].
- fields=true,p=1 requires no `_Field` and applies no correction. fields=true,p=2,odd d,omitted tff and a missing required `_Field` is a frame error even at time=0.
