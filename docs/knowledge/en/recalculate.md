# Recalculate: map old vectors to a new grid and measure again

## 1. What the function computes

`Recalculate` takes an array of old analysis nodes, maps each member's vectors onto a new block grid, measures fresh errors on the new Super samples, and searches further when needed. Output member count and order match the input array.

Old errors are validated but do not decide whether to search. Even unchanged vectors receive newly measured errors. Visible output and initial properties come from `super`, not old vector nodes.

### Mixed metric weights

The two local modes use the same per-candidate trigger and Q16 mixture as [Analyse](analyse.md). Parameters are independent of input vectors and default to 0.5 and 0.03125. The three global modes use fixed base_weight=8 in Recalculate (effective weight 4 for half), without coarsest-level statistics. Remeasure the mapped candidate using this policy before comparing its new raw error with thsad; the old stored error does not enter the mixture.

## 2. Objects and notation

Old geometry has subscript 0: blocks `Bx0,By0`, steps `sx0,sy0`, and pel `p0`. Target geometry is `Bx,By,Ox,Oy,p`, with block index `bx,by`. d is the old `DeltaFrame` saved when the member was created.

Each calculation uses `vectors[n],super[n],super[k]`, where `k=clamp(n+d,0,Nsuper-1)`. Outside the sequence it compares with the first/last frame, unlike Analyse's missing-reference branch.

## 3. Overall calculation

Build the target grid and sampleable candidate domain; decode old vectors; choose or interpolate old vectors at each target block center; convert pel and clip to the target domain; measure new error; search only above threshold; export the new grid and errors.

## 4. Step-by-step calculation

### 4.1 Target geometry

The target uses [Analyse](analyse.md)'s finest-level ceiling coverage formula. For block origin `x=bx(Bx-Ox),y=by(By-Oy)`, candidates satisfy:

$$-p(x+h_p)\le v_x<p(W+h_p-x-B_x),$$
$$-p(y+v_p)\le v_y<p(H+v_p-y-B_y).$$

The entire integer domain must be sampleable, even if low error will avoid searching. There is no special zero seed outside Ω here.

### 4.2 Locating the target center in the old grid

The horizontal center and old-grid index are:

$$c_x=\lfloor B_x/2\rfloor+(B_x-O_x)b_x,$$
$$j_x=\operatorname{trunc}\frac{c_x-\lfloor B_{x0}/2\rfloor}{s_{x0}},\quad
d_x=\max(0,c_x-(\lfloor B_{x0}/2\rfloor+s_{x0}j_x)).$$

The vertical axis is analogous. Read vectors A, B, C, D at `(jx,jy),(jx+1,jy),(jx,jy+1),(jx+1,jy+1)`, clipping each lookup index independently to the old grid. Compute dx, dy before clipping lookup indices.

Mapping uses the declared block centers. Different old/new image dimensions do not imply image-scale conversion. Even odd externally supplied old block dimensions use floor for the half-block center.

### 4.3 Nearest choice and smooth interpolation

With `smooth=false`, choose left if `2dx<sx0`, otherwise right; choose top if `2dy<sy0`, otherwise bottom. Exact midpoint ties select right/bottom.

With `smooth=true`, compute each vector component independently:

$$u=A s_{x0}+d_x(B-A),\quad v=C s_{x0}+d_x(D-C),$$
$$I=\operatorname{trunc}\left(\frac{u+\operatorname{trunc}(d_y(v-u)/s_{y0})}{s_{x0}}\right).$$

This forms horizontal weighted quantities on both rows, then interpolates vertically, preserving both truncations toward zero. Convert the chosen/interpolated component z to `floor(z·p/p0)`, then clamp it to Ω's inclusive endpoints. For `z=-1,p0=2,p=1`, the result is −1, not zero.

### 4.4 Threshold, new measurement, and search

The new error uses the [SAD, SATD, DCT, and mixed error calculation in Analyse](analyse.md#45-measuring-raw-error). `metric` defaults independently to `"sad"`; it does not inherit the metric used to produce the input vectors. To measure DCT again, explicitly pass `metric="dct"`. DCT requires integer samples and supports every valid target block shape. Threshold scaling below remains unchanged; there is no metric-to-metric conversion.

Using [Analyse](analyse.md)'s depth conversion Qb:

$$\lambda_0=Q_b(\operatorname{trunc}(mvlambda\,B_xB_y/64)),\quad
T=\operatorname{trunc}(Q_b(thsad)\,B_xB_y/64).$$

With chroma, set `T←T+2·trunc(T/(rx·ry))`: divide and truncate before multiplying by 2. The first block row has λ=0; later rows use `λ=floor(λ0/p²)`.

Measure `s(u)` at mapped position u. If `s(u)≤T`, output u and this fresh error. Otherwise search from u with initial cost `s(u)`, fixed predictor u, `q=pnew`, and range `max(1,searchparam)`. Candidate generation and costs follow [Analyse](analyse.md).

There is no LSAD adaptation, level multiplier, global predictor, multiple-start search, or bad-block expansion. A negative threshold makes every nonnegative error enter search, but movement still requires strict improvement.

## 5. A complete numerical example

Old blocks are nonoverlapping `8×8`; the first target block is `16×8`. Its horizontal center is 8 versus old center 4, so `jx=0,dx=4,sx0=8`. If old left/right X vectors are 0, 4 and both rows match, nearest selection chooses 4 at the midpoint; smooth interpolation gives 2.

Let both pel values be 1, vector 2 be inside the target domain, and the new GRAY8 error be 100. `thsad=50` and area 128 give T=100. Equality passes, so no search follows; output vector 2, error 100. An old error of 999 changes nothing. Output `Levels` is 1.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `super,vectors` | Required; nonempty vector-node array | New samples and old grids |
| `thsad` | 200; signed int32 | Search threshold for new error |
| `smooth` | true | Nearest choice or interpolation |
| `blksize,overlap` | Inherited from Super | Target grid; array rules as in Analyse |
| `search,searchparam` | 2, 2; search 0–5, range at least 1 | Single-level search |
| `mvlambda,pnew` | 1000, 25; nonnegative and 0–256 respectively | Distance and error penalties |
| `chroma` | true | Include chroma SAD |
| `metric` | Default `"sad"`; SATD family requires dimensions divisible by 4; DCT and mixed modes require integers | Luma error policy; chroma stays SAD |
| `metric_weight` | Local only; default 0.5, [0,1] | Transform weight after triggering; Q16 quantization |
| `metric_threshold` | Local only; default 1/32, [0,1] | Relative luma-sum change threshold; Q16 quantization |
| `meander` | true | Evaluation direction; independent block results do not change |
| `fields,tff` | false, omitted | Field-information validation |
| `prefix` | `MVUtensils` | Super/Analysis names |

## 7. Boundaries, missing data, and errors

Old precision must match new Super precision; old blocks, grids, pel, and real dimensions may differ. Valid metadata with absent/wrong-count arrays uses zero old vectors for mapping, measurement, and search. Complete malformed arrays fail; see [analysis data](shared/analysis-data.md).

Each member's d is fixed at creation. Later property changes cannot alter the reference or exported delta. Changes violating established geometry or precision fail. Any member creation error fails the whole call.

`fields=true` requires old `AnalysisPel>1` and valid current parity. With target pel>1 and odd d, also validate the clamped reference frame's parity. This validation does not add a half-pixel shift to mapped vectors.

## 8. Precision and determinism

Interpolation truncation and pel-conversion floor are separate stages. Old errors do not participate in interpolation or area scaling. Each target block depends only on old data and new images, so meander does not change spatial predictors as it can in Analyse. Request order does not change results.

[Back to the English index](README.md)
