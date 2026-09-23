# FlowBlur: average two motion trajectories within the current image

## 1. What the function computes

`FlowBlur` expands two directional block fields into dense trajectories, samples along them in the current Super image, and averages. Every pixel sample comes from that one image, not multiple temporal images.

## 2. Objects and notation

Pair `[bw,fw]` has d>0. Output n uses `B=bw[n-d],F=fw[n+d]`, not both at n. Convert blur to binary32, then `t=trunc(fl32(fl32(blur·256)/200))`. prec is positive effective int32.

## 3. Overall calculation

Check both field indices; out of range copies clip. Otherwise validate both; either unavailable also copies clip. Available fields require only Super[n]. Generate dense fields, trajectories, and the full sample list; validate all positions before averaging in fixed order.

## 4. Step-by-step calculation

Use [Flow dense fields](flow.md) with field shift zero. There is no occlusion mask or ml.

For center pel coordinate q=(px, py), compute separately for dense F and B vectors G:

$$z_G=tG,\quad a_G=\max(|z_{Gx}|,|z_{Gy}|),\quad
m_G=\left\lfloor\frac{trunc(a_G/prec)}{256}\right\rfloor.$$

m=0 adds no samples. Otherwise first take integer step `eG=trunc(zG/mG)`, then for i=1…m:

$$q_{G,i}=q+(\lfloor i e_{Gx}/256\rfloor,\lfloor i e_{Gy}/256\rfloor).$$

Order is center, increasing F indices, increasing B indices. Repeated coordinates count repeatedly. Truncate the step before multiplication by i; do not directly subdivide as i·z/m.

Split coordinates into phase/integer positions and sample Super[n]. Total c=1 copies center bits exactly. For c>1, integer output is floor(sum/c), without bias. Float uses binary64 accumulation in center/F/B order, rounding every addition, then division by c and conversion to binary32.

## 5. A complete numerical example

Interior n, p=1, blur=200, prec=1, dense F=(2, 0), B=(−2, 0). t=256, m=2 in each direction, giving offsets `[0,1,2,-1,-2]`.

Current Super samples `[10,20,30,0,1]` sum to 61. Integer output is floor(61/5)=12; floating output is binary32 12.2. Properties come from clip[n].

blur=50 instead gives t=64 and m=0 both ways, copying Super's center, not clip unless the field/boundary fallback applies.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,super,vectors` | Required; ordered signed pair | Output, current image, trajectories |
| `blur` | 50.0; finite binary32 0–200 | Trajectory length |
| `prec` | 1; saturate int64 to int32, require ≥1 | Sampling density; larger values can eliminate extra points |
| `thscd1,thscd2` | 400, 51.0 | Scene decision |
| `prefix` | `MVUtensils` | Data names |

## 7. Boundaries, missing data, and errors

Boundary fallback reads no fields or Super. In range, both fields are validated; a missing field cannot hide the other's negative SAD. blur=0 still validates fields and required Super. Every trajectory point must have defined phase support; no shortened trajectories or skipped illegal points.

Each direction adds at most 32768 samples, total at most 65537. Integer16 sums cannot exceed 4294967295 and can be accumulated exactly in wide integers.

## 8. Precision and determinism

Negative step division truncates, final coordinate division by 256 floors. Floating order matters: binary64 sum `[2^60,1,-2^60]` is zero, while another order can give 1. Center-only output copies bits rather than using division that could change zero signs.

[Back to the English index](README.md)
