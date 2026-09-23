# Degrain: from block errors to temporal weights and final pixels

## 1. What the function computes

`Degrain` establishes integer weights for the center and reference frames per block and processed plane, samples references using motion vectors, creates weighted blocks, then applies overlap composition and a change limit. Input vectors/errors are read, not re-estimated.

`Degrain1` through `Degrain25` use the same calculation. R in the name additionally requires exactly 2R vector members. Generic Degrain derives R from input count.

## 2. Objects and notation

Input order is `[a1,b1,…,aR,bR]`; each pair has opposite offsets, and absolute distances strictly increase. Either sign may come first within a pair; members are not sorted by sign. Center/reference samples are C, Rr. Final weights Vc, Vr sum exactly to 256.

Offsets `[-1,+1,+4,-4]` are valid. `[1,-1,1,-1]` repeats a distance; `[1,-2]` is asymmetric; both fail.

## 3. Overall calculation

Validate pairing, shared geometry, and availability; derive error thresholds per pair/plane; convert errors to reliability and normalize with user coefficients; sample and temporally weight blocks; combine overlap; limit changes relative to the center sample.

## 4. Step-by-step calculation

### 4.1 Near and far thresholds

thsad and thsad2 provide luma/chroma near/far thresholds, converted with [S](shared/block-rendering.md) to analysis-error scale. Every scaled value must be in `0…2147483646`, including unused planes and far values unused when R=1.

For pair j, use the near value when R=1 or j=1. Otherwise interpolate:

$$\theta_j=(j-1)\pi/(R-1),\quad a_j=(1-\cos\theta_j)/2,$$
$$T_j=\lfloor T_{near}+a_j(T_{far}-T_{near})+0.5\rfloor.$$

Operations round separately to binary64. Interpolation uses **pair index**, not actual temporal distance. Three pairs from 100 to 300 get 100, 200, 300 whether distances are 1, 4, 10 or 1, 2, 3. Far thresholds may be smaller than near thresholds.

### 4.2 Error to reliability

For an available reference with total block error s and current plane threshold T:

$$w(s,T)=\begin{cases}0&s\ge T\\
trunc\left(\frac{256(1-(s/T)^2)}{1+(s/T)^2}\right)&s<T.
\end{cases}$$

Evaluate ratio, square, numerator, denominator, and quotient in binary64 order. Unavailable references get w=0. T=0 always takes the first branch, avoiding division. Every plane uses the same stored total SAD with its own threshold; no separate chroma SAD array exists.

T=100 with s=0, 50, 100 gives w=256, 153, 0. Reliability declines with error and contributes nothing at the threshold.

### 4.3 User coefficients and normalization

User weights order is `[aR,…,a1,centre,b1,…,bR]`, unlike interleaved vector nodes. In zero-based U:

$$u_c=U[R],\quad u_{a_j}=U[R-j],\quad u_{b_j}=U[R+j].$$

First form the exact integer sum, then:

$$Z=256u_c+1+\sum_r w_ru_r,\quad g=fl64(256/Z),$$
$$V_r=trunc(fl64(fl64(w_ru_r)g)),\quad V_c=256-\sum_rV_r.$$

The +1 is intentional. Center receives the remainder after reference truncation rather than its own proportional truncation. All-zero user coefficients still give Z=1 and center weight 256. Zero center coefficient does not guarantee zero final center weight.

### 4.4 Sampling and weighting within blocks

Center uses `Super[n]`'s integer phase. Available references use raw input vectors without time scaling or field correction; see [block sampling](shared/block-rendering.md). Unavailable references mathematically use Rr=C, Vr=0 and need no image.

Integer block output is:

$$q=\left\lfloor\frac{128+V_cC+\sum_rV_rR_r}{256}\right\rfloor.$$

Weights stay constant across the block's pixels on that plane. Float32 starts from `fl32(C·Vc)`, accumulates `fl32(Rr·Vr)` in vector-member order with rounded additions, then divides by 256. No +128 bias or clipping. Zero-weight terms retain their floating arithmetic meaning, including possible signed-zero effects.

### 4.5 Limiting the composed result

First compose rounded blocks with [overlap windows](shared/block-rendering.md). Then limit resulting pixel q relative to current Super integer-phase sample C.

Convert limit to binary32. Any nonfinite value disables limiting; finite values must be positive. Integer output also disables it when limit≥maximum M. Otherwise `L=trunc(fl32(limit+0.5))`:

$$out=\min(\min(M,C+L),\max(\max(0,C-L),q)).$$

A small positive limit can round to zero and restore C. Float32 always applies finite positive limits through `lo=fl32(C-limit),hi=fl32(C+limit)`; values above 1 do not disable it. Endpoints must be finite; values already inside retain their zero sign.

## 5. A complete numerical example

One nonoverlapping GRAY8 `8×8` block, R=1, both references available with SAD=0, default thresholds, all user coefficients 1. Both reliabilities are 256, so Z=769, references each get 85, center 86.

Center 100 and references 80, 140 give:

$$q=\lfloor(128+86\cdot100+85\cdot80+85\cdot140)/256\rfloor=107.$$

Without overlap and with default infinite limit, output is 107. limit=2.2 gives L=2 and final 102. If only reference 80 remains, its weight is 127 and center 129, giving unlimited output 90.

If all references are unavailable, center weight is 256, but block composition and limit still run. Center samples come from Super; unprocessed planes and all properties come from clip, whose pixels may differ.

## 6. Parameters and their calculation steps

| Parameter | Default and constraints | Role |
| --- | --- | --- |
| `clip,super,vectors` | Required; 2–50 vector members, even count | Output, samples, paired analysis |
| `thsad` | `[400,400]`; nonnegative int64 | Near thresholds |
| `thsad2` | Effective thsad | Far thresholds |
| `planes` | Omitted/empty means all actual planes | Unprocessed planes copy clip |
| `limit` | Two positive infinities | Post-composition change limit |
| `thscd1,thscd2` | 400, 51.0 | Scene availability |
| `weights` | `2R+1` ones | User coefficients and normalization |
| `prefix` | `MVUtensils` | Data names |

One threshold/limit value duplicates to luma/chroma; empty arrays use defaults; more than two fail. U/V share the chroma value. planes accepts distinct 0, 1, 2; legal indices absent from the format are not processed. weights cannot be empty. Each coefficient saturates to int32, then must satisfy `0≤u≤floor(2147483646/(256(2R+1)))`.

### Named entry points

| Entry | Vectors | Entry | Vectors | Entry | Vectors |
| --- | --- | --- | --- | --- | --- |
| Degrain1 | 2 | Degrain10 | 20 | Degrain19 | 38 |
| Degrain2 | 4 | Degrain11 | 22 | Degrain20 | 40 |
| Degrain3 | 6 | Degrain12 | 24 | Degrain21 | 42 |
| Degrain4 | 8 | Degrain13 | 26 | Degrain22 | 44 |
| Degrain5 | 10 | Degrain14 | 28 | Degrain23 | 46 |
| Degrain6 | 12 | Degrain15 | 30 | Degrain24 | 48 |
| Degrain7 | 14 | Degrain16 | 32 | Degrain25 | 50 |
| Degrain8 | 16 | Degrain17 | 34 | | |
| Degrain9 | 18 | Degrain18 | 36 | | |

## 7. Boundaries, missing data, and errors

Validate every member even when its user coefficient is zero. Available references require valid Super; malformed data or dependency failure does not become zero weight. Member descriptors match except delta and positive Levels; later valid descriptor changes fail.

At creation, all processed planes validate sampling over the entire public vector domain. No actual processed planes does not waive parameter or required-data checks. Degrain has no fields, time, satd, or search parameters.

## 8. Precision and determinism

Reliability truncation, weight normalization, block rounding, overlap rounding, and final limiting are separate stages. Reference weights plus center remainder sum exactly to 256. Output does not depend on a previously calculated frame.

[Back to the English index](README.md)
