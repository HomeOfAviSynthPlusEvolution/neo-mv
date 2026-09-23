# Bidirectional interpolation: field pairing, occlusion, and pixel composition

[FlowInter](../flow-inter.md) and [FlowFPS](../flow-fps.md) share this calculation after selecting left/right frames l, r=l+d and time coefficient t.

## 1. Pairing and selecting data

vectors must be `[bw,fw]`, with deltas +d, −d and d>0. Reversed order or zero offsets fail. Descriptors match except delta and positive Levels, match Super geometry, and cover output. Analysis and rendering precision may differ.

Main fields are `B=bw[l],F=fw[r]`. Check image indices first; out-of-range indices select the function's original-image fallback without reading vectors or Super. In range, decode both main fields even if the first is unavailable: corruption in the second still fails.

If both main fields are available and extra fields enabled, read `BB=bw[r],FF=fw[l]`, again validating both. Use extra mode only when both are available, otherwise ordinary mode. Complete malformed extra data fail.

Availability concerns data and scene decisions, not whether a field's own nominal target index exists. Image samples only come from `L=Super[l],R=Super[r]`; BB/FF do not request more distant Super frames.

## 2. Dense motion and occlusion grids

Motion uses [Flow](../flow.md)'s saturation/chroma conversion and [integer resampling](grid-resampling.md), with field correction fixed to zero. Spatial resampling precedes time scaling.

Occlusion grids use [OcclusionMask events](../occlusion-mask.md) with fixed inputs:

| Grid | Raw vectors | Direction | Time | Maximum/gamma |
| --- | --- | --- | --- | --- |
| GB | B, before saturation/chroma conversion | +d | 256−t | 255, 1 |
| GF | F, before saturation/chroma conversion | −d | t | 255, 1 |

Event scale uses luma block steps, pel, and binary32 ml. Quantize to a 0–255 block grid, then resample per plane to mB, mF. Chroma changes spatial expansion only, not convergence strength. Integer16/float images still use maximum 255, not sample maximum or 256. BB/FF create no events.

## 3. Selecting image samples

For plane-pixel pel coordinates q=(px, py), define componentwise `D(v,t)=floor(vt/256)`, without Flow's +128 bias.

Main samples are `A=L(q+D(F,t)),C=R(q+D(B,256-t))`: F samples the left image, B the right.

Ordinary mode also takes unmoved `A0=L(q),C0=R(q)`. Extra mode instead takes `E=L(q+D(FF,t)),K=R(q+D(BB,256-t))`. Coordinates already use plane units; do not divide again for chroma. Split into integer position and phase and read Super directly.

Before reading pixels, validate every position required by the selected mode, including zero-coefficient terms. No-contribution samples cannot excuse undefined phase edges, and failed sampling cannot dynamically downgrade to ordinary mode.

## 4. Ordinary nested composition

For integers define `H(z)=floor((z+256)/256),J(z)=floor(z/256)`:

$$U=H\left(A(256-m_F)+H\left(m_F(C(256-m_B)+m_BA_0)\right)\right),$$
$$V=H\left(C(256-m_B)+H\left(m_B(A(256-m_F)+m_FC_0)\right)\right),$$
$$out=J(U(256-t)+Vt)-1.$$

In U, A is the left main sample. Increasing mF introduces a replacement formed from right main C and left unmoved A0, controlled by mB. V is the corresponding other-side expression. Time finally combines U, V.

Every H rounds separately with bias 256, not conventional 128. Retain H(0)=1 and the final −1. Intermediates can exceed sample maximum and must not be prematurely narrowed/clipped.

## 5. Extra mode

Let `lo=min(A,C),hi=max(A,C)`. Clamp K, E into this interval to obtain CK, CE:

$$U=H(C_Km_F+A(256-m_F)),\quad V=H(C_Em_B+C(256-m_B)),$$
$$out=J(U(256-t)+Vt)-1.$$

This is not a direct four-image average. Extra samples first take the main sample range, then replace occluded portions. m=255 still leaves main coefficient 256−m=1.

Float32 keeps the same grouping/order, replaces H/J by binary32 division by 256, and omits final −1. Products/additions round separately; no [0, 1] clipping. Equal min/max retains the first operand, including signed zero. Required samples/intermediates must be finite.

## 6. Numerical examples and fallback blending

With mF=mB=0, A=10, C=21, t=128, integers give U=11, V=22, output 15; float gives 15.5. Nearest-integer averaging would incorrectly give 16.

Extra mode A=10, C=30, E=50, K=0, mF=255, mB=0 first gives CK=10, CE=30, then U=11, V=31. At t=128 output is 20.

If the main pair is unavailable and blending enabled, use original clip samples:

$$out=\lfloor(C_0(256-t)+C_1t)/256\rfloor.$$

Float uses separately rounded binary32 multiplication, addition, division without bias. Even identical frames do not universally make this a bitwise copy; only explicit copy branches can skip arithmetic.

[Back to the English index](../README.md)
