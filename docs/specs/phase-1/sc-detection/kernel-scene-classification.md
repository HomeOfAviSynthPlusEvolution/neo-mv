# Scene classification from block errors

Inputs: an analysis descriptor Bx,By,Nx,Ny,chroma flag C,ratios rx,ry,precision b; user thscd1 and thscd2; and current [validated field](../analyse/kernel-vector-validation.md). Output: scaled thresholds T1,T2 and integer change in {0,1}. No image samples or motion re-estimation are used.

thscd1 is an int64 in [0,16320]. Convert thscd2 to binary32 nearest-even before requiring it to be finite and in [0,100]. Metadata must have a positive valid grid. Geometric products, thresholds and counts must be representable. All binary64/binary32 operations below round independently to nearest-even; no implicit FMA.

## Thresholds

Using binary64 arithmetic, define

$$A=(B_xB_y)/64,\quad C_f=\begin{cases}1+2/(r_xr_y)&C\ne0\\1&C=0,\end{cases}\quad D=(2^{\min(16,b)}-1)/255,$$
$$T_1=\left\lfloor thscd1\,((A C_f)D)+0.5\right\rfloor.$$

Bx*By is first an exact integer product; divisions shown here are real floating divisions, not integer truncations. T1 is int64. For float analysis b=32, D=257. Do not use the normalized float sample maximum 1.

Let t2 be thscd2 converted to binary32. Promote it to binary64; compute ((t2*Nx)*Ny)/100 in that order, then round the result to binary32 T2. Do not round to a whole block count. Both thresholds are functions of this descriptor and the user arguments; a caller may cache them without changing their values.

## Decision

InvalidMetadata gives change=1 without reading error entries. Otherwise require the current field's Bx,By,Nx,Ny,boolean chroma flag,rx,ry,b to equal the descriptor; a mismatch is an error, including for MetadataOnly fields. A matching MetadataOnly field gives change=1. Malformed complete data is an error, not a scene change. For a Complete field, N=Nx*Ny and

$$K=\sum_{i=0}^{N-1}\mathbf1_{SAD_i>T_1},\qquad
change=\mathbf1_{\operatorname{binary32}(K)>T_2}.$$

Both comparisons are strict. SAD equal to T1 is not bad; a bad-block count equal to T2 does not indicate a change. The entry count and thresholds refer to the same descriptor. thscd2=100 does not override the missing-field result.

Inputs are immutable; no array output or shared writable state is required. A scalar threshold cache belongs to the caller, not to a global kernel object.

## Examples

- 8-bit block 8x8, no chroma, grid 2x2,thscd1=400,thscd2=50 gives T1=400,T2=2. SAD [400,401,900,0] gives K=2,change=0. [401,401,900,0] gives K=3,change=1.
- thscd2=51 on that grid gives binary32(2.04), not 51/255 or a rounded count of 2.
- 10-bit block 16x8,YUV420,chroma enabled,thscd1=400 gives T1=4814. SAD 4814 does not count; 4815 does.
- Float analysis, block 8x8, no chroma,thscd1=400 gives T1=102800.
- thscd2=0 changes on any positive bad-block count; thscd2=100 never changes for a complete fixed-grid field, but unavailable fields still give 1.
