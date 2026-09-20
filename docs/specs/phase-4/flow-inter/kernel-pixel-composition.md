# Occlusion-aware pixel composition

Inputs are samples A,C and either A0,C0 for basic mode or E,K for extra mode from [bidirectional sampling](kernel-bidirectional-sampling.md), integer masks mF,mB in [0,255], and integer t in [0,256]. Output is one sample of the render type. This kernel neither chooses modes nor accesses images.

## Integer render samples

For exact nonnegative integers define H(z)=floor((z+256)/256) and J(z)=floor(z/256). The bias in H is 256, not 128. Basic mode is

$$U=H\left(A(256-m_F)+H\left(m_F\left(C(256-m_B)+m_BA_0\right)\right)\right),$$
$$V=H\left(C(256-m_B)+H\left(m_B\left(A(256-m_F)+m_FC_0\right)\right)\right),$$
$$out=J\left(U(256-t)+Vt\right)-1.$$

Every H/J operation rounds separately; do not simplify away an inner H, including H(0)=1. Intermediate values may exceed the sample range and must not be narrowed to storage prematurely. For valid input samples the final result is in [0,2^renderDepth-1]; there is no additional saturation or modulo conversion.

In extra mode let lo=min(A,C),hi=max(A,C), CK=max(lo,min(K,hi)), CE=max(lo,min(E,hi)). Then

$$U=H(C_Km_F+A(256-m_F)),\quad V=H(C_Em_B+C(256-m_B)),$$
$$out=J(U(256-t)+Vt)-1.$$

The opposite-side extra sample is clamped to the interval of the two main samples before weighting. This is not a four-sample average.

## Float32 render samples

Use the same parenthesized equations and min/max choices with these substitutions: H(z)=fl32(z/256), J(z)=fl32(z/256), and omit the final subtraction of 1. Each multiplication and addition in the displayed formulas is individually rounded to binary32. Integer masks and coefficients convert exactly to binary32 before the operation. Evaluate the written nesting and operand order without reassociation or contraction. Min/max compare finite values; for equal values retain the first operand, including signed-zero ties. Every required intermediate must be finite. Do not clamp output to [0,1].

## Examples

- mF=mB=0,A=10,C=21,t=128 gives integer U=11,V=22,out=15. Float output is 15.5. Integer round-to-nearest would incorrectly give 16.
- Basic mode, A=10,C=30,A0=100,C0=200,mF=255,mB=0,t=128: inner H(255*7680)=7651, U=30,V=31,out=29. A mask value 255 must not be interpreted as exact weight 1.
- Extra mode, A=10,C=30,E=50,K=0,mF=255,mB=0,t=128: CK=10,CE=30. Integer U=11,V=31,out=20; float U=10,V=30,out=20.
- With every input sample equal to integer M, output remains M for all legal masks/times; with every sample zero, output is zero. Float interpolation of equal finite values still follows the arithmetic expression and need not preserve the exact input representation.
