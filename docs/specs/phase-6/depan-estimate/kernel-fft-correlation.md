# Window extraction and periodic correlation

Inputs are the first-plane windows of current frame n and predecessor max(0,n-1), with identical [geometry](kernel-window-geometry.md). Call their binary32 sample rectangles A and B. Output is a real binary32 correlation surface C of shape (wy,wx) for each rectangle. Other planes and pixels outside these rectangles are not numerical inputs.

Used integer samples must be in [0,2^b-1] for the declared precision b=8..16; nonzero unused high bits that make a sample exceed this nominal range are a frame error. Copy admitted samples at their stored scale into binary32 exactly. Float32 samples retain their values and must be finite inside each used rectangle. These numeric checks apply only to used first-plane windows; untouched pixels retain their stored bits. Do not subtract a mean, divide by a pixel maximum, apply a spatial taper, alternate sample rows for fields, or shift the rectangle between the two frames. No zero padding changes the transform dimensions. Let K=wx*wy.

## Frequency-domain definition

For rectangle A, the ideal forward transform is

$$\widehat A(k,l)=\sum_{y=0}^{w_y-1}\sum_{x=0}^{w_x-1}A(x,y)\exp\left[-2\pi\mathrm{i}\left(\frac{kx}{w_x}+\frac{ly}{w_y}\right)\right].$$

Use the same transform for B. For k=0..wx/2 and l=0..wy-1, form H=conj(Ahat)*Bhat. If Ahat=ar+i ai and Bhat=br+i bi, the scalar complex product is

$$h_r=\operatorname{fl32}(\operatorname{fl32}(a_r b_r)+\operatorname{fl32}(a_i b_i)),\qquad
h_i=\operatorname{fl32}(\operatorname{fl32}(a_r b_i)-\operatorname{fl32}(a_i b_r)).$$

Apply the unnormalized inverse real transform to H. Conceptually the omitted half is given by Hermitian symmetry; do not discard non-real values on an entire k=0 or k=wx/2 column, since only the fully self-conjugate frequency positions must be real. The [FFT execution profile](kernel-fft-execution.md) supplies the actual binary32 C.

The exact mathematical target, useful for small independent examples, is

$$C(i,j)=K\sum_{y=0}^{w_y-1}\sum_{x=0}^{w_x-1}A(x,y)B((x+i)\bmod w_x,(y+j)\bmod w_y).$$

Indices are periodic within the rectangle. Positive i means a current sample is correlated with a previous sample to its right. This fixes the sign of exported dx. Swapping the two images reverses the shift. The factor K is observable through confidence's additive 0.1 term; a normalized inverse without compensation changes results.

## Examples and errors

- wx=wy=4. A is zero except A(0,0)=1; B is zero except B(1,0)=10. The ideal surface is zero except C(1,0)=160. A conjugate product in the opposite order would incorrectly place the peak at (3,0).
- Two constant 4x4 rectangles with every sample 2 produce the ideal constant surface C=1024: 16 times the sum of sixteen products of 4. Dividing pixels by the bit-depth maximum changes the confidence arithmetic even though the images look identical.
- At n=0, A and B both come from clip[0]. Correlation and required field parity are still evaluated; forcing final bad motion does not excuse an invalid used sample or failed transform.
- A NaN outside every used first-plane window is copied when untouched; a NaN inside a required window is a frame error. Chroma samples are not analyzed.
