# Phase 1: motion analysis

Implement in the following dependency order. Within each directory, implement the kernels before composing the plugin.

| Function | Operators and data | Plugin interface |
| --- | --- | --- |
| Super | [Geometry](super/kernel-geometry.md), [border extension](super/kernel-border-extension.md), [pyramid reduction](super/kernel-pyramid-reduction.md), [subpixel interpolation](super/kernel-subpixel-interpolation.md) | [Super](super/plugin.md) |
| Analyse | [Block error](analyse/kernel-block-error.md), [search](analyse/kernel-motion-search.md), [prediction](analyse/kernel-vector-prediction.md), [analysis composition](analyse/kernel-analysis.md), [data format](analyse/data-format.md), [validation](analyse/kernel-vector-validation.md) | [Analyse](analyse/plugin.md) |
| AnalyseMany | Reuses Analyse without another numerical kernel | [AnalyseMany](analyse-many/plugin.md) |
| Recalculate | [Vector refinement](recalculate/kernel-vector-refinement.md); reuses block error and search | [Recalculate](recalculate/plugin.md) |
| SCDetection | [Scene classification](sc-detection/kernel-scene-classification.md); reuses vector validation | [SCDetection](sc-detection/plugin.md) |

[External vector scaling](analyse/vector-scaling.md) defines the typed-data transformation that public analysis readers must accept.

## Common numerical and memory contracts

`floor` rounds toward negative infinity; `trunc` rounds toward zero. All integer sums, products, sizes and coordinate calculations must be representable before use. Use sufficiently wide intermediates; do not rely on signed overflow or unsigned wraparound except the explicit vector bit packing.

For b-bit integer samples, M=2^b-1 and

$$R_k(z)=\left\lfloor\frac{z+2^{k-1}}{2^k}\right\rfloor,\qquad K(z)=\min(M,\max(0,z)).$$

Define A(a,b)=R1(a+b) and D(a,b,c,d)=R2(((a+b)+c)+d). For float32 samples, replace Rk with division by 2^k without a bias, and K with the identity. Each floating operation rounds to nearest-even in the specified precision; do not fuse multiply-add in the scalar baseline. Unless specified otherwise, sample arithmetic uses binary32 and parameter/weight arithmetic uses binary64. Floating reassociation is not assumed exact.

A plane has a sample type T, width w, height h, row stride s in bytes, and a readable or writable logical region. Integer samples use unsigned 8-bit storage for depth 8 and unsigned 16-bit storage for depths 9-16; float samples use binary32 storage. Every input, output and scratch plane view must satisfy:

- Every row's first sample address meets the natural alignment requirement alignof(T). No SIMD alignment is required.
- s is strictly positive, divisible by sizeof(T), and at least w*sizeof(T). Zero and negative strides are unsupported, including for a single-row view.
- Each row contains w valid T samples within the declared storage lifetime and accessible region. Address calculations and the full accessed extent are representable. Row gaps need not contain accessible samples.

Reject a view that violates these conditions with a controlled error before accessing its samples or converting its byte stride to an element stride. No fallback for naturally misaligned samples, fractional element strides or negative strides is required. This is a view-admission rule, not permission to reinterpret invalid storage as typed objects. Kernels must accept otherwise valid rows that are not SIMD-aligned and strides larger than the logical row size.

For example, with sizeof(T)=alignof(T)=2, a valid allocated uint16 plane of width 8 and height 2 may begin at address 0x1002 with stride 18 bytes. Its row starts 0x1002 and 0x1014 are naturally aligned but neither is 16-byte aligned; the element stride is 9. Given the same sample values it must produce the same result as tightly packed aligned storage. A start address of 0x1001 is unsupported. Separately, strides 17, 0 and -18 are unsupported even with a naturally aligned start address. These cases produce an error, not a numerical result.

No kernel may read row gaps, uninitialized samples or another phase implicitly. Inputs are immutable; output and scratch may not overlap inputs unless a kernel expressly allows it. Independent calls have independent writable output/scratch. Kernels do not allocate host frames, parse host properties or own worker pools. Zero-sized work is rejected unless explicitly allowed.

Sample families are GRAY and YUV420/422/440/444, integer 8-16 bits or float32, subject to host support. Unsupported host formats fail explicitly. Floating numerical definitions cover finite inputs and finite intermediate results. Integer equations, property transport and copied samples are exact; the floating equations define the scalar numerical baseline.

Plugin integer arguments marked int32 use sat32(x)=min(2147483647,max(-2147483648,x)) before validation. Boolean arguments use zero/nonzero truth semantics. Missing optional arguments select their documented defaults; malformed argument types are errors. A two-axis argument has one value replicated or two values in horizontal/vertical order; more than two is an error. Empty-array handling is specified at the relevant plugin.

Errors detected from arguments or frame-0 metadata occur at creation. Errors dependent on an actual requested frame occur at frame evaluation. Report allocation failure, unsupported geometry, unrepresentable arithmetic, invalid samples and missing required data as controlled errors. Never return a partly initialized successful frame. Unless a plugin specifies an ordering, no exact wording or precedence between simultaneous independent errors is required.
