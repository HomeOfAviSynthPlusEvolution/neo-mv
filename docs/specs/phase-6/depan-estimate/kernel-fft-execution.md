# FFT execution and numerical boundary

Input is a real binary32 rectangle of shape (wy,wx), or a complex binary32 Hermitian half-spectrum of shape (wy,wx/2+1). Output is the corresponding half-spectrum or real rectangle. Width is even and at least 2; height is at least 2 and can be odd. The final axis is horizontal. These are logical shapes; no padded real row or private binary payload is part of the operator contract.

## Transform profile

Use the PocketFFT C++ backend with float precision. Pin its revision and consistent configuration in the implementation build. The scalar baseline disables its SIMD paths and uses one execution thread per transform. A forward real-to-complex transform has a negative exponential sign; the inverse complex-to-real transform has a positive sign. Both have scale factor 1, with no automatic normalization. Do not interchange the two axes or infer the output real width from the half-spectrum alone.

Use separate input/output storage for these real/complex transforms. Dependency strides describe bytes and must address only the admitted allocation; the plugin-facing memory rules still require positive naturally aligned sample views. Input video storage remains immutable. A transform may not consume another request's mutable scratch or overwrite a cached input spectrum. The same shape with different values must not reuse earlier results.

The [PocketFFT public interface](https://github.com/mreineck/pocketfft/blob/cpp/README.md) defines its direction, scale and half-spectrum arguments. This specification requires these numerical settings but does not prescribe a dependency-internal decomposition or work-buffer layout.

## Precision and reproducibility

The DFT in [correlation](kernel-fft-correlation.md) is the ideal mathematical target. PocketFFT returns binary32 components with its own floating-point evaluation order; it is not required to reproduce another library's bit patterns. Do not replace this float transform profile with a double transform followed by a final float conversion. The complex product between transforms has the separately specified scalar rounding boundaries. Actual FFT outputs and the correlation samples must be finite; overflow or another non-finite result is a controlled frame error.

A fixed build/profile must give deterministic results for repeated inputs regardless of frame request order. Optimized FFT profiles can introduce ordinary floating-point differences, but must retain precision type, sign, shape and normalization. Evaluate their numerical differences at the transform/correlation boundary before attributing a changed peak or goodmotion flag to rounding. Given the selected profile's actual correlation samples, all subsequent comparisons, interpolation and display quantization follow their stated rules. No universal numerical tolerance against an unspecified old dependency build is defined here, and this is not permission to change a result arbitrarily. Versioned dependency configuration belongs to the build; comparative acceptance limits belong to the implementation's measured numerical validation.

## Ownership and concurrency

Different filter instances and simultaneous requests must have independent writable data. Shared immutable transform configuration or cached spectra require safe lifetime management. Creating, destroying or unloading one instance must not invalidate work in another live instance. A cached value is keyed by all inputs that affect it, including rectangle, source frame and numerical profile.

The baseline uses host frame parallelism with single-thread transforms. Later internal parallelism must use an explicit bounded execution budget, rather than automatically selecting every processor on each call. Per-transform thread requests, actual worker-pool size and simultaneous frame requests are distinct quantities. Do not assume a per-call setting caps the entire process or controls other plugins. Avoid a full-sized worker pool per filter instance. Isolate dependency symbols/state across independently built plugins, and keep configuration consistent within a build.

The scalar backend must remain buildable without x86-only instructions. Optimized configurations must preserve a portable fallback for ARM64 and other admitted targets. Allocation/plan/execution failure is a controlled error with cleanup; no partially initialized output is published. Concurrency, teardown and platform support require actual validation of the selected build, not an inference from the dependency's name.

## Examples

- A 4x3 real rectangle has a 3x3 complex half-spectrum: nine complex elements are valid; an odd element count is not an error.
- An impulse of value 1 at (0,0) has an ideal forward spectrum of 1 at every frequency. Forward followed by the unnormalized inverse has ideal output wx*wy at that impulse, not 1.
- Two simultaneous 4x4 transforms with different inputs must not overwrite each other's spectrum or produce a result dependent on which finishes first.
