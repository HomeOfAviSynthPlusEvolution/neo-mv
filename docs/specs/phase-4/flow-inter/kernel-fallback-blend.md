# Ordinary two-image fallback blend

Inputs are two visible render images C0,C1 with matching format/dimensions and integer t in [0,256]. Output has that same format/dimensions. Each image and the output may have a different valid positive stride. No Super phases, motion fields or occlusion masks are involved. The caller selects frames and output properties.

For each integer sample in [0,M], use exact wide arithmetic:

$$out=\left\lfloor\frac{C_0(256-t)+C_1t}{256}\right\rfloor.$$

There is no rounding bias. For finite float32 samples use

$$a=\operatorname{fl32}(C_0\operatorname{fl32}(256-t)),\quad b=\operatorname{fl32}(C_1\operatorname{fl32}(t)),\quad out=\operatorname{fl32}(\operatorname{fl32}(a+b)/256).$$

Require finite used samples and every intermediate, including zero-weight operands. Do not turn equal input values or repeated frame identities into a sample-copy shortcut unless the result and error behavior are preserved. Pure endpoint-copy branches of FlowFPS are separately specified and do not call this kernel.

Examples: C0=10,C1=21,t=128 produces integer 15 or float 15.5. Integer C0=C1=255 produces 255. With float C0=C1=1.75+2^-23 and t=85, the expression produces 1.75+2^-22; replacing this blend with a direct copy would change the output. Any invalid used view or non-finite required arithmetic is an error rather than another fallback selection.
