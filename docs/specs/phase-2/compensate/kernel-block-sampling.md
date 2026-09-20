# Render block sampling from Super phases

Inputs: a logical level-zero Super image, render block origin x,y in unpadded luma pixels, luma block dimensions Bx,By, displacement dx,dy in luma pel units, factor p, and a processed plane's render ratios rx,ry. Output: the block's Bx/rx by By/ry samples in the same sample type. Y uses ratios 1,1. The caller selects the image and displacement; this kernel does not select references or measure errors.

## Coordinate map

Require x,Bx divisible by rx and y,By divisible by ry. Form the total unpadded plane coordinates in plane-pel units:

$$a_x=\left\lfloor\frac{px+d_x}{r_x}\right\rfloor,\qquad a_y=\left\lfloor\frac{py+d_y}{r_y}\right\rfloor.$$

Split with floor division and nonnegative remainder:

$$q_x=\lfloor a_x/p\rfloor,\quad \alpha_x=a_x-pq_x,\qquad
q_y=\lfloor a_y/p\rfloor,\quad \alpha_y=a_y-pq_y.$$

For the plane's actual Super padding hP,vP, output sample (i,j) is phase (alphaX,alphaY) at

$$P_{\alpha_x,\alpha_y}(h_P+q_x+i,\ v_P+q_y+j).$$

The complete rectangle, including portions that will be cropped from visible output, must belong to the phase's defined logical domain. Read the phase sample exactly; do not interpolate again. This total-coordinate floor rule is different from truncating a chroma motion component in the Phase 1 block-error kernel. Do not reuse that coordinate map here.

## Whole-domain admission

For a block define the public displacement rectangle V by

$$-p(x+hp)\le v_x<p(W+hp-x-B_x),\qquad
-p(y+vp)\le v_y<p(H+vp-y-B_y),$$

where W,H,hp,vp are luma Super working dimensions and padding. V is unchanged from the corresponding [public vector bounds](../../phase-1/analyse/data-format.md). It must be nonempty.

At creation require the full block rectangle to be sampleable for every processed plane under these checks:

- Degrain checks (dx,dy)=(vx,vy) for every integer vector in V, and also requires the centre (0,0).
- Compensate checks (trunc(vx*t/256),trunc(vy*t/256)) for every integer vector in V, with the configured time coefficient t and no field shift. Additionally check the current-block position (0,f) for every f in the set defined by its compensation kernel. The actual field-shifted reference footprint has a separate frame-time check below.

Use each plane's own Super padding and domain, including the restricted edges of built-in quarter phases. These creation checks quantify over geometry, not pixel values; a bounds proof may replace enumeration. Failure is a controlled creation error, even if the offending field would be unavailable or a weight/threshold would avoid its use. Do not shrink V, clamp vectors to a safer subset, omit in-domain vectors, add edge samples or increase padding. Time scaling is part of the Compensate map, not an extra search eligibility rule.

For an available Compensate field, obtain the actual f from the current/reference frame parities. Before generating any blocks, require every supplied vector's footprint at (trunc(vx*t/256),trunc(vy*t/256)+f) to be sampleable in all processed planes. Check this even when that block's SAD would select the current image. A failed check is a controlled frame error, not a skipped vector or current-image fallback. It is not a requirement that every hypothetical vector in V be safe with every nonzero f: a boundary vector may become unsafe after that additional displacement. Degrain has no field shift and needs no such extra displacement check.

Later Super frames must preserve the validated logical geometry and phase domains. Validate their storage views before reading. A direct call with an unsafe footprint is a controlled error with no successful block output. Public vector-array validation remains required independently; a valid packed vector alone is not evidence that this render sampling map is safe.

## Storage and examples

Output cannot alias input phases; no sample reads from row gaps or undefined phase locations are permitted. Each input plane and each output may have a different positive stride. Kernel operation has no host frame or global-state side effects.

- p=2, x=0, dx=-1, rx=2 gives ax=-1, qx=-1, alphaX=1: half a chroma pixel left. With padding hP=2, the first read is phase 1 at u=1.
- p=2, x=8, dx=-3, rx=2 gives ax=floor(13/2)=6, qx=3, alphaX=0. With hP=2, reading begins at u=5.
- YUV420 16x16, block 8x8, no overlap, pad=3, p=2: the top-left V contains vx=-6. Degrain maps this to chroma qx=-2 with hP=1, giving u=-1; creation fails when chroma is processed. For pad=4, every vector in all four blocks is sampleable in all planes. Constant-7 phases produce all-7 blocks.
- For the same pad=3 geometry, Compensate with time=0 and fields=false uses (0,0) for every vector. Its sampling precondition passes; incoming arrays must still satisfy public validation. This does not turn an unavailable reference into an available one.
- With pad=4,p=2,time=100,fields=true and odd d, Compensate passes creation for the 16x16 geometry above. An actual zero vector with f=1 is sampleable. At the top row an actual vy=-8 with f=-1 instead gives luma qy=floor(-9/2)=-5 and padded start -1: this requested frame fails before generating blocks, even if its SAD would select the current block.
- A built-in phase with alphaX=3 at p=4 excludes its last column. A four-sample read [8,12) in a plane of width 12 is invalid, whereas [7,11) is within its horizontal domain. Cropping the final output does not authorize the invalid read.
