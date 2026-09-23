# Global motion: properties, coordinate maps, and composition

## 1. Five motion properties

`Depan_dx,Depan_dy,Depan_rot,Depan_zoom` each contain one float64 value, an exact promotion of a computed binary32 value. `Depan_goodmotion` contains one int64. dx is horizontal pixel displacement; dy is vertical displacement in aspect-normalized units; rot is degrees; zoom is multiplicative scale; nonzero goodmotion means valid.

Invalid results from DepanAnalyse and DepanEstimate use `(0,0,0,1,0)`. Consumers still read all five keys, element zero only. Missing, empty, wrongly typed, or nonfinite floating input is an error even with good=0.

Finite float64 values round to binary32, then clamp dx/dy to ±1000000, rotation to ±360000, and zoom to `[fl32(0.01),100]`. If a finite value overflows during conversion, use the corresponding clamp endpoint. Zero/negative zoom becomes 0.01 rather than missing data.

## 2. Motion to a backward sampling map

Output position x, y reads source X, Y:

$$X=t_x+ux+vy,\qquad Y=t_y+wx+hy.$$

This locates source samples rather than projecting source pixels outward. For positive aspect a, center cx, cy, time fraction f, and motion dx, dy, r, z, operations independently round to binary32 unless stated otherwise:

$$dx_f=f\,dx,\quad dy_f=f\,dy,\quad\theta=((fr)\pi)/180,\quad Z=\exp(f\log z).$$

Snap |θ|<1e−6 to +0 and |Z−1|<1e−6 to 1. The conversion itself replaces z≤0 by 1; ordinary property decoding already clamps z positive. π is binary32. Set s=sinθ, c=cosθ:

$$u=h=cZ,\quad v=((-s)/a)Z,\quad w=(sZ)a.$$

For forward=true:

$$t_x=(c_x+(((-c_x)c+(c_y/a)s)Z))+dx_f,$$
$$t_y=c_y+((((-c_y)/a)c+(-c_x)s)Z+dy_f)a.$$

For forward=false:

$$t_x=c_x+(((-c_x+dx_f)c)-(((-c_y)/a+dy_f)s))Z,$$
$$t_y=c_y+((((-c_y)/a+dy_f)c)+((-c_x+dx_f)s))Za.$$

Center terms make rotation/scale occur about the specified center. Directions compose translation and rotation differently; pure translations looking alike does not make the formulas interchangeable.

## 3. Composition and the analysis inverse

Apply A first, then B: C=B∘A.

$$t_{xC}=(t_{xB}+u_Bt_{xA})+v_Bt_{yA},\quad t_{yC}=(t_{yB}+w_Bt_{xA})+h_Bt_{yA},$$
$$u_C=u_Bu_A+v_Bw_A,\quad v_C=u_Bv_A+v_Bh_A,$$
$$w_C=w_Bu_A+h_Bw_A,\quad h_C=w_Bv_A+h_Bh_A.$$

Round products before additions; all six results are independent, without forcing h=u. Scaling by 2 then translating 3 gives 2x+3; reversing the order gives 2x+6.

DepanAnalyse inverts its fitted similarity model through a specific operation requiring h=u. Set `b=sqrt((-w)/v)` when v≠0, otherwise b=1:

$$D=u^2+((v^2)b)b,\quad u'=h'=u/D,\quad v'=((-u')v)/u,\quad w'=((-v')b)b,$$
$$t'_x=((-u')t_x)-(v't_y),\quad t'_y=((-w')t_x)-(u't_y).$$

Square-root arguments must be nonnegative and divisors nonzero. u=0 fails even if a generic affine matrix with those coefficients would be invertible.

## 4. Map to motion

Use tx, ty, u, v with u≠0:

$$\theta=-atan((av)/u),\quad r=(\theta180)/\pi,\quad s=\sin\theta,\quad c=\cos\theta,\quad z=u/c.$$

Require c≠0 and use single-ratio atan, not atan2. For forward=true:

$$dx=(t_x-c_x)-(((-c_x)c+(c_y/a)s)z),$$
$$dy=((t_y/a)-(c_y/a))-((((-c_y)/a)c+(-c_x)s)z).$$

For forward=false, additions/subtractions associate left to right:

$$dx=(t_x/z)c+((t_y/z)/a)s-(c_x/z)c+c_x-((c_y/z)/a)s,$$
$$dy=((-t_x)/z)s+((t_y/z)/a)c+(c_x/z)s-((-c_y)/a)-((c_y/z)/a)c.$$

This branch also needs z≠0. Producer output does not receive the property reader's magnitude clamps.

## 5. Fields and diagnostics

When parity is needed, explicit tff takes priority: `top(n)=bool(tff) XOR (n is odd)`. Otherwise read integer `_Field` element zero on the caller-designated frame; nonzero means top. Missing/empty/wrongly typed values fail. Do not infer parity from `_FieldBased` or a previous frame. Branches not needing parity do not read it.

With info=true, a function writes its diagnostic string and invokes the host property-text renderer. VapourSynth uses `text.FrameProps` selecting only that key, leaving other arguments at renderer defaults. Renderer output pixels/properties are final. Missing rendering support is an error, not a silent property-only result. info=false needs no renderer and preserves inherited same-named diagnostics.

## 6. Numerical rules

Geometric integers are exact before conversion; floating operations round separately without fused multiply-add. sqrt uses correctly rounded binary32. sin/cos/atan/log/exp may return the correctly rounded value or an adjacent finite value, stable for fixed inputs; later operations use that result. sin(±0), atan(±0) preserve sign, so zero rotation can display as −0.000.

[Back to the English index](../README.md)
