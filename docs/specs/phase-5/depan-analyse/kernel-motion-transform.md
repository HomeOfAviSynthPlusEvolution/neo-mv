# Motion and sampling transforms

Input is a motion tuple, finite positive aspect a, finite center `(cx,cy)`, a finite time fraction f and a direction flag forward. Output is six binary32 coefficients of a backward sampling map:

$$X=t_x+u x+v y,\qquad Y=t_y+w x+h y.$$

This tells an output pixel where to sample the source. It does not scatter source pixels. The [common numeric contract](../README.md#common-contracts) applies. The motion validity flag is handled by the caller, not by these equations. Let pi=fl32(pi), the nearest binary32 value of mathematical pi.

## Motion to coordinates

Set `dx_f=f*dx`, `dy_f=f*dy`, theta=((f*r)*pi)/180. If abs(theta)<fl32(0.000001), replace theta by +0. For this operator alone, z<=0 is replaced by 1 before log; the property decoder normally already clamps it positive. Set `Z=exp(f*log(z))`; if abs(Z-1)<fl32(0.000001), replace Z by 1. Let s=sin(theta), c=cos(theta), and

$$u=cZ,\quad h=cZ,\quad v=((-s)/a)Z,\quad w=(sZ)a.$$

For forward=true:

$$t_x=(c_x+(((-c_x)c+(c_y/a)s)Z))+dx_f,$$
$$t_y=c_y+((((-c_y)/a)c+(-c_x)s)Z+dy_f)a.$$

For forward=false:

$$t_x=c_x+(((-c_x+dx_f)c)-(((-c_y)/a+dy_f)s))Z,$$
$$t_y=c_y+((((-c_y)/a+dy_f)c)+((-c_x+dx_f)s))Za.$$

Preserve the displayed operation order. Check every required intermediate before use, including exp/log products and the resulting coefficients.

## Composition

For maps A then B, output C=B composed with A. All six coefficients remain independent:

$$t_{xC}=(t_{xB}+u_Bt_{xA})+v_Bt_{yA},\quad t_{yC}=(t_{yB}+w_Bt_{xA})+h_Bt_{yA},$$
$$u_C=u_Bu_A+v_Bw_A,\quad v_C=u_Bv_A+v_Bh_A,$$
$$w_C=w_Bu_A+h_Bw_A,\quad h_C=w_Bv_A+h_Bh_A.$$

Products round before their addition. Do not force h=u after composition; rounding can separate them. Identity is `(tx,ty,u,v,w,h)=(0,0,1,0,0,1)`.

## Inverse used by global analysis

This operator accepts the fitted similarity model, not an arbitrary six-coefficient affine map. Require h=u. Let b=sqrt((-w)/v) when v!=0, otherwise b=1. Require its argument nonnegative and all divisors below nonzero. Define

$$D=(u u)+(((v v)b)b),\quad u'=u/D,\quad h'=u',$$
$$v'=((-u')v)/u,\quad w'=((-v')b)b,$$
$$t'_x=((-u')t_x)-(v't_y),\quad t'_y=((-w')t_x)-(u't_y).$$

A zero u is an error for this operator, including an otherwise invertible quarter-turn. Do not silently substitute a different inverse at that boundary.

## Coordinates to motion

Inputs are a map, aspect and center, and the same forward flag. This conversion uses tx,ty,u,v; h and w do not enter the equations. Require u!=0. Set

$$\theta=-\operatorname{atan}((a v)/u),\quad r=(\theta\,180)/pi,\quad s=\sin\theta,\quad c=\cos\theta,\quad z=u/c.$$

Require c!=0. This is the single-ratio atan convention; substituting atan2 changes the result for negative u. For forward=true:

$$dx=(t_x-c_x)-(((-c_x)c+(c_y/a)s)z),$$
$$dy=((t_y/a)-(c_y/a))-((((-c_y)/a)c+(-c_x)s)z).$$

For forward=false, the following additions/subtractions associate left to right:

$$dx=(t_x/z)c+((t_y/z)/a)s-(c_x/z)c+c_x-((c_y/z)/a)s,$$
$$dy=((-t_x)/z)s+((t_y/z)/a)c+(c_x/z)s-((-c_y)/a)-((c_y/z)/a)c.$$

This branch additionally requires z!=0. Conversion does not apply property-reader magnitude clamps to produced motion.

## Examples

- With center (10,8),a=1,r=0,z=1,f=1,dx=2,dy=-1, either direction flag gives `(tx,ty,u,v,w,h)=(2,-1,1,0,0,1)` numerically. Output (10,10) samples (12,9). f=-1 instead samples (8,11). These exact values do not authorize simplifying the center arithmetic for arbitrary fractional motion.
- Two coefficient-map translations (2,-1) then (3,4) compose to (5,3). Their inverse is (-5,-3).
- For coefficient maps A(x,y)=(2x,2y) and B(x,y)=(x+3,y), B composed with A gives (2x+3,2y). The reverse order gives (2x+6,2y); changing temporal composition order changes observable sampling.
- Center (10,8), a=1, no translation/rotation, z=4,f=0.5 gives approximately Z=2 and `(X,Y)=(2x-10,2y-8)`, within the permitted elementary-function results. A direct coefficient map `(tx,ty,u,v,w,h)=(-10,-8,2,0,0,2)` maps the center exactly to itself.
- u=v=w=0 fails analysis inversion; it is not valid zero motion. Identity inversion succeeds.
