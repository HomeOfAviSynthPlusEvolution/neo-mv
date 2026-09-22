# Geometric adaptive-scale bound

Inputs are a finite map T=(tx,ty,u,v,w,h), positive W,H,cx,cy and positive z0. Output B(T) is one binary32 value. Start B=z0, then evaluate these candidates in order, replacing B only when the candidate is strictly smaller:

$$1+(t_x+v c_y)/c_x,$$
$$1-(((t_x+u W)+v c_y)-W)/c_x,$$
$$1+(t_y+w c_x)/c_y,$$
$$1-(((t_y+w c_x)+h H)-H)/c_y.$$

W,H convert to binary32 at their use sites. Use the full dimensions, not W-1 or H-1. This is a bound evaluated at four center-line intersections, not a search over image corners. Keep the first value on equal comparisons, including signed-zero ties. A non-finite required intermediate is an error. The output need not be positive; no lower clamp is applied here.

## Examples

- With W=100,H=80,cx=50,cy=40,z0=1 and T a translation (+10,-4), candidates are 1.2,0.8,0.9,1.1. B is approximately 0.8, with each arithmetic operation rounded to binary32.
- With the same dimensions and identity, B=1 for z0=1, and B=0.75 for z0=0.75.
- With translation +100 horizontally and z0=1, the right candidate is -1 and B=-1. This is a valid finite bound. A later call Z(-1) uses the motion-conversion rule and therefore produces unit scale.
