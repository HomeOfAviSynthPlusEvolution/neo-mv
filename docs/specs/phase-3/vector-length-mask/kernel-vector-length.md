# Vector magnitude mask grid

Inputs are a Complete scene-eligible block field, pel p, mask maximum M and the shared binary32 f and gamma. Output is one mask sample per block, with the same Nx by Ny row-major grid. SAD values do not affect this operator after eligibility is established. time is validated by the plugin but does not affect this mask.

At creation form finite f2=fl32(f*f) and g2=fl32(gamma/2). For each signed public vector vx,vy define

$$d_x=\operatorname{fl64}(v_x/p),\qquad d_y=\operatorname{fl64}(v_y/p),$$
$$q=\operatorname{fl64}(\operatorname{fl64}(d_xd_x)+\operatorname{fl64}(d_yd_y)),\qquad a=\operatorname{fl64}(q\operatorname{fl64}(f_2)),$$
$$L=\operatorname{fl64}(M\operatorname{pow64}(a,\operatorname{fl64}(g_2))),\qquad G(b_x,b_y)=Q(L).$$

The first divisions are floating divisions. Q and power accuracy follow the [shared mask contract](kernel-mask-input.md). Vector components are in pel units; dividing by p converts their magnitude to luma pixels. Do not include padding, block coordinates or the sign of DeltaFrame in this magnitude. Require finite required intermediates and L. Feed the already quantized grid into [grid resampling](kernel-grid-resampling.md).

## Examples

- Integer8, p=1, ml=5, gamma=1: vector (3,4) gives 255 after saturation; (0,0) gives 0. With p=2, vector (6,8) gives the same value.
- Integer8, p=1, ml=8, gamma=2: vector (4,0) has a=0.25 and g2=1, giving L=63.75 and block value 63. A constant grid of these vectors gives a visible image of 63.
- Float32 with the preceding parameters gives 0.25. With gamma=0 every valid vector, including zero, produces M. An ineligible field still produces scval, not M.
- Changing time from 100 to 0 leaves these values unchanged; time=-1 is nevertheless a creation error.
