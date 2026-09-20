# Occlusion event mask grid

Inputs are a Complete scene-eligible field, saved signed DeltaFrame d, block geometry, pel p, maximum M and shared f,gamma,t. Output is one quantized sample per block. Start the Nx by Ny grid at zero. Contributions from converging horizontal and vertical neighbors are combined by maximum. SAD only affects the earlier scene-eligibility decision.

At creation, with sx=Bx-Ox, sy=By-Oy, compute

$$h_x=\operatorname{trunc}(16t/(s_xp)),\quad h_y=\operatorname{trunc}(16t/(s_yp)),$$
$$a_x=\operatorname{fl32}(\operatorname{fl32}(80f)/\operatorname{fl32}(s_xp)),\quad a_y=\operatorname{fl32}(\operatorname{fl32}(80f)/\operatorname{fl32}(s_yp)).$$

Require finite floating constants and their required intermediates. Products/differences used for intervals are exact wide integers.

## Horizontal event

For every adjacent pair (bx,by),(bx+1,by), let o=vx(bx,by)-vx(bx+1,by). If o<=0, generate no event. Otherwise let k=trunc(o*hx/4096) and use the inclusive column interval

$$[l,r]=\begin{cases}[\max(0,b_x+1-k),\ b_x+1]&d>0,\\[b_x,\ \min(b_x+1-k,N_x-1)]&d\le0.\end{cases}$$

If l>r, the event is empty and has no contribution. Do not swap the endpoints. Otherwise compute a contribution using a=ax:

$$L=\begin{cases}\operatorname{fl32}(\operatorname{fl32}(M\operatorname{fl32}(o))a)&\gamma=1,\\\operatorname{fl32}(M\operatorname{pow32}(\operatorname{fl32}(\operatorname{fl32}(o)a),\gamma))&\gamma\ne1.\end{cases}$$

Require finite intermediates and L, then set G(c,by)=max(G(c,by),Q(L)) for each integer l<=c<=r. The gamma=1 branch's multiplication order is normative.

## Vertical event and result

For every adjacent pair (bx,by),(bx,by+1), use o=vy(bx,by)-vy(bx,by+1), hy and ay. Apply the same equations with by in place of bx, Ny in place of Nx, and update the inclusive row interval at fixed column bx. No comparisons across a row or column wrap are made. The last column can participate in vertical events and the last row in horizontal events.

For each grid location take the maximum of zero and all nonempty event contributions. Horizontal and vertical contributions are not summed. [Q and power](../vector-length-mask/kernel-mask-input.md) apply before this maximum. No event means zero, even for gamma=0. Then apply [grid resampling](../vector-length-mask/kernel-grid-resampling.md). time=0 is allowed and does not in general eliminate events.

## Examples

These are eligible integer8 kernel inputs with Nx=2,Ny=1,Bx=By=4,overlap=0,p=1,ml=80,gamma=1 and adequate public vector padding.

- vx=[4,0],time=100: o=4,hx=1024,k=1,ax=0.25,L=255. For d>0 the grid is [255,255]; for d<=0 it is [255,0].
- vx=[8,0],time=100,d<0: k=2, interval [0,-1] is empty; output grid [0,0]. It must not be turned into [255,0] by swapping or clamping endpoints.
- vx=[4,0],time=0: k=0. For d>0 the grid is [0,255]; for d<=0 it is [255,255].
- vx=[0,4] generates no horizontal event. With vy identical across rows and gamma=0 the entire grid remains zero.
- With Nx=Ny=2 and both vx and vy grids [[4,0],[0,0]],time=100,d>0, horizontal and vertical events yield [[255,255],[255,0]]. The shared top-left block is 255, not the sum 510.
