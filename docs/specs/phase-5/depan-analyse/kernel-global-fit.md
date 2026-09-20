# Global motion fitting

Inputs are observations and initial weights, aspect a>0, flags zoom/rot, error limit, and the weight-selection inputs. Output is a final map, residual error E, diagnostic iteration number iter, and a validity decision. Use the [common binary32 arithmetic](../README.md#common-contracts). This finite recurrence defines the result; an unrestricted least-squares solution is not interchangeable with it.

Initialize tx=ty=v=w=0, u=h=1, E=2*error, iter=0. For an ineligible field stop here with invalid motion; do not run any update or parity operation. For an eligible field use the following operator and schedule.

## One update

For each observation, using the old map throughout this update:

$$e_{xi}=(((t_x+uX_i)+vY_i)-X_i)-dx_i,$$
$$e_{yi}=(((t_y+wX_i)+hY_i)-Y_i)-dy_i.$$

Initialize each of N,X2,Y2,R to fl32(0.1). In row-major order add the following term to its own accumulator, rounding on each addition:

$$N:\ b_i,\quad X2:\ \operatorname{fl32}(X_i^2)b_i,\quad Y2:\ \operatorname{fl32}(Y_i^2)b_i,\quad R:\ ((e_{xi}e_{xi})+(e_{yi}e_{yi}))b_i.$$

Integer Xi squared and Yi squared are formed exactly before conversion. Separately accumulate derivative numerators from +0 in row-major order. For the step's enable flags ze,re:

$$g_x={\sum (2e_{xi})b_i\over N\,2},\quad g_y={\sum (2e_{yi})b_i\over N\,2},$$
$$g_{xx}={\sum (2X_i)e_{xi}b_i\over (X2\,2)1.5},\quad g_{yy}={\sum (2Y_i)e_{yi}b_i\over (Y2\,2)1.5},$$
$$g_{xy}={\sum (2Y_i)e_{xi}b_i\over (Y2\,2)3},\quad g_{yx}={\sum (2X_i)e_{yi}b_i\over (X2\,2)3}.$$

The integer products 2Xi and 2Yi are exact before conversion. Each numerator term multiplies left to right, then is added to its accumulator. When ze=false the xx/yy numerators stay +0 and their terms are not evaluated; likewise xy/yx when re=false. The displayed divisions are still performed for these zero numerators. N,X2,Y2,R and the translation derivatives are always computed. Require the displayed divisors nonzero and R/N nonnegative.

With step size s, set E'=sqrt(R/N) and simultaneously update

$$t_x'=t_x-sg_x,\quad t_y'=t_y-sg_y,$$
$$u'=\begin{cases}u-(s\,0.5)(g_{xx}+g_{yy})&ze\\u&\text{otherwise},\end{cases}\qquad h'=u',$$
$$v'=v-(s\,0.5)(g_{xy}-g_{yx}/(a a)),\quad w'=((-a)a)v'.$$

Require a*a nonzero. E' measures the residuals before this update, not those of the new map. The xy update is evaluated even when rotation is disabled. All next-state right-hand sides use the old coefficients except w', which uses v'.

## Schedule and validity

- For k=0..4, use s=0.3, ze=re=false. Update the map and E, then [select weights](kernel-block-weights.md) with G=1000. All five steps run regardless of E.
- For k=5..99, use s=0.3 when k<8, s=0.6 when 8<=k<10, otherwise s=1. ze=zoom and re=rot. Save Eold, update to E'. Stop with iter=k if `(Eold-E'<0.01*0.5 and k>9)` or `E'<0.01`. The stopping step keeps its new map and E' and does not select weights again.
- Otherwise select weights with G=E'*2 and continue. If step 99 completes without stopping, iter=100.

After an eligible run, good=(E<error), strictly. An invalid result exports the standard invalid tuple, not the final map. A valid result is converted by the plugin, which may still encounter a coordinate-conversion error. No positive-weight-count condition is added. Negative error, wrong or zerow are not independently forbidden; this recurrence, its comparisons and its arithmetic-domain checks define their behavior.

## Examples

- One observation d=(2,0), b=1, identity map, s=0.3 and ze=re=false gives N=fl32(1.1), gx approximately -1.8181818, tx' approximately 0.54545456 and E' approximately 1.9306146. This is one update, not the final fit.
- Eligible zero observations with all base weights zero preserve identity. N=X2=Y2=R=fl32(0.1), so E=1 on every update and the run stops at iter=10. With error=15 it is valid; error=1 makes it invalid. A zero-weight fit is not automatically rejected.
- An ineligible field and error=15 yield E=30, iter=0 and `(dx,dy,r,z,good)=(0,0,0,1,0)`.
- E=error is invalid. A negative R/N caused by signed weights is a frame error, not an invalid-motion tuple.
