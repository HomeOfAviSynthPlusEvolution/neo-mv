# Estimation window geometry

Inputs are positive clip dimensions W,H and the normalized integer arguments winx,winy,wleft,wtop,dxmax,dymax plus finite binary32 zoommax. Each integer argument first saturates from int64 to signed int32. Output is final even width wx, height wy, top, one or two left coordinates, search limits mx,my and a boolean two=(zoommax!=1). Parameter errors occur at creation, including when show/info are disabled.

## Horizontal geometry

Let left0 be wleft and L=max(left0,0). Require 0<=winx<=W-L and winx even. If winx=0, choose the greatest power of two not exceeding min(W-L,8192). A nonexistent positive choice or a result of 1 is an error. Otherwise retain the supplied winx; the 8192 ceiling applies only to automatic selection.

Without two-window mode, wx is this width. In two-window mode divide it by two, requiring the result to be positive and even. Thus the pre-halved width must be divisible by four. Search limits below refer to the post-halving width.

If left0<0, choose left=floor((W-wx)/2) in one-window mode, or left=floor((W-2wx)/4) in two-window mode. Otherwise left=left0. The second rectangle, when used, starts at left2=left+floor(W/2), not at left+wx. Require every rectangle to lie within the first plane: left>=0, left+wx<=W, and left2+wx<=W when two=true. The separation D=left2-left is positive.

## Vertical geometry and limits

Let top0=wtop and T=max(top0,0). Require 0<=winy<=H-T. If winy=0, choose the greatest power of two not exceeding min(H-T,8192), failing if none exists; otherwise wy=winy. If top0<0 set top=floor((H-wy)/2), otherwise top=top0. Require wy>0, top>=0 and top+wy<=H. Supplied odd heights are supported.

Set mx=floor(wx/4) when dxmax<0, otherwise mx=dxmax. Set my=floor(wy/4) when dymax<0, otherwise my=dymax. Require

$$0\le m_x<\lfloor w_x/2\rfloor,\qquad 0\le m_y<\lfloor w_y/2\rfloor.$$

These checks exclude a one-row window. Zero search extent is supported; it is not an automatic request. Negative wleft/wtop and negative dxmax/dymax mean automatic selection, regardless of their magnitude after saturation. Neither fields nor pixaspect changes window geometry. There is no automatic frame resize or border extension.

## Examples

- W=1920,H=1080 and all geometry defaults, zoommax=1: wx=1024,wy=1024,left=448,top=28,mx=my=256.
- The same inputs with zoommax=1.1: wx=512,wy=1024,left=224,left2=1184,top=28,mx=128,my=256.
- W=12,H=5,winx=8,winy=3,other geometry defaults,zoommax=1: left=2,top=1,mx=2,my=0. An odd height is a valid transform dimension.
- W=100,H=20,winx=12,winy=4,wleft=45,wtop=0,zoommax=1.1 fails: the right rectangle would end at 45+50+6=101. The initial pre-halving width alone fits, which is insufficient.
- winx=6 with two=true is an error because the final width would be 3. A height of 1 fails the my bound even when dymax=0. A 2x2 one-window rectangle with mx=my=0 is supported.
- W=20000, automatic width gives 8192; an explicit even winx=10000 can be supported when its rectangle and storage fit.
