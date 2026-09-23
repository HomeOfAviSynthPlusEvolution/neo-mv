# 全局运动：属性、坐标变换与组合

## 1. 五个运动属性

`Depan_dx,Depan_dy,Depan_rot,Depan_zoom` 是各一个 float64，保存计算所得 binary32 值的精确提升；`Depan_goodmotion` 是一个 int64。dx 为水平像素位移，dy 是按像素宽高比归一化的垂直位移，rot 为角度，zoom 为乘法尺度，goodmotion 非零表示有效。

`DepanAnalyse` 与 `DepanEstimate` 的无效结果都使用标准元组 `(0,0,0,1,0)`。消费者仍须读取全部五键：缺失、空、类型错误或非有限浮点输入，即使 good=0 也报错。只读取元素 0。

有限 float64 先舍入为 binary32，再把 dx/dy 夹到 ±1000000、rot 夹到 ±360000、zoom 夹到 `[fl32(0.01),100]`。有限大值转换溢出时取相应夹取端点。零或负 zoom 变成 0.01，不代表数据缺失。

## 2. 运动元组转换为反向采样映射

映射定义为输出 `(x,y)` 在源图中取 `(X,Y)`：

$$X=t_x+ux+vy,\qquad Y=t_y+wx+hy.$$

它是查找源像素的反向映射，不是把源像素向外投射。令像素宽高比 a>0，中心 cx、cy，时间比例 f，运动 dx、dy、r、z。所有算术在未特别说明时逐步 binary32 舍入：

$$dx_f=f\,dx,\quad dy_f=f\,dy,\quad \theta=((fr)\pi)/180,\quad Z=\exp(f\log z).$$

`|θ|<1e-6` 时置 +0；`|Z-1|<1e-6` 时置 1。此转换自身把 z≤0 替为 1，但正常属性解码已先将 z 夹正。π 为 binary32。设 s=sinθ、c=cosθ：

$$u=h=cZ,\quad v=((-s)/a)Z,\quad w=(sZ)a.$$

forward=true 时：

$$t_x=(c_x+(((-c_x)c+(c_y/a)s)Z))+dx_f,$$
$$t_y=c_y+((((-c_y)/a)c+(-c_x)s)Z+dy_f)a.$$

forward=false 时：

$$t_x=c_x+(((-c_x+dx_f)c)-(((-c_y)/a+dy_f)s))Z,$$
$$t_y=c_y+((((-c_y)/a+dy_f)c)+((-c_x+dx_f)s))Za.$$

中心项使旋转/缩放围绕指定中心发生，两个方向对平移与旋转组合的顺序不同。纯平移可能看起来一样，不能由此把两套式子合并。

## 3. 映射组合和分析反变换

先 A 后 B，即 `C=B∘A`：

$$t_{xC}=(t_{xB}+u_Bt_{xA})+v_Bt_{yA},\quad t_{yC}=(t_{yB}+w_Bt_{xA})+h_Bt_{yA},$$
$$u_C=u_Bu_A+v_Bw_A,\quad v_C=u_Bv_A+v_Bh_A,$$
$$w_C=w_Bu_A+h_Bw_A,\quad h_C=w_Bv_A+h_Bh_A.$$

乘法先舍入再加法；组合后六个系数独立，不强制 h=u。缩放两倍后平移 3 是 `2x+3`，反过来是 `2x+6`，因此组合次序影响结果。

DepanAnalyse 对拟合相似模型使用特定逆运算，要求 h=u。v≠0 时 `b=sqrt((-w)/v)`，否则 b=1；然后：

$$D=u^2+((v^2)b)b,\quad u'=h'=u/D,\quad v'=((-u')v)/u,\quad w'=((-v')b)b,$$
$$t'_x=((-u')t_x)-(v't_y),\quad t'_y=((-w')t_x)-(u't_y).$$

根号需非负，除数非零。即使某个仿射矩阵从线性代数看可逆，这个特定运算遇到 u=0 仍报错。

## 4. 映射转换回运动

转换使用 tx、ty、u、v，要求 u≠0：

$$\theta=-atan((av)/u),\quad r=(\theta180)/\pi,\quad s=\sin\theta,\quad c=\cos\theta,\quad z=u/c.$$

要求 c≠0，使用单比值 atan，不替换成 atan2。forward=true：

$$dx=(t_x-c_x)-(((-c_x)c+(c_y/a)s)z),$$
$$dy=((t_y/a)-(c_y/a))-((((-c_y)/a)c+(-c_x)s)z).$$

forward=false 的加减依次左结合：

$$dx=(t_x/z)c+((t_y/z)/a)s-(c_x/z)c+c_x-((c_y/z)/a)s,$$
$$dy=((-t_x)/z)s+((t_y/z)/a)c+(c_x/z)s-((-c_y)/a)-((c_y/z)/a)c.$$

此分支还要求 z≠0。生产输出时不套用属性读取器的幅度夹取。

## 5. 场信息与诊断

需要场奇偶时，显式 tff 优先：`top(n)=bool(tff) XOR (n 为奇数)`；否则读取调用函数指定帧的整数 `_Field` 元素 0，非零表示顶场。缺失、空值或类型不符均报错。不从 `_FieldBased` 或上一帧推断。不需要场奇偶的分支不读取此属性。

info=true 的函数先写自己的诊断字符串，再调用宿主属性文本渲染器；VapourSynth 使用 `text.FrameProps`，只指定相应属性名，其他参数取渲染器默认。最后像素和属性以渲染器输出为准。缺少渲染能力报错，不能悄悄只写属性。info=false 不需要渲染器，也不删除继承来的同名诊断。

## 6. 数值约定

几何整数在转换前精确计算；浮点逐操作舍入，不融合乘加。sqrt 使用正确舍入的 binary32；sin/cos/atan/log/exp 允许正确舍入值或相邻有限值，但对固定输入稳定，后续计算遵循实际取值。`sin(±0)` 保留符号，`atan(±0)` 也保留符号，因而零旋转可能显示为 -0.000。

[返回中文目录](../README.md)
