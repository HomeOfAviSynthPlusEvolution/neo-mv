# DepanCompensate：累积时间段内的全局变换并采样

## 1. 这个函数计算什么

`DepanCompensate` 根据 offset 选一张源帧，将源帧到目标帧之间的 Depan 运动逐段转换、组合为采样映射，再对源图像插值。它不消费块向量或 Super。

## 2. 计算对象与记号

输出 n，offset 为有限 binary32。offset>0 时 `I=ceil(offset)`，否则 `I=floor(offset)`，源帧 `s=n-I`。像素宽高比 `a=pixaspect/(fields?2:1)`，中心为 clip 宽高的一半。

## 3. 总体计算流程

先判断是否直接返回当前帧；否则按升序读取运动，组合映射，必要时做目标场匹配；读取 clip[s]，换算各平面映射并按所选插值渲染。输出继承所用源图像的属性。

## 4. 逐步计算

I=0 或 s 越出序列时直接返回 clip[n]，不读 motion，也不读场奇偶。这才是 bypass；以后得到单位映射不再触发这条路径。

渲染分支中 `forward=(I>0)`，每段统一使用比例：

$$f=fl32(fl32(offset+(forward?1:-1))-fl32(I)).$$

从单位映射开始，按 `k=min(s,n)+1…max(s,n)` 升序读 data[k]。按 [公共属性规则](shared/global-motion.md) 解码，首个合法但 good=false 的元组将整个映射重置为单位并停止；之后数据不再依赖。此前损坏元组仍报错。

每个有效元组用相同 f、forward、aspect 和中心转成 Tk，再令 `T=Tk∘T`。每段都乘 f，不是仅给最后一段使用小数时间。

fields=true 且 matchfields=true 时，目标 top 给最终 ty 加 -0.5，bottom 加 +0.5。即使因为无效运动重置成单位也要执行。场信息来自 clip[n] 或 tff，不需要源帧奇偶；禁用 matchfields 不取消 fields 对 aspect 的减半。

随后对 clip[s] 使用 [Depan 坐标与采样](shared/depan-sampling.md)。边框亮度0，色度为半量程；所有原属性来自 clip[s]，不把合成运动写回 Depan 属性。info=true 时由最终平面转换前映射生成 `DepanCompensate_info`，再调用文本渲染器。

## 5. 一个完整的数值例子

16×16 图像，n=5、offset=1.5，得 I=2、s=3、f=0.5，读取 data[4]、data[5]。若每段都是 dx=2、dy=rot=0、zoom=1，aspect=1，无场修正，每段变成水平平移1，组合得2。

subpixel=0 时，输出 `(0,0)` 读取 clip[3] 的 `(2,0)`。不是选 clip[4]，也不是总位移3。属性同样继承 clip[3]。

若首元组 good=false，映射单位、第二个元组不读，仍渲染 clip[3]；不会改取 clip[5]。

## 6. 参数与计算步骤对照

| 参数 | 默认与约束 | 作用 |
| --- | --- | --- |
| `clip,data` | 必需；data 至少有 clip 帧数 | 图像与运动元组 |
| `offset` | 0.0；binary32 有限 -10～10 | 源帧及每段比例 |
| `subpixel` | 2；int32 饱和后0～2 | 最近邻、双线性、三次 |
| `pixaspect` | 1.0；有限正 | 变换纵横尺度 |
| `matchfields` | true | fields 开启时目标场匹配 |
| `mirror` | 0；0～15 | 上下左右位掩码 |
| `blur` | 0；非负 int32 有效值 | 反射边缘水平平均 |
| `info` | false | 诊断与文本渲染 |
| `fields,tff` | false、省略 | aspect 减半与场奇偶来源 |

## 7. 边界、缺失数据与错误

只支持整数8～16位 GRAY、YUV420/422/444；模式2每平面高至少2，宽1允许。offset=0 仍检查参数与格式，但不读 data 属性。创建时也不要求 data[0] 的运动键。

info 渲染器在 bypass 上仍执行，但不新建诊断键，可能显示继承的同名属性。必须的源帧或已访问运动帧失败是错误，不变成 bypass。

## 8. 数值精度与结果确定性

所有段保持六系数组合，不强制两个对角相等。正负 offset 均按 data 索引升序组合。单位映射仍进入所选采样边缘规则，例如双线性无右镜像时可能填右边缘，不作通用复制优化。

[返回中文目录](README.md)
