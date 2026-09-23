# API 使用参考

这里收录全部 44 个公开入口的签名、默认值、参数范围、输入输出要求和可运行的 VapourSynth/AviSynth 示例。25 个编号 Degrain 变体与 Degrain 共用一页。

VapourSynth 使用 `core.neo_mv.Name`，AviSynth 使用 `neo_mv_Name`；分别通过 `core.std.LoadPlugin` 或 `LoadPlugin` 加载插件。示例使用空白源和占位插件路径，运行前请替换路径。

数组参数允许用单值简写一个元素；Python 使用 `[16, 8]` 这样的列表，AviSynth 使用同样写法的原生数组。省略参数与显式空数组并不总是等价，具体规则见各页。`AnalyseMany` 和 `Recalculate` 返回剪辑数组，Recalculate 只有一个输入时也如此。

| 函数 | 用途 |
| --- | --- |
| [Super](super.md) | 构建多层及亚像素参考图像。 |
| [Analyse](analyse.md) | 估计每帧到 n+delta 参考帧的分块运动。 |
| [AnalyseMany](analyse-many.md) | 生成按距离排列的双向运动分析剪辑数组。 |
| [Recalculate](recalculate.md) | 将已有矢量映射到目标网格，并按新测误差进行细化搜索。 |
| [SCDetection](sc-detection.md) | 根据存储的块误差附加场景切换标记。 |
| [Compensate](compensate.md) | 使用一个矢量剪辑进行分块运动补偿。 |
| [Degrain / Degrain1–Degrain25](degrain.md) | 按匹配误差加权合成当前图像与运动补偿参考；带编号的入口固定矢量对数。 |
| [Flow](flow.md) | 将块矢量网格展开后进行逐像素运动补偿。 |
| [FlowInter](flow-inter.md) | 在第 n 帧和 n+d 帧之间插值，不改变时间轴。 |
| [FlowFPS](flow-fps.md) | 使用运动插值将时间轴转换为目标帧率。 |
| [FlowBlur](flow-blur.md) | 沿当前帧两侧的运动轨迹平均采样。 |
| [VectorLengthMask](vector-length-mask.md) | 将块矢量长度映射为蒙版。 |
| [SADMask](sad-mask.md) | 按矢量投影后将存储的块误差映射为蒙版。 |
| [OcclusionMask](occlusion-mask.md) | 根据相邻块矢量差异生成遮挡蒙版。 |
| [DepanAnalyse](depan-analyse.md) | 根据已有块矢量拟合相邻帧全局运动。 |
| [DepanCompensate](depan-compensate.md) | 按整数或小数帧偏移应用全局运动补偿。 |
| [DepanEstimate](depan-estimate.md) | 使用第一平面的 FFT 相关估计相邻帧平移及可选缩放。 |
| [DepanStabilise](depan-stabilise.md) | 平滑全局运动并渲染修正图像，可使用邻帧填补边缘。 |
| [KernelInfo](kernel-info.md) | 查询选用的计算与 FFT 后端。 |

各函数背后的计算过程见[计算知识库](../../knowledge/zh-CN/README.md)。
