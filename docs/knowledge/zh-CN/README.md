# neo-mv 计算原理知识库

这里逐个解释 neo-mv 公开函数怎样把输入变成输出：数据表示、计算顺序、公式、参数进入计算的位置，以及边界和舍入怎样影响结果。每篇函数说明包含一个数值例子；多处共用的运算在独立文章中展开。

文中函数名省略宿主前缀：VapourSynth 使用 `core.neo_mv.Super(...)`，AviSynth 使用 `neo_mv_Super(...)`，其他函数同理。两者共用计算过程和参数名。AviSynth 使用 `[8, 8]` 这样的原生数组；`AnalyseMany` 和 `Recalculate` 返回 clip 数组，即使 `Recalculate` 只有一个输出也一样。`KernelInfo` 按 `[backend, target, fft, fft_lanes]` 的顺序返回数组。

## 图像层级与块运动

| 函数 | 计算内容 |
| --- | --- |
| [Super](super.md) | 边缘扩展、降采样金字塔与亚像素相位图 |
| [Analyse](analyse.md) | 候选预测、多层搜索、块误差与运动向量选择 |
| [AnalyseMany](analyse-many.md) | 多个时间参考的独立分析与输出排列 |
| [Recalculate](recalculate.md) | 把已有向量投到新块网格，再局部搜索 |
| [SCDetection](sc-detection.md) | 块误差阈值、坏块计数与场景标记 |

## 根据块运动生成图像

| 函数 | 计算内容 |
| --- | --- |
| [Compensate](compensate.md) | 参考块采样、当前块回退与重叠合成 |
| [Degrain](degrain.md) | 多参考误差权重、整数归一化、重叠合成与限幅 |
| [Degrain1～Degrain25](degrain.md#6-参数与计算步骤对照) | 固定参考对数的入口；各入口的向量数量列于参数表 |

## 运动掩模与逐像素位移

| 函数 | 计算内容 |
| --- | --- |
| [VectorLengthMask](vector-length-mask.md) | 块向量长度到灰度掩模的映射 |
| [SADMask](sad-mask.md) | 块误差的归一化、幂映射与量化 |
| [OcclusionMask](occlusion-mask.md) | 相邻向量分歧形成的遮挡分数 |
| [Flow](flow.md) | 稠密位移场与参考相位采样 |
| [FlowInter](flow-inter.md) | 固定时间位置的双向插值与遮挡修正 |
| [FlowFPS](flow-fps.md) | 有理帧率时间映射与双向插值 |
| [FlowBlur](flow-blur.md) | 沿前后运动轨迹取样并平均 |

## 全局运动与几何变换

| 函数 | 计算内容 |
| --- | --- |
| [DepanAnalyse](depan-analyse.md) | 从块运动迭代拟合平移、旋转与缩放 |
| [DepanEstimate](depan-estimate.md) | 从周期相关峰估计平移与双窗口缩放 |
| [DepanCompensate](depan-compensate.md) | 多段全局运动复合与图像重采样 |
| [DepanStabilise](depan-stabilise.md) | 累计运动、时间平滑、修正限制与图层覆盖 |
| [KernelInfo](kernel-info.md) | 普通计算核与 FFT 执行配置查询 |

## 共用计算

| 文章 | 内容 |
| --- | --- |
| [分析数据](shared/analysis-data.md) | 元数据、向量编码、有效与缺失数据的区别 |
| [块渲染](shared/block-rendering.md) | 参考块坐标、平面处理、重叠窗和整数累计 |
| [网格重采样](shared/grid-resampling.md) | 块网格到像素网格的坐标、系数和舍入 |
| [掩模输入](shared/mask-input.md) | 掩模共用的输入、无效填充与输出属性 |
| [双向插值](shared/bidirectional-interpolation.md) | 时间缩放、遮挡场、辅助向量与像素合成 |
| [全局运动](shared/global-motion.md) | Depan 属性、运动与映射的转换、复合和求逆 |
| [Depan 采样](shared/depan-sampling.md) | 三类坐标运算、最近邻/双线性/双三次及边界 |
| [稳像时间平滑](shared/stabilisation-smoothing.md) | 惯性递推、窗口平均、自适应缩放与恢复 |
| [稳像图层](shared/stabilisation-layers.md) | 前后图像选择、映射累计与逐层覆盖 |
