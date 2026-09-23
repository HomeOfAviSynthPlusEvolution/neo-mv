# 三种 mask 的共同输入和数值规则

## 输出由分析元数据决定

VectorLengthMask、SADMask、OcclusionMask 只读取向量帧属性，不读载体像素，也不请求 Super 或 `n+delta` 图像。输出为分析实际尺寸的 GRAY 图像，帧数、帧率来自 vectors；分析精度为 8～16 时输出同精度整数，为 32 时输出 float32。整数最大值 `M=2^b-1`，浮点计算得分的上限 M=1。

创建时允许只有元数据，要求块网格完整覆盖实际图像并落在工作尺寸内。逐帧无效元数据、缺失数组或场景判定失败选择 scval；完整损坏数组和有效描述变化报错。除正 Levels 外，所有有效标量含 delta 固定。具体状态见 [分析数据](analysis-data.md)。

## 参数转换和得分量化

ml、gamma、scval 先转 binary32，要求 `ml>0,gamma≥0` 且三者有限。time 使用有限 binary64 的 0～100。常量：

$$f=fl32(1/ml),\quad t=trunc(fl64(fl64(time\cdot256)/100)).$$

每个函数自己的派生常量也在创建时检查。浮点下溢为零允许，但不能主动清除次正规数。得分 L 必须先有限，再量化为：

$$Q(L)=\begin{cases}trunc(\min(L,M))&\text{整数}\\fl32(\min(L,M))&\text{浮点}.\end{cases}$$

整数得分直接截断，不加 0.5。scval 使用另一条规则：整数回退值 `trunc(fl32(scval+0.5))`，转换后的数学整数须在 `[0,M]`；浮点回退直接用 scval，可超出 `[0,1]`。所以整数 scval=12.5 得 13，而计算得分 L=12.5 得 12。

## 幂函数

非负底数和指数下，`pow(0,0)=1,pow(x,0)=1,pow(0,g>0)=0,pow(x,1)=x`。其他幂按指定 binary32/64 精度计算，允许正确舍入值或相邻的有限非负可表示值，但同一输入须稳定。后续截断使用该实际值，没有额外的最终像素容差。

gamma=0 不豁免前面的中间计算有限性检查。OcclusionMask 只有发生且目标区间非空的事件才计算贡献，没事件不会因 `pow(0,0)` 变成白色。

## 像素和属性输出

可用场先算每块的量化值，再 [重采样到像素](grid-resampling.md)。不可用场直接整幅填转换后的 scval，不做网格运算。

输出属性只有 `_Range=[1]`，表示本接口的全范围，不继承输入的 Analysis、Super、场景、色彩属性。不替换成旧 `_ColorRange`，其数值约定不同；范围标记本身不会再缩放像素。

没有时间边界判定：最后一帧的完整合法外部向量即使 delta=1 也能生成 mask。thscd1 默认 400、thscd2 默认 51.0，范围与计算见 [SCDetection](../sc-detection.md)。

[返回中文目录](../README.md)
