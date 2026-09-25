# Analyse

估计每帧到 n+delta 参考帧的分块运动。

## 调用方式

VapourSynth：`core.neo_mv.Analyse`；AviSynth：`neo_mv_Analyse`。参数顺序：

```text
Analyse(super [, blksize, levels, search, searchparam, pelsearch, mvlambda, chroma, delta, lsad, plevel, globalmv, pnew, pzero, pglobal, overlap, badsad, badrange, meander, trymany, fields, tff, metric, prefix, metric_weight, metric_threshold])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `super` | 剪辑 | 必填 | 携带辅助图像的 Super 剪辑；前缀须与生成时一致。 |
| `blksize` | 整数数组 | 继承 Super | 目标 [宽, 高]，省略或空数组时继承 Super；支持的组合和对齐要求同 Super。 |
| `levels` | 整数 | `0` | 0 自动决定层数；正数指定层数，负数从自动层数中扣减。受 Super 可用层数限制，最终至少一层。 |
| `search` | 整数 | `2` | 搜索模式 0～5，见下方模式表。 |
| `searchparam` | 整数 | `2` | 粗层搜索参数（Recalculate 为其唯一层）；小于 1 按 1 处理。不是所有模式下的最大位移限制。 |
| `pelsearch` | 整数 | `SuperPel` | 最细层搜索参数，以亚像素矢量单位计，必须为正；默认取 Super 的 pel。 |
| `mvlambda` | 整数 | `1000` | 相对预测矢量的距离惩罚，非负；0 关闭该惩罚。 |
| `chroma` | 布尔 | `true` | 在存在色度时将其计入匹配误差；GRAY 强制关闭。 |
| `delta` | 整数 | `1` | 带符号且非零的参考偏移：第 n 帧分析 n+delta 帧；正数表示后面的参考帧。 |
| `lsad` | 整数 | `400` | 控制距离惩罚调整的预测误差阈值；允许负值。 |
| `plevel` | 整数 | `1` | 金字塔层级惩罚缩放方式：0、1 或 2。 |
| `globalmv` | 布尔 | `true` | 启用全局运动预测候选。 |
| `pnew` | 整数 | `25` | 新候选误差惩罚，0～256。 |
| `pzero` | 整数 | `pnew` | 零矢量初始候选惩罚，0～256；默认取有效 pnew。 |
| `pglobal` | 整数 | `0` | 全局初始候选惩罚，0～256；globalmv=false 时有效值取 pzero。 |
| `overlap` | 整数数组 | 继承 Super | 目标 [水平, 垂直] 重叠，省略或空数组时继承 Super；各轴为 0 到半块且满足色度对齐。 |
| `badsad` | 整数 | `10000` | 触发坏块扩展搜索的原始误差阈值；允许负值。 |
| `badrange` | 整数 | `24` | 正数使用多尺度扩展，负数使用扩张环，0 关闭这两类扩展；仍可能进行局部亚像素细化。 |
| `meander` | 布尔 | `true` | 相邻块行交替左右遍历方向。 |
| `trymany` | 整数 | `0` | 0 从选定初始候选搜索；1 在粗层分别尝试多个候选；2 在所有层尝试。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |
| `tff` | 布尔 | 省略 | 显式指定第 0 帧是否为顶场，随后按帧号交替；省略时读取所需帧的 `_Field` 属性。 |
| `metric` | 字符串 | `"sad"` | 亮度匹配度量：纯 SAD/SATD/DCT 或局部/全局混合模式；色度仍使用 SAD。限制见下文。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |
| `metric_weight` | 浮点数 | `0.5` | local 触发后变换误差所占比例，[0,1]，精度 1/65536。 |
| `metric_threshold` | 浮点数 | `0.03125` | local 相对亮度和变化门限，[0,1]，精度 1/65536。 |

### 匹配度量

`metric` 接受区分大小写的字符串，默认 `"sad"`，替代旧的 `satd` 布尔参数。

| 值 | 亮度误差 | 格式与块尺寸 |
| --- | --- | --- |
| `"sad"` | 像素绝对差之和。 | 8–16 位整数和 float32；全部合法块尺寸。 |
| `"satd"` | 基于 4×4 Hadamard 变换的绝对差。 | 8–16 位整数和 float32；块宽、高均须能被 4 整除，不支持 6×6 和 16×2。 |
| `"dct"` | 分别对源块和参考块做 DCT-II、量化系数，再计算带 DC 权重和块尺寸缩放的系数绝对差。 | 仅 8–16 位整数；全部合法块尺寸，包括 6×6 和 16×2。 |

`chroma=true` 时，色度在所有模式下都使用 SAD；`metric` 只改变亮度度量。DCT 的 AC 系数采用最近偶数舍入，DC 使用整数截断；它不是未量化 DCT 系数的直接距离，也不保证复现历史实现的浮点舍入差异。

输出属性仍名为 `AnalysisSAD`，但保存的是所选度量计算的块误差（启用色度时包含色度 SAD），不是始终保存像素 SAD。切换度量会改变误差分布；误差阈值不会自动换算为等效 SAD 阈值。


#### 混合模式

以下五种新模式仅接受 8–16 位整数。所有 SATD 家族模式都要求块宽、高能被 4 整除，即使权重为零也不放宽尺寸限制；DCT 家族支持全部合法块尺寸。既有 `sad`、`satd`、`dct` 的数值定义和默认行为不变。

| `metric` | 亮度误差策略 |
| --- | --- |
| `sad_dct_global` | SAD 与 DCT 混合，帧对统一决定权重；DC 权重为 4。 |
| `sad_dct_local` | 逐候选检查亮度变化，触发后按 `metric_weight` 混合；DC 权重为 1。 |
| `sad_satd_global` | SAD 与 SATD 混合，帧对统一决定权重。 |
| `sad_satd_local` | 逐候选检查亮度变化，触发后按 `metric_weight` 混合。 |
| `sad_satd_global_half` | 全局 SATD 权重减半，最多占 50%。 |

`metric_weight` 和 `metric_threshold` 只允许在两个 local 模式中显式传入。二者须为 [0,1] 内的有限数，分别默认为 0.5 和 0.03125；创建时量化至最近的 1/65536 网格，正好居中时取较大值。非 local 模式显式传入任一个都会报错，即使等于默认值。参数追加在原签名末尾。

设源块、候选参考块的亮度样本和为 Ls、Lr；仅当 `abs(Ls-Lr)*65536 > (Ls+Lr)*H` 时触发，其中 H 是量化门限。触发后亮度误差为 `(S*(65536-W)+X*W)/65536`，W 为量化权重，X 为相应变换误差；最后做一次整数截断。未触发则使用 SAD。两块均为零或门限恰好相等均不触发。weight=0 或 threshold=1 恒为 SAD；weight=1 只在触发后使用纯变换误差。

全局模式从最粗层实际块网格的零位移亮度差求有符号平均值，再归一化为 0–16 的整数权重；正负变化可以抵消，重叠区域重复计入。最粗层本身使用 SAD，细层复用同一权重。只有一层时全局模式就是 SAD。AnalyseMany 的每个成员独立统计。色度始终只加一次 SAD，阈值不随度量自动换算。

#### 从 MVTools 的 dct 参数迁移

这是策略对应表，不是不同 DCT 后端之间的逐位兼容保证。表中空缺的新参数应省略。

| 原 `dct` | 新 `metric` | `metric_weight` | `metric_threshold` |
| ---: | --- | ---: | ---: |
| 0 | `sad` | — | — |
| 1 | `dct` | — | — |
| 2 | `sad_dct_global` | — | — |
| 3 | `sad_dct_local` | 0.5 | 0.03125 |
| 4 | `sad_dct_local` | 0.75 | 0.03125 |
| 5 | `satd` | — | — |
| 6 | `sad_satd_global` | — | — |
| 7 | `sad_satd_local` | 0.5 | 0.03125 |
| 8 | `sad_satd_local` | 0.75 | 0.03125 |
| 9 | `sad_satd_global_half` | — | — |
| 10 | `sad_satd_local` | 0.25 | 0.0625 |

局部比例可以自行设置，例如 `metric="sad_satd_local", metric_weight=0.6, metric_threshold=0.04`。比例和门限互相独立。与上游分项截断相比，在基础误差和触发决策相同的前提下，单次最终截断的 50% 组合可能高 0–1，25%/75% 组合可能高 0–2。这不限制最终向量差异；候选胜负可能改变。


### 搜索模式

| 值 | 候选模式 |
| --- | --- |
| 0 | 带方向提示的递减步长搜索。 |
| 1 | 围绕固定起点的同心方环。 |
| 2 | 六边形搜索与局部细化。 |
| 3 | 十字、稀疏模式和六边形细化。 |
| 4 | 围绕固定起点的纯水平搜索。 |
| 5 | 围绕固定起点的纯垂直搜索。 |

搜索参数控制遍历过程，不是所有模式下的位移半径限制。`fields=true` 使用场序信息；显式 `tff` 表示第一个场的场序，不是所有帧都固定同一场序。

## 输入要求

使用前缀一致、受支持 GRAY/YUV 格式的 Super。轴数组可传单值或单元素数组（两轴相同），或两个值 [水平, 垂直]；空数组继承 Super，超过两项报错。块尺寸与对齐规则见 [Super](super.md)。

## 返回值与属性

保留 Super 的可见图像、尺寸、格式、帧数、帧率及无关属性。写入指定前缀的 `Analysis*` 元数据，参考可用时另写 `AnalysisVectors` 和 `AnalysisSAD`。n+delta 越界时仍输出元数据，但移除这两个数组。可见画面不是矢量可视化。

AviSynth 从第一个输入 Super 传递音频和场序。

## 最短示例

替换插件路径即可运行；示例使用空白源，不需要外部视频读取插件，所选输出可直接取帧。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neo_mv.Super(clip, blksize=8, overlap=4, pad=32)
result = core.neo_mv.Analyse(s, delta=1, badrange=0, metric="dct")
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
result = neo_mv_Analyse(s, delta=1, badrange=0, metric="dct")
return result
```

## 限制与常见错误

缺失 Super 数据、块几何或枚举无效、delta=0、pelsearch 非正或没有可用层时创建失败。完整搜索采样域须落在有效 Super 支持范围内。场模式要求 pel>1 和所需场序；SATD 拒绝 6×6 和 16×2；DCT 和混合模式拒绝 float32，未知或大小写错误的 metric 字符串也会报错。取帧时元数据变化或非有限计算会报错。

## 计算原理

公式、舍入及数值示例见 [Analyse 计算原理](../../knowledge/zh-CN/analyse.md)。

[API 目录](README.md)
