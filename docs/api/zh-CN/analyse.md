# Analyse

估计每帧到 n+delta 参考帧的分块运动。

## 调用方式

VapourSynth：`core.neo_mv.Analyse`；AviSynth：`neo_mv_Analyse`。参数顺序：

```text
Analyse(super [, blksize, levels, search, searchparam, pelsearch, mvlambda, chroma, delta, lsad, plevel, globalmv, pnew, pzero, pglobal, overlap, badsad, badrange, meander, trymany, fields, tff, satd, prefix])
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
| `satd` | 布尔 | `false` | 亮度使用 SATD，色度仍使用 SAD；不支持 6×6 和 16×2 块。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |

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
result = core.neo_mv.Analyse(s, delta=1, badrange=0)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
result = neo_mv_Analyse(s, delta=1, badrange=0)
return result
```

## 限制与常见错误

缺失 Super 数据、块几何或枚举无效、delta=0、pelsearch 非正或没有可用层时创建失败。完整搜索采样域须落在有效 Super 支持范围内。场模式要求 pel>1 和所需场序；SATD 拒绝 6×6 和 16×2。取帧时元数据变化或非有限计算会报错。

## 计算原理

公式、舍入及数值示例见 [Analyse 计算原理](../../knowledge/zh-CN/analyse.md)。

[API 目录](README.md)
