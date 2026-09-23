# Recalculate

将已有矢量映射到目标网格，并按新测误差进行细化搜索。

## 调用方式

VapourSynth：`core.neo_mv.Recalculate`；AviSynth：`neo_mv_Recalculate`。参数顺序：

```text
Recalculate(super, vectors [, thsad, smooth, blksize, search, searchparam, mvlambda, chroma, pnew, overlap, meander, fields, tff, satd, prefix])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `super` | 剪辑 | 必填 | 携带辅助图像的 Super 剪辑；前缀须与生成时一致。 |
| `vectors` | 剪辑数组 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `thsad` | 整数 | `200` | 有符号 int32 阈值：映射矢量的新测误差超过缩放阈值时搜索。允许负值。 |
| `smooth` | 布尔 | `true` | 插值旧矢量网格；false 选择最近的旧块。 |
| `blksize` | 整数数组 | 继承 Super | 目标 [宽, 高]，省略或空数组时继承 Super；支持的组合和对齐要求同 Super。 |
| `search` | 整数 | `2` | 搜索模式 0～5，见下方模式表。 |
| `searchparam` | 整数 | `2` | 粗层搜索参数（Recalculate 为其唯一层）；小于 1 按 1 处理。不是所有模式下的最大位移限制。 |
| `mvlambda` | 整数 | `1000` | 相对预测矢量的距离惩罚，非负；0 关闭该惩罚。 |
| `chroma` | 布尔 | `true` | 在存在色度时将其计入匹配误差；GRAY 强制关闭。 |
| `pnew` | 整数 | `25` | 新候选误差惩罚，0～256。 |
| `overlap` | 整数数组 | 继承 Super | 目标 [水平, 垂直] 重叠，省略或空数组时继承 Super；各轴为 0 到半块且满足色度对齐。 |
| `meander` | 布尔 | `true` | 相邻块行交替左右遍历方向。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |
| `tff` | 布尔 | 省略 | 显式指定第 0 帧是否为顶场，随后按帧号交替；省略时读取所需帧的 `_Field` 属性。 |
| `satd` | 布尔 | `false` | 亮度使用 SATD，色度仍使用 SAD；不支持 16×2 块。 |
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

使用前缀一致、受支持 GRAY/YUV 格式的 Super。轴数组可传单值或单元素数组（两轴相同），或两个值 [水平, 垂直]；空数组继承 Super，超过两项报错。块尺寸与对齐规则见 [Super](super.md)。 vectors 须为非空剪辑数组。旧数据采样精度须匹配 Super；旧块尺寸、pel 和网格可以不同。

## 返回值与属性

按输入顺序返回等长剪辑数组，即使只有一个成员也返回数组。每个结果保留 Super 可见图像和时间轴，写入重新计算的 Analysis 属性，delta 保留为该成员原始值。越界参考取最近的源帧。

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
v = core.neo_mv.AnalyseMany(s, radius=1, badrange=0)
result = core.neo_mv.Recalculate(s, v, blksize=8, overlap=4)
result[0].set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_Recalculate(s, v, blksize=8, overlap=4)
return result[0]
```

`result` 是数组，示例仅输出成员 0；可以索引其他成员或将整个数组交给支持的下游函数。

## 限制与常见错误

vectors 为空时报错。元数据须有效；旧数组缺失或数量不符时使用零旧矢量，完整但损坏的数组报错。fields=true 要求旧 pel>1 和所需场序。目标网格须有有效采样支持，后续不兼容的描述变化会报错。

## 计算原理

公式、舍入及数值示例见 [Recalculate 计算原理](../../knowledge/zh-CN/recalculate.md)。

[API 目录](README.md)
