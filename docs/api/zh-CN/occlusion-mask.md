# OcclusionMask

根据相邻块矢量差异生成遮挡蒙版。

## 调用方式

VapourSynth：`core.neomv.OcclusionMask`；AviSynth：`neo_mv_OcclusionMask`。参数顺序：

```text
OcclusionMask(vectors [, ml, gamma, time, scval, thscd1, thscd2, prefix])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `vectors` | 剪辑 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `ml` | 浮点数 | `100.0` | 有限正归一化尺度，分别作用于矢量长度、存储误差或遮挡强度。 |
| `gamma` | 浮点数 | `1.0` | 归一化块分数的幂指数，有限且非负。 |
| `time` | 浮点数 | `100.0` | 有限值 0～100。SADMask 用于投影，OcclusionMask 用于区间长度；VectorLengthMask 只校验、不参与计算。 |
| `scval` | 浮点数 | `0.0` | 矢量不可用或场景拒绝时的有限填充值。整数输出按 trunc(scval+0.5) 舍入并须在样本范围内；浮点输出直接使用。 |
| `thscd1` | 整数 | `400` | 场景判断的逐块误差阈值，整数 0～16320，按块面积、精度和色度缩放。 |
| `thscd2` | 浮点数 | `51.0` | 坏块百分比，有限值 0～100；坏块数严格超过该比例时拒绝参考场。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |

## 输入要求

只读取 vectors 属性；载体像素和参考图像不参与计算。创建时要求有效分析元数据，须保留所选前缀。

## 返回值与属性

返回 GRAY 剪辑，尺寸取 AnalysisRealWidth/Height，精度取 AnalysisBitsPerSample，帧数/帧率取 vectors。属性仅有 `_Range=1`，不复制输入属性。场不可用时填充 scval。AviSynth 传递第一个输入的音频。

## 最短示例

替换插件路径即可运行；示例使用空白源，不需要外部视频读取插件，所选输出可直接取帧。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neomv.Super(clip, blksize=8, overlap=4, pad=32)
v = core.neomv.AnalyseMany(s, radius=1, badrange=0)
result = core.neomv.OcclusionMask(v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_OcclusionMask(v[0])
return result
```

## 限制与常见错误

参数无效、完整数组损坏或有效描述发生变化时报错。不检查 n+delta 是否存在，完整外部矢量场可在边界生成蒙版。VectorLengthMask 的 gamma=0 也会将零矢量映射为最大值；OcclusionMask 的 gamma=0 只作用于实际事件。

## 计算原理

公式、舍入及数值示例见 [OcclusionMask 计算原理](../../knowledge/zh-CN/occlusion-mask.md)。

[API 目录](README.md)
