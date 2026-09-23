# Flow

将块矢量网格展开后进行逐像素运动补偿。

## 调用方式

VapourSynth：`core.neo_mv.Flow`；AviSynth：`neo_mv_Flow`。参数顺序：

```text
Flow(clip, super, vectors [, time, fields, thscd1, thscd2, tff, prefix])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `super` | 剪辑 | 必填 | 携带辅助图像的 Super 剪辑；前缀须与生成时一致。 |
| `vectors` | 剪辑 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `time` | 浮点数 | `100.0` | 参考位移的百分比，有限值 0～100；不是图像混合比例。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |
| `thscd1` | 整数 | `400` | 场景判断的逐块误差阈值，整数 0～16320，按块面积、精度和色度缩放。 |
| `thscd2` | 浮点数 | `51.0` | 坏块百分比，有限值 0～100；坏块数严格超过该比例时拒绝参考场。 |
| `tff` | 布尔 | 省略 | 显式指定第 0 帧是否为顶场，随后按帧号交替；省略时读取所需帧的 `_Field` 属性。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |

## 输入要求

`clip` 与 Super 须具有相同的可见尺寸、样本类型/位深、颜色家族、色度比例和帧数。每个矢量载体的帧数须不少于 clip，其可见像素不参与计算。处理支持整数 8～16 位或 float32 GRAY/YUV，具体格式还受宿主限制。

Analysis 的可见/工作尺寸、边距和 pel 须与 Super 匹配；块网格须覆盖可见图像，且不超出工作图像。参与处理的色度平面要求块与重叠对齐。分析精度可以不同于渲染精度，场景和误差阈值仍按分析精度缩放。须保留 Super/Analysis 属性并使用一致前缀；辅助剪辑的帧率不必与 clip 相同。

## 返回值与属性

返回单个剪辑，尺寸、格式、帧数、帧率与 `clip` 相同，保留源帧属性。AviSynth 从第一个输入传递音频和场序。 参考缺失或被拒绝时复制 clip，不输出稠密矢量场。

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
result = core.neo_mv.Flow(clip, s, v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_Flow(clip, s, v[0])
return result
```

## 限制与常见错误

fields=true 允许 pel=1，此时没有场偏移。亚像素精度且 delta 为奇数时要求所需场序。time=0 不免除校验；非法实际采样位置或四分之一相位边界会报错而非钳位。允许 delta=0。

## 计算原理

公式、舍入及数值示例见 [Flow 计算原理](../../knowledge/zh-CN/flow.md)。

[API 目录](README.md)
