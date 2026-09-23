# FlowFPS

使用运动插值将时间轴转换为目标帧率。

## 调用方式

VapourSynth：`core.neomv.FlowFPS`；AviSynth：`neo_mv_FlowFPS`。参数顺序：

```text
FlowFPS(clip, super, vectors [, num, den, extramask, ml, blend, thscd1, thscd2, prefix])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `super` | 剪辑 | 必填 | 携带辅助图像的 Super 剪辑；前缀须与生成时一致。 |
| `vectors` | 剪辑数组 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `num` | 整数 | `25` | 目标帧率非负分子；num 或 den 任一为 0 时，将原帧率翻倍。 |
| `den` | 整数 | `1` | 目标帧率非负分母；两者均正时输出帧率为 num/den。 |
| `extramask` | 布尔 | `true` | 从同一组矢量剪辑读取额外帧以处理遮挡；false 移除这些额外依赖。 |
| `ml` | 浮点数 | `100.0` | 有限正数，控制运动或遮挡强度的归一化。 |
| `blend` | 布尔 | `true` | 插值运动不可用时混合原始端点图像；false 时复制左端图像。 |
| `thscd1` | 整数 | `400` | 场景判断的逐块误差阈值，整数 0～16320，按块面积、精度和色度缩放。 |
| `thscd2` | 浮点数 | `51.0` | 坏块百分比，有限值 0～100；坏块数严格超过该比例时拒绝参考场。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |

## 输入要求

`clip` 与 Super 须具有相同的可见尺寸、样本类型/位深、颜色家族、色度比例和帧数。每个矢量载体的帧数须不少于 clip，其可见像素不参与计算。处理支持整数 8～16 位或 float32 GRAY/YUV，具体格式还受宿主限制。

Analysis 的可见/工作尺寸、边距和 pel 须与 Super 匹配；块网格须覆盖可见图像，且不超出工作图像。参与处理的色度平面要求块与重叠对齐。分析精度可以不同于渲染精度，场景和误差阈值仍按分析精度缩放。须保留 Super/Analysis 属性并使用一致前缀；辅助剪辑的帧率不必与 clip 相同。 vectors 恰为 `[bw, fw]`，delta 分别为同一正距离 d 的 `+d, -d`。Super 帧数与原 clip 相同，两个矢量载体的帧数均不少于原 clip；除方向及允许的层数差异外，描述须匹配。

## 返回值与属性

保留尺寸/格式，帧率改为约分后的目标分数，帧数为 floor(输入帧数 × 目标帧率 / 原帧率)。属性取所选左端或端点源帧。端点位置复制对应原图；插值不可用时按 blend 混合或复制。AviSynth 传递输入音频，不对音频变速。

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
result = core.neomv.FlowFPS(clip, s, v, num=30, den=1)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_FlowFPS(clip, s, v, num=30, den=1)
return result
```

## 限制与常见错误

要求已知正原帧率、非零且可表示的输出帧数。num/den 为负时无效，即使另一项为 0。辅助输入帧数按原时间轴，不按输出时间轴。extramask=false 不请求额外观测。

## 计算原理

公式、舍入及数值示例见 [FlowFPS 计算原理](../../knowledge/zh-CN/flow-fps.md)。

[API 目录](README.md)
