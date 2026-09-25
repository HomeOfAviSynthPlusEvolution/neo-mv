# DepanAnalyse

根据已有块矢量拟合相邻帧全局运动。

这里的块误差指存储的 `AnalysisSAD`，可能包含 SAD、SATD、DCT 或混合亮度误差，以及启用的色度 SAD。本函数不重算像素 SAD，也不在度量间换算；现有阈值和缩放公式直接作用于存储值。见[公共分析数据](../../knowledge/zh-CN/shared/analysis-data.md)。

## 调用方式

VapourSynth：`core.neo_mv.DepanAnalyse`；AviSynth：`neo_mv_DepanAnalyse`。参数顺序：

```text
DepanAnalyse(clip, vectors [, mask, zoom, rot, pixaspect, error, info, wrong, zerow, thscd1, thscd2, fields, tff])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `vectors` | 剪辑 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `mask` | 剪辑 | 省略 | 可选整数 8 位蒙版，尺寸与 clip 相同，帧数不少于 clip；第一平面提供拟合权重。 |
| `zoom` | 布尔 | `true` | 允许拟合全局缩放。 |
| `rot` | 布尔 | `true` | 允许拟合全局旋转。 |
| `pixaspect` | 浮点数 | `1.0` | 有限正像素宽高比；场模式会调整有效纵向比例。 |
| `error` | 浮点数 | `15.0` | 有限的拟合误差控制值，用于初始化和接受判断。 |
| `info` | 布尔 | `false` | 写入诊断文本并使用宿主绘制；AviSynth 使用 propShow。 |
| `wrong` | 浮点数 | `10.0` | 有限阈值，用于剔除与邻域不一致的运动观测。 |
| `zerow` | 浮点数 | `0.05` | 零位移观测的有限权重倍数；不限定在 0～1。 |
| `thscd1` | 整数 | `400` | 场景判断的逐块误差阈值，整数 0～16320，按块面积、精度和色度缩放。 |
| `thscd2` | 浮点数 | `51.0` | 坏块百分比，有限值 0～100；坏块数严格超过该比例时拒绝参考场。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |
| `tff` | 布尔 | 省略 | 显式指定第 0 帧是否为顶场，随后按帧号交替；省略时读取所需帧的 `_Field` 属性。 |

## 输入要求

clip 为固定视频载体，其像素不参与拟合，因此可为 RGB/浮点。vectors 须携带默认前缀 `MVUtensils` 的有效元数据，delta 为 ±1，帧数不少于 clip。+1 方向输出 n 读取 vectors[max(0,n−1)]，−1 方向读取 vectors[n]。可选 mask 须匹配 clip 尺寸并提供足够整数 8 位帧。

## 返回值与属性

写入 `Depan_dx`、`Depan_dy`（像素位移）、`Depan_rot`（角度）、`Depan_zoom`（缩放）和 `Depan_goodmotion`（0/1）。这些名称没有可配置前缀。goodmotion=0 表示估计不可用，不等于零运动。除显示选项绘制内容外，保留可见剪辑和时间轴。

info=true 时诊断属性为 `DepanAnalyse_info`，文本绘制可能修改可见像素。

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
result = core.neo_mv.DepanAnalyse(clip, v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_DepanAnalyse(clip, v[0])
return result
```

## 限制与常见错误

没有 prefix 参数。控制值非有限、宽高比无效、缺失所需场序、矢量损坏、mask 几何错误或数值失败都会报错。场不可用可产生无效运动，但不会隐藏缺失的必需 mask 帧。

## 计算原理

公式、舍入及数值示例见 [DepanAnalyse 计算原理](../../knowledge/zh-CN/depan-analyse.md)。

[API 目录](README.md)
