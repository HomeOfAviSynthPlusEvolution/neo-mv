# DepanStabilise

平滑全局运动并渲染修正图像，可使用邻帧填补边缘。

## 调用方式

VapourSynth：`core.neomv.DepanStabilise`；AviSynth：`neo_mv_DepanStabilise`。参数顺序：

```text
DepanStabilise(clip, data [, cutoff, damping, initzoom, addzoom, prev, next, mirror, blur, dxmax, dymax, zoommax, rotmax, subpixel, pixaspect, fitlast, tzoom, info, method, fields])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `data` | 剪辑 | 必填 | 携带 Depan 运动属性的剪辑，帧数至少与 clip 相同；尺寸、格式、帧率不要求相同。 |
| `cutoff` | 浮点数 | `1.0` | 有限正平滑截止频率，单位 Hz，控制时间范围和响应。 |
| `damping` | 浮点数 | `0.9` | method=0 的有限惯性阻尼；允许负值。 |
| `initzoom` | 浮点数 | `1.0` | 有限正初始缩放；采样使用其倒数。 |
| `addzoom` | 布尔 | `false` | 启用所选平滑方法的自适应缩放。 |
| `prev` | 整数 | `0` | 允许参与空缺填补的更早帧数，非负。 |
| `next` | 整数 | `0` | 允许参与空缺填补的更晚帧数，非负。 |
| `mirror` | 整数 | `0` | 边缘反射位掩码 0～15：上=1、下=2、左=4、右=8，相加组合。 |
| `blur` | 整数 | `0` | 反射边缘的水平平均范围，非负；不是时间运动模糊。 |
| `dxmax` | 浮点数 | `60.0` | 有限水平惯性修正控制/上限，单位像素；负限制可使整个修正复位。 |
| `dymax` | 浮点数 | `30.0` | 有限垂直惯性修正控制/上限，单位像素；负限制可使整个修正复位。 |
| `zoommax` | 浮点数 | `1.05` | 有限惯性缩放限制；负值可使整个修正复位。 |
| `rotmax` | 浮点数 | `1.0` | 有限惯性旋转限制，单位度；负值可使整个修正复位。 |
| `subpixel` | 整数 | `2` | 重采样方式：0 最近邻，1 双线性，2 双三次。 |
| `pixaspect` | 浮点数 | `1.0` | 有限正像素宽高比；场模式会调整有效纵向比例。 |
| `fitlast` | 整数 | `0` | 正值让末尾若干帧渐变回初始修正，非正值关闭该渐变；用于 method=0。 |
| `tzoom` | 浮点数 | `3.0` | 有限非负自适应缩放时间参数；启用自适应缩放时，0 可能造成无效归一化。 |
| `info` | 布尔 | `false` | 写入诊断文本并使用宿主绘制；AviSynth 使用 propShow。 |
| `method` | 整数 | `0` | 平滑方法：0 惯性，1 对称窗口。方法 1 不使用惯性限制、阻尼或 fitlast。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |

## 输入要求

clip 接受固定整数 8～16 位 GRAY/YUV420/422/444。data 携带五项 Depan 运动属性，通常来自 DepanEstimate 或 DepanAnalyse；帧数至少与 clip 相同。不支持浮点图像处理。双三次采样要求每个平面至少两行。 clip 还须具有已知的正帧率。

## 返回值与属性

返回单个剪辑，尺寸、格式、帧数、帧率与 `clip` 相同，保留源帧属性。AviSynth 从第一个输入传递音频和场序。

info=true 时诊断属性为 `DepanStabilise_info`，文本绘制可能修改可见像素。

## 最短示例

替换插件路径即可运行；示例使用空白源，不需要外部视频读取插件，所选输出可直接取帧。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
data = core.neomv.DepanEstimate(clip, winx=32, winy=32)
result = core.neomv.DepanStabilise(clip, data)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
data = neo_mv_DepanEstimate(clip, winx=32, winy=32)
result = neo_mv_DepanStabilise(clip, data)
return result
```

## 限制与常见错误

浮点控制值转为 float32 后须有限；cutoff/initzoom/pixaspect 必须为正，tzoom、prev、next、blur 非负；method 为 0 或 1。访问到的损坏运动属性会报错；goodmotion=0 表示区间边界。没有 tff 或 prefix 参数。

## 计算原理

公式、舍入及数值示例见 [DepanStabilise 计算原理](../../knowledge/zh-CN/depan-stabilise.md)。

[API 目录](README.md)
