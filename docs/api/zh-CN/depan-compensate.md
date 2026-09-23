# DepanCompensate

按整数或小数帧偏移应用全局运动补偿。

## 调用方式

VapourSynth：`core.neo_mv.DepanCompensate`；AviSynth：`neo_mv_DepanCompensate`。参数顺序：

```text
DepanCompensate(clip, data [, offset, subpixel, pixaspect, matchfields, mirror, blur, info, fields, tff])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `data` | 剪辑 | 必填 | 携带 Depan 运动属性的剪辑，帧数至少与 clip 相同；尺寸、格式、帧率不要求相同。 |
| `offset` | 浮点数 | `0.0` | 转为 float32 后为有限值 −10～10；正数选择更早的源帧，负数选择更晚的源帧；0 绕过补偿。 |
| `subpixel` | 整数 | `2` | 重采样方式：0 最近邻，1 双线性，2 双三次。 |
| `pixaspect` | 浮点数 | `1.0` | 有限正像素宽高比；场模式会调整有效纵向比例。 |
| `matchfields` | 布尔 | `true` | fields=true 时匹配目标场序。 |
| `mirror` | 整数 | `0` | 边缘反射位掩码 0～15：上=1、下=2、左=4、右=8，相加组合。 |
| `blur` | 整数 | `0` | 反射边缘的水平平均范围，非负；不是时间运动模糊。 |
| `info` | 布尔 | `false` | 写入诊断文本并使用宿主绘制；AviSynth 使用 propShow。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |
| `tff` | 布尔 | 省略 | 显式指定第 0 帧是否为顶场，随后按帧号交替；省略时读取所需帧的 `_Field` 属性。 |

## 输入要求

clip 接受固定整数 8～16 位 GRAY/YUV420/422/444。data 携带五项 Depan 运动属性，通常来自 DepanEstimate 或 DepanAnalyse；帧数至少与 clip 相同。不支持浮点图像处理。双三次采样要求每个平面至少两行。

## 返回值与属性

返回单个剪辑，尺寸、格式、帧数、帧率与 `clip` 相同，保留源帧属性。AviSynth 从第一个输入传递音频和场序。

info=true 时诊断属性为 `DepanCompensate_info`，文本绘制可能修改可见像素。

## 最短示例

替换插件路径即可运行；示例使用空白源，不需要外部视频读取插件，所选输出可直接取帧。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
data = core.neo_mv.DepanEstimate(clip, winx=32, winy=32)
result = core.neo_mv.DepanCompensate(clip, data, offset=1)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
data = neo_mv_DepanEstimate(clip, winx=32, winy=32)
result = neo_mv_DepanCompensate(clip, data, offset=1)
return result
```

## 限制与常见错误

offset=0 或选定源帧越界时绕过运动采样。无效运动导致绕过；访问帧的属性缺失/损坏或依赖失败会报错。offset=0 仍校验参数和格式。fields 与 matchfields 同时开启时需要目标场序；没有 prefix 参数。

## 计算原理

公式、舍入及数值示例见 [DepanCompensate 计算原理](../../knowledge/zh-CN/depan-compensate.md)。

[API 目录](README.md)
