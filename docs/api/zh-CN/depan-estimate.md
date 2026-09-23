# DepanEstimate

使用第一平面的 FFT 相关估计相邻帧平移及可选缩放。

## 调用方式

VapourSynth：`core.neomv.DepanEstimate`；AviSynth：`neo_mv_DepanEstimate`。参数顺序：

```text
DepanEstimate(clip [, trust, winx, winy, wleft, wtop, dxmax, dymax, zoommax, stab, pixaspect, info, show, fields, tff])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `trust` | 浮点数 | `4.0` | 接受运动的可信度阈值，有限值 0～100。 |
| `winx` | 整数 | `0` | 窗口宽度；0 自动选取不超过 8192 的 2 的幂。显式值须非负、为偶数且能放入图像。 |
| `winy` | 整数 | `0` | 窗口高度；0 自动选取不超过 8192 的 2 的幂。显式值须非负且能放入图像。 |
| `wleft` | 整数 | `-1` | 窗口水平起点，负值自动定位。 |
| `wtop` | 整数 | `-1` | 窗口垂直起点，负值垂直居中。 |
| `dxmax` | 整数 | `-1` | 水平峰值搜索半径，负值取有效窗口宽的四分之一；须小于有效窗口宽的一半。 |
| `dymax` | 整数 | `-1` | 垂直峰值搜索半径，负值取窗口高的四分之一；须小于窗口高的一半。 |
| `zoommax` | 浮点数 | `1.0` | 有限的缩放接受控制值；恰为 1 时关闭缩放估计，其他值使用水平分离的两个半宽窗口。 |
| `stab` | 浮点数 | `1.0` | 相关峰选择中的有限位移惩罚；允许负值。 |
| `pixaspect` | 浮点数 | `1.0` | 有限正像素宽高比；场模式会调整有效纵向比例。 |
| `info` | 布尔 | `false` | 写入诊断文本并使用宿主绘制；AviSynth 使用 propShow。 |
| `show` | 布尔 | `false` | 将相关面绘制到可见图像上；仍生成运动属性。 |
| `fields` | 布尔 | `false` | 启用场模式计算；不会自动将交错帧分离成场。 |
| `tff` | 布尔 | 省略 | 显式指定第 0 帧是否为顶场，随后按帧号交替；省略时读取所需帧的 `_Field` 属性。 |

### 自动窗口与显示

起点为负时自动定位；窗口尺寸为 0 时选择能放入图像且不超过 8192 的最大 2 的幂。zoommax≠1 时先将水平宽度减半再定位。自动峰值搜索半径为有效窗口相应尺寸的四分之一。

`show=true` 绘制相关面；`info=true` 写入 `DepanEstimate_info` 并绘制诊断文本。函数删除旧的 `DepanEstimateFFT`、`DepanEstimateFFT2`、`DepanEstimateX`、`DepanEstimateY`、`DepanEstimateZoom`、`DepanEstimateGood`、`DepanEstimateTrust` 属性；这些不是输入。

## 输入要求

固定 GRAY/YUV 整数 8～16 位或 float32 剪辑，只分析第一平面。窗口须位于图像内部；所用样本须有限，整数样本须符合声明位深。

## 返回值与属性

写入 `Depan_dx`、`Depan_dy`（像素位移）、`Depan_rot`（角度）、`Depan_zoom`（缩放）和 `Depan_goodmotion`（0/1）。这些名称没有可配置前缀。goodmotion=0 表示估计不可用，不等于零运动。除显示选项绘制内容外，保留可见剪辑和时间轴。

## 最短示例

替换插件路径即可运行；示例使用空白源，不需要外部视频读取插件，所选输出可直接取帧。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
result = core.neomv.DepanEstimate(clip, winx=32, winy=32)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
result = neo_mv_DepanEstimate(clip, winx=32, winy=32)
return result
```

## 限制与常见错误

有效窗口宽须为正偶数，高须为正。缩放模式会将宽减半，两个分离窗口都须能放入图像。峰值半径须小于有效窗口对应尺寸的一半。缺失所需场序或 FFT 失败会报错，不作为无效运动回退。

## 计算原理

公式、舍入及数值示例见 [DepanEstimate 计算原理](../../knowledge/zh-CN/depan-estimate.md)。

[API 目录](README.md)
