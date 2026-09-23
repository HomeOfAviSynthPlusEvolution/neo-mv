# SCDetection

根据存储的块误差附加场景切换标记。

## 调用方式

VapourSynth：`core.neomv.SCDetection`；AviSynth：`neo_mv_SCDetection`。参数顺序：

```text
SCDetection(clip, vectors [, thscd1, thscd2, prefix])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `vectors` | 剪辑 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `thscd1` | 整数 | `400` | 场景判断的逐块误差阈值，整数 0～16320，按块面积、精度和色度缩放。 |
| `thscd2` | 浮点数 | `51.0` | 坏块百分比，有限值 0～100；坏块数严格超过该比例时拒绝参考场。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |

## 输入要求

clip 提供可见输出，vectors 提供分析属性。创建时要求有效元数据，但不要求第 0 帧已有完整数组。vectors 须能提供请求的帧号。

## 返回值与属性

返回单个剪辑，尺寸、格式、帧数、帧率与 `clip` 相同，保留源帧属性。AviSynth 从第一个输入传递音频和场序。 delta 为负时设置 `_SceneChangePrev`，否则设置 `_SceneChangeNext`，值为 0 或 1；另一方向的键保留。数据缺失或不可用时为 1。

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
result = core.neomv.SCDetection(clip, v[0])
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=1, badrange=0)
result = neo_mv_SCDetection(clip, v[0])
return result
```

## 限制与常见错误

thscd1、thscd2 须满足范围。完整但损坏的矢量/误差或有效几何发生变化时报错。不检查参考帧是否存在；thscd2=100 也不会抑制缺失数据产生的标记。

## 计算原理

公式、舍入及数值示例见 [SCDetection 计算原理](../../knowledge/zh-CN/sc-detection.md)。

[API 目录](README.md)
