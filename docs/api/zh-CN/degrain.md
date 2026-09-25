# Degrain / Degrain1–Degrain25

按匹配误差加权合成当前图像与运动补偿参考；带编号的入口固定矢量对数。

这里的块误差指存储的 `AnalysisSAD`，可能包含 SAD、SATD、DCT 或混合亮度误差，以及启用的色度 SAD。本函数不重算像素 SAD，也不在度量间换算；现有阈值和缩放公式直接作用于存储值。见[公共分析数据](../../knowledge/zh-CN/shared/analysis-data.md)。

## 调用方式

VapourSynth：`core.neo_mv.Degrain`；AviSynth：`neo_mv_Degrain`。参数顺序：

```text
Degrain(clip, super, vectors [, thsad, thsad2, planes, limit, thscd1, thscd2, weights, prefix])
```

此处方括号表示可选参数，不是数组字面量。可选参数建议按名称传入；Python 布尔值写作 `True`/`False`，AviSynth 写作 `true`/`false`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 可见源剪辑；尺寸、格式固定，帧数非零。 |
| `super` | 剪辑 | 必填 | 携带辅助图像的 Super 剪辑；前缀须与生成时一致。 |
| `vectors` | 剪辑数组 | 必填 | 携带运动分析属性的剪辑；具体要求见下文。 |
| `thsad` | 整数数组 | `[400, 400]` | 最近矢量对的非负阈值 [亮度, 色度]，按采样精度和块几何缩放。 |
| `thsad2` | 整数数组 | `thsad` | 最远矢量对的非负阈值 [亮度, 色度]；默认取有效 thsad。 |
| `planes` | 整数数组 | 全部平面 | 平面索引 0、1、2；省略或空数组处理全部实际平面。重复索引报错；格式中不存在的合法索引不处理。 |
| `limit` | 浮点数组 | `[+inf, +inf]` | 相对 clip 的最大变化量，使用原样本单位 [亮度, 色度]。转为 float32 后有限值须为正，非有限值关闭限制。 |
| `thscd1` | 整数 | `400` | 场景判断的逐块误差阈值，整数 0～16320，按块面积、精度和色度缩放。 |
| `thscd2` | 浮点数 | `51.0` | 坏块百分比，有限值 0～100；坏块数严格超过该比例时拒绝参考场。 |
| `weights` | 整数数组 | 2R+1 个 1 | 用户系数顺序为第一侧由远到近、中心、第二侧由近到远；恰好 2R+1 项，默认全 1，显式空数组无效。 |
| `prefix` | 字符串 | `"MVUtensils"` | 属性名前缀，生成和读取数据时须一致。允许空字符串，不允许 NUL。 |

### 矢量对、权重及编号入口

每个相邻矢量对的 delta 须非零、绝对值相同且符号相反。各对的距离须严格递增；函数不会替你排序。`AnalyseMany` 生成通常的 `[+1,-1,+2,-2,…]` 顺序，每对内部也允许负方向在前。

`thsad`、`thsad2`、`limit` 可传单值/单元素（亮度色度相同）或两个元素 [亮度, 色度]；空数组使用默认值。U、V 共用色度设置。阈值缩放后须可表示；limit 使用实际像素单位，不是统一的 8 位尺度。

R=2 时，`weights=[a,b,c,d,e]` 将 c 用于中心，b/d 用于矢量成员 0/1，a/e 用于成员 2/3。每项先饱和为 int32，再要求处于 `0..floor(2147483646 / (256*(2R+1)))`。中心权重不在第一项。

`Degrain` 从矢量数量推导 R；`DegrainR` 要求恰好 2R 个成员，其余参数一致：

| 入口 | 矢量成员数 |
| --- | --- |
| `Degrain1` / `neo_mv_Degrain1` | 2 |
| `Degrain2` / `neo_mv_Degrain2` | 4 |
| `Degrain3` / `neo_mv_Degrain3` | 6 |
| `Degrain4` / `neo_mv_Degrain4` | 8 |
| `Degrain5` / `neo_mv_Degrain5` | 10 |
| `Degrain6` / `neo_mv_Degrain6` | 12 |
| `Degrain7` / `neo_mv_Degrain7` | 14 |
| `Degrain8` / `neo_mv_Degrain8` | 16 |
| `Degrain9` / `neo_mv_Degrain9` | 18 |
| `Degrain10` / `neo_mv_Degrain10` | 20 |
| `Degrain11` / `neo_mv_Degrain11` | 22 |
| `Degrain12` / `neo_mv_Degrain12` | 24 |
| `Degrain13` / `neo_mv_Degrain13` | 26 |
| `Degrain14` / `neo_mv_Degrain14` | 28 |
| `Degrain15` / `neo_mv_Degrain15` | 30 |
| `Degrain16` / `neo_mv_Degrain16` | 32 |
| `Degrain17` / `neo_mv_Degrain17` | 34 |
| `Degrain18` / `neo_mv_Degrain18` | 36 |
| `Degrain19` / `neo_mv_Degrain19` | 38 |
| `Degrain20` / `neo_mv_Degrain20` | 40 |
| `Degrain21` / `neo_mv_Degrain21` | 42 |
| `Degrain22` / `neo_mv_Degrain22` | 44 |
| `Degrain23` / `neo_mv_Degrain23` | 46 |
| `Degrain24` / `neo_mv_Degrain24` | 48 |
| `Degrain25` / `neo_mv_Degrain25` | 50 |

## 输入要求

`clip` 与 Super 须具有相同的可见尺寸、样本类型/位深、颜色家族、色度比例和帧数。每个矢量载体的帧数须不少于 clip，其可见像素不参与计算。处理支持整数 8～16 位或 float32 GRAY/YUV，具体格式还受宿主限制。

Analysis 的可见/工作尺寸、边距和 pel 须与 Super 匹配；块网格须覆盖可见图像，且不超出工作图像。参与处理的色度平面要求块与重叠对齐。分析精度可以不同于渲染精度，场景和误差阈值仍按分析精度缩放。须保留 Super/Analysis 属性并使用一致前缀；辅助剪辑的帧率不必与 clip 相同。

## 返回值与属性

返回单个剪辑，尺寸、格式、帧数、帧率与 `clip` 相同，保留源帧属性。AviSynth 从第一个输入传递音频和场序。 未选择的平面复制 clip。不可用或被场景判断拒绝的参考有效权重为零，当前帧获得剩余权重。

## 最短示例

替换插件路径即可运行；示例使用空白源，不需要外部视频读取插件，所选输出可直接取帧。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, fpsnum=24, format=vs.YUV420P8)
s = core.neo_mv.Super(clip, blksize=8, overlap=4, pad=32)
v = core.neo_mv.AnalyseMany(s, radius=2, badrange=0)
result = core.neo_mv.Degrain2(clip, s, v)
result.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, fps=24, pixel_type="YV12")
s = neo_mv_Super(clip, blksize=8, overlap=4, pad=32)
v = neo_mv_AnalyseMany(s, radius=2, badrange=0)
result = neo_mv_Degrain2(clip, s, v)
return result
```

## 限制与常见错误

要求 1～25 对（2～50 个）矢量剪辑。用户权重为零也会校验该成员。描述无效、完整数组损坏或 Super 采样支持不足时报错。本函数没有 fields、time、search、metric 参数。

## 计算原理

公式、舍入及数值示例见 [Degrain 计算原理](../../knowledge/zh-CN/degrain.md)。

[API 目录](README.md)
