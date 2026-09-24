# Super

为运动分析和补偿生成图像金字塔、边缘扩展区域及亚像素采样数据。返回一个携带这些辅助数据的剪辑；可见画面保持不变。

## 调用方式

VapourSynth 使用 `core.neo_mv.Super`，AviSynth 使用 `neo_mv_Super`。两个接口的参数顺序相同：

```text
Super(clip, blksize, overlap [, pad, onelevel, sharp, rfilter, pel, pelclip, prefix])
```

这里的方括号表示可选参数，不是要写入脚本的数组。前三项必填；可选参数建议按名称传入。返回值为 VapourSynth 的 `VideoNode` 或 AviSynth 的 `clip`。

## 参数

| 参数 | 类型 | 默认值 | 取值与作用 |
| --- | --- | --- | --- |
| `clip` | 剪辑 | 必填 | 固定尺寸、固定格式、至少一帧的平面 GRAY/YUV。支持整数 8～16 位或 32 位浮点；YUV 每个方向的色度缩小比例只能是 1 或 2。 |
| `blksize` | 整数轴数组 | 必填 | `[块宽, 块高]`，单位为亮度像素。决定工作图像的块覆盖范围和金字塔层数。允许的组合见下文。 |
| `overlap` | 整数轴数组 | 必填 | `[水平重叠, 垂直重叠]`，单位为亮度像素。每项从 0 到对应块尺寸的一半，含端点；须满足色度对齐。 |
| `pad` | 整数轴数组 | `[16, 16]` | `[水平边距, 垂直边距]`，单位为亮度像素，两个值都必须大于 0。水平值用于左右两侧，垂直值用于上下两侧。 |
| `onelevel` | 布尔 | `false` | 为真时只生成第 0 层；否则根据图像、块和边距计算层数。 |
| `sharp` | 整数 | `2` | 内建亚像素插值：`0` 为二点平均，`1` 为四抽头滤波，`2` 为六抽头滤波。只允许 0～2；不是输出画面的锐化开关。 |
| `rfilter` | 整数 | `1` | 金字塔缩小滤波：`0` 为 2×2 平均，`1` 为可分离四抽头滤波，`2` 为可分离六抽头滤波。只允许 0～2。 |
| `pel` | 整数 | `2` | 亚像素精度：`1` 为整像素，`2` 为半像素，`4` 为四分之一像素。只允许 1、2、4。 |
| `pelclip` | 剪辑 | 省略 | 提供外部放大图像，以替代内建的非整数采样相位。格式、尺寸和帧数要求见下文。 |
| `prefix` | 字符串 | `"MVUtensils"` | 写入属性的名称前缀。后续读取此 Super 的函数必须使用相同前缀。允许空字符串，不允许 NUL 字符。 |

VapourSynth 脚本中布尔值写作 `True`/`False`；AviSynth 中写作 `true`/`false`。

### 轴数组与块尺寸

`blksize`、`overlap`、`pad` 都遵循以下规则：

| 传入形式 | 含义 |
| --- | --- |
| `8` 或 `[8]` | 两个轴都取 8。 |
| `[16, 8]` | 水平方向取 16，垂直方向取 8。 |
| `[]` | 使用回退值：`blksize` 为 `[8, 8]`，`overlap` 为 `[0, 0]`，`pad` 为 `[16, 16]`。 |
| 超过两个元素 | 报错。 |

**空数组不等于省略参数。** `blksize` 和 `overlap` 即使存在空数组回退值，调用时仍必须提供。

允许的块尺寸为：`4×4`、`8×4`、`8×8`、`12×12`、`16×2`、`16×8`、`16×16`、`24×24`、`32×16`、`32×32`、`48×48`、`64×32`、`64×64`、`128×64`、`128×128`。宽高顺序不可随意交换。

输入图像的宽、高不得小于对应块尺寸。YUV 图像的尺寸、块尺寸和重叠值须在各轴上被色度缩小比例整除。例如 YUV420 的块宽、块高、水平重叠和垂直重叠都必须为偶数；图像尺寸不要求是块尺寸的整数倍。

### 外部亚像素图像

当提供 `pelclip` 且 `pel>1` 时：

- `pelclip` 必须与 `clip` 的颜色格式、采样类型和位深一致。
- 宽和高分别为原图的 `pel` 倍，帧数与原图相同。
- 输出第 n 帧使用两个输入的第 n 帧；时间对应由调用者保证，不进行帧率转换。
- 整像素相位及粗层仍来自 `clip`；只有非整数相位取自 `pelclip`。`sharp` 不参与外部相位生成，但仍必须在 0～2 内。

`pel=1` 时，传入的 `pelclip` 仍须具有受支持的固定尺寸、非零帧数及相同格式，但不检查放大尺寸关系或与原图帧数相等，也不请求其帧。

## 返回值与帧属性

返回单个剪辑，宽、高、格式、帧数和帧率与 `clip` 一致，可见像素原样传递。AviSynth 还从 `clip` 传递音频和场序。

辅助图像通过帧属性携带。以下名称均需在前面加上 `prefix`；默认情况下，例如 `SuperPel` 的完整名称是 `MVUtensilsSuperPel`。

| 属性后缀 | 内容 |
| --- | --- |
| `SuperWidth`、`SuperHeight` | 第 0 层的工作尺寸，不含四周 padding，可能大于可见尺寸。 |
| `SuperRealWidth`、`SuperRealHeight` | 原图可见尺寸。 |
| `SuperHPad`、`SuperVPad` | 亮度边距。 |
| `SuperPel`、`SuperLevels` | 亚像素精度和金字塔层数。 |
| `SuperChroma`、`SuperXRatioUV`、`SuperYRatioUV` | 是否包含色度，以及色度缩小比例。 |
| `SuperBitsPerSample` | 样本位深。 |
| `SuperBlkSizeX`、`SuperBlkSizeY`、`SuperOverlapX`、`SuperOverlapY` | 块尺寸和重叠值。 |
| `NeoMVSuperDescriptorV1`、`NeoMVSuperPlanesV1` | 后续函数读取的描述数据及辅助帧引用；不要手动修改或删除。 |

同一前缀已有的 Super 数据会被本次结果替换，其他帧属性保留。后续处理若删除这些辅助属性，剪辑仍可能显示正常，但无法再作为有效的 Super 输入。

## 最短示例

将插件路径替换为本机路径。示例使用空白源，保存后即可运行，无需另装视频读取插件。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
clip = core.std.BlankClip(width=64, height=48, length=12, format=vs.YUV420P8)
super_clip = core.neo_mv.Super(clip, blksize=[8, 8], overlap=[4, 4])
super_clip.set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=64, height=48, length=12, pixel_type="YV12")
super_clip = neo_mv_Super(clip, blksize=[8, 8], overlap=[4, 4])
return super_clip
```

两个示例都使用默认的 `pad=[16,16]`、`pel=2`、`sharp=2`、`rfilter=1`。输出仍是原空白画面；辅助数据已附在帧上，可将 `super_clip` 传给 `Analyse` 等函数。

## 限制与常见错误

| 情况 | 结果或修正方式 |
| --- | --- |
| 省略 `blksize` 或 `overlap` | 必填参数错误；显式指定值或空数组。 |
| RGB、带 alpha 的格式或不支持的色度比例 | 格式错误；先转换为受支持的平面 GRAY/YUV。 |
| 块组合不在允许列表中、图像小于块、重叠超过半块或 padding 为 0 | Super 几何参数错误。 |
| 色度轴未对齐 | 几何对齐错误，例如 YUV420 使用奇数重叠。 |
| 内建插值的带边距平面过小 | `sharp=0/1/2` 分别要求各平面的带边距宽、高至少为 2/4/6。 |
| `pelclip` 格式不符，或 `pel>1` 时尺寸、帧数不符 | 创建失败；按外部亚像素图像要求准备输入。 |
| 几何尺寸超出可表示范围、缩小滤波没有足够可读样本或内存不足 | 创建或取帧失败；正的 `pad` 并不保证所有极端几何组合可用。 |

## 计算原理

工作尺寸、层数、滤波系数、边界处理及舍入过程见 [Super 计算原理](../../knowledge/zh-CN/super.md)。

[API 目录](README.md)
