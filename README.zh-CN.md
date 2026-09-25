# neo-mv

[English](README.md) | **简体中文** | [日本語](README.ja.md)

neo-mv 是 VapourSynth 和 AviSynth 的运动处理插件，提供块运动估计、运动补偿、时域混合、运动遮罩、帧插值，以及全局运动估计与稳像，接口以 MVUtensils 为基础。

内部使用 C++17，提供标量内核、基于 Google Highway 的跨平台 SIMD，以及用于频域运动估计的 PocketFFT。DualSynth2 将计算核心连接到两个宿主。VapourSynth 使用 `core.neo_mv`，AviSynth 使用带有 `neo_mv_` 前缀的函数。

## 设计

neo-mv 将运动计算与宿主帧管理分离。核心处理图像平面、运动场和显式几何信息，宿主层负责剪辑、帧请求、属性及输出分配。核心可以独立构建和测试。

实现依据行为规格开发，明确规定搜索顺序、舍入、边界及运动不可用时的处理。标量实现提供计算基准，SIMD 路径按各自的数值规则验证。函数名称相同，不代表与所有历史 MVTools 或 MVUtensils 版本的输出完全一致。

## 支持的操作

| 类别 | 函数 |
|---|---|
| 工作图像 | `Super`：图像金字塔、边界扩展及亚像素相位。 |
| 块运动 | `Analyse`、`AnalyseMany`、`Recalculate`、`SCDetection`：运动搜索、多时间距离分析、向量细化及场景标记。 |
| 块渲染 | `Compensate`、`Degrain`、`Degrain1`–`Degrain25`：运动补偿采样及加权时域合成。 |
| 运动遮罩 | `VectorLengthMask`、`SADMask`、`OcclusionMask`：向量长度、块误差及遮挡图。 |
| 逐像素运动 | `Flow`、`FlowInter`、`FlowFPS`、`FlowBlur`：稠密位移、插值、帧率转换及轨迹平均。 |
| 全局运动 | `DepanAnalyse`、`DepanEstimate`、`DepanCompensate`、`DepanStabilise`：运动拟合、FFT 相关、几何补偿及稳像。 |
| 诊断 | `KernelInfo`：查询选用的计算及 FFT 后端。 |

块运动和 Flow 系列支持平面 GRAY/YUV 的 8–16 位整数及 32 位浮点样本，具体格式和子采样限制因函数而异。`DepanAnalyse`、`DepanCompensate` 和 `DepanStabilise` 使用整数图像；`DepanEstimate` 也接受 float32。不支持 RGB。

`Analyse`、`AnalyseMany` 和 `Recalculate` 通过 `metric="sad"`（默认）、`"satd"` 或 `"dct"` 选择亮度匹配度量。SATD 要求块宽、高均能被 4 整除；DCT 支持全部合法块尺寸，包括 6×6 和 16×2，但仅接受 8–16 位整数。色度始终使用 SAD。该字符串参数替代旧的 `satd` 布尔参数。误差阈值保留原有缩放规则，不在不同度量之间自动换算。

仅整数的混合模式 `sad_dct_global`、`sad_dct_local`、`sad_satd_global`、`sad_satd_local` 和 `sad_satd_global_half` 支持帧对自适应权重，或可调的局部 `metric_weight` / `metric_threshold`。参见 [API 迁移表](docs/api/zh-CN/analyse.md#从-mvtools-的-dct-参数迁移)。既有纯模式的数值定义不变。

运动数据保存在帧属性中。默认属性前缀为 `MVUtensils`，与插件命名空间 `neo_mv` 相互独立。Super 的辅助图像属于生成它的实现，供 neo-mv 使用的 Super 应由 neo-mv 生成。

## 文档与使用

计算知识库逐个解释函数如何将输入变成输出，包括数据表示、公式、运算顺序、参数作用、数值示例、边界及精度。

- [API 使用参考](docs/api/zh-CN/README.md)：函数签名、参数表、默认值及使用示例。
- [知识库](docs/knowledge/zh-CN/README.md)

可以显式加载构建出的插件，也可以将其放入 VapourSynth 的插件自动加载目录。下面使用 Windows 文件名，其他平台请替换为实际插件路径。

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")

print(core.neo_mv.KernelInfo())
clip = core.std.BlankClip(width=640, height=360, format=vs.YUV420P8, length=24)
super_clip = core.neo_mv.Super(clip, blksize=16, overlap=8, pad=32, pel=2)
vectors = core.neo_mv.Analyse(super_clip, delta=1)
output = core.neo_mv.Compensate(clip, super_clip, vectors)
output.set_output()
```

这个最小示例使用合成剪辑展示调用关系。`Super` 必须显式指定块大小和重叠量。正 `delta` 引用后面的帧，负 `delta` 引用前面的帧。完整参数和计算过程见对应函数文章。

同一个插件文件也提供 AviSynth C++ 接口。使用 AviSynth+ 3.7.4 或更新版本（接口 11），或兼容的 AviSynthMinus 运行时，通过 `LoadPlugin` 加载：

```avs
LoadPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=640, height=360, length=24, pixel_type="YV12")
super_clip = neo_mv_Super(clip, blksize=16, overlap=8, pad=32, pel=2)
vectors = neo_mv_AnalyseMany(super_clip, radius=1)
return neo_mv_Degrain1(clip, super_clip, vectors)
```

函数名和参数顺序对应 API 使用参考，函数名前加 `neo_mv_`。数组参数接受 `[16, 8]` 这样的原生数组，单值可作为一个元素的简写。`AnalyseMany` 和 `Recalculate` 返回剪辑数组，单个 Recalculate 输入也返回一个元素的数组。`neo_mv_KernelInfo()` 返回 `[backend, target, fft, fft_lanes]`。布尔参数使用 `true`/`false`。音频和场序从第一个输入剪辑传递；场模式计算使用 `_Field` 属性或显式 `tff`。Depan 的 `info=true` 使用 AviSynth 的 `propShow` 绘制诊断属性。

## SIMD 与 CPU 选择

启用 SIMD 的构建会选择当前 CPU 支持且已编译的 Highway 目标，并保留标量回退。在首次初始化后端之前设置 `NEO_MV_KERNEL=scalar`，可使用标量内核和标量 FFT；`NEO_MV_KERNEL=highway` 显式要求启用 SIMD 的构建。不设置该变量时使用构建的默认后端。

首次成功初始化后会缓存选择结果，此后修改环境变量不会为已有或新建滤镜切换后端。`core.neo_mv.KernelInfo()` 返回 `backend`、`target`、`fft` 和 `fft_lanes`；FFT 目标可能与通用内核目标不同。通道数不是线程数，也不是加速倍数。

FFT 配置和允许的浮点差异可能影响结果，更宽的 SIMD 不保证吞吐量更高。详见 [KernelInfo](docs/knowledge/zh-CN/kernel-info.md) 及各函数的精度章节。

`fft` 和 `fft_lanes` 描述 DePan 使用的 PocketFFT，不代表块 DCT 变换。DCT 随选定后端使用标量或 Highway 内核，系数量化规则相同。

## 构建与测试

需要 CMake 3.24 或更新版本、Git 及支持 C++17 的编译器。CMake 获取固定版本的 DualSynth2、PocketFFT、Boost.Multiprecision 和 Boost.Config，启用 SIMD 时还会获取 Highway 1.4.0。两个宿主的 SDK 均可从本地发现或自动下载。VapourSynth 测试需要架构匹配的运行时和 `vspipe`；AviSynth 测试需要架构匹配的运行时库。

```sh
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/release --config Release --parallel 4
ctest --test-dir build/release -C Release --output-on-failure
```

| 选项 | 用途 |
|---|---|
| `NEO_MV_BUILD_VAPOURSYNTH=OFF` | 禁用 VapourSynth 入口。 |
| `NEO_MV_BUILD_AVISYNTH=OFF` | 禁用 AviSynth 入口；两个宿主选项均为 OFF 时只构建核心。 |
| `NEO_MV_ENABLE_SIMD=OFF` | 禁用 SIMD 内核和向量化 FFT 配置。 |
| `BUILD_TESTING=OFF` | 不构建测试。 |
| `NEO_MV_TEST_VAPOURSYNTH=OFF` | 保留核心测试和插件，跳过宿主测试。 |
| `NEO_MV_VSPIPE_EXECUTABLE=/path/to/vspipe` | 指定宿主测试使用的运行时。 |
| `NEO_MV_VS_SDK=/path/to/sdk` | 指定本地 VapourSynth SDK。 |
| `NEO_MV_AVS_SDK=/path/to/sdk` | 指定本地 AviSynth SDK。 |
| `NEO_MV_TEST_AVISYNTH=ON` | 启用 AviSynth 宿主测试，默认关闭。 |
| `NEO_MV_AVISYNTH_RUNTIME=/path/to/avisynth.dll` | 指定 AviSynth 宿主测试使用的运行时库。 |
| `FETCHCONTENT_SOURCE_DIR_DUALSYNTH2=/path/to/dualsynth2` | 使用本地 DualSynth2 源码代替固定版本下载。 |
| `NEO_MV_BUILD_BENCHMARKS=ON` | 构建手动运行的内核基准测试，需要启用 SIMD。 |

插件目标保留名称 `neo_mv_vs`，输出文件基本名为 `neo-mv`，默认包含两个宿主入口。测试覆盖算术、运动搜索、图像采样、属性校验、标量/SIMD 比较、边界及宿主行为。另有与 MVUtensils 比较公开行为的黑盒测试，需要单独安装参考实现。

CI 配置包含 Windows x64、Linux x64、macOS ARM64 及 Linux ASan/UBSan 检查。发布工作流构建 Windows、Linux 和 macOS 的 x64/ARM64 版本，其中 VapourSynth 宿主测试目前在 Windows x64 上运行，AviSynth 运行时测试通过上述选项单独启用。

## 性能

这些历史测量不包含新增的 DCT 度量。

已有测量中，各条已测滤镜路径的吞吐量约为 **MVUtensils R9 的 0.62–1.83 倍**。比值为 **neo-mv 吞吐量 / MVUtensils 吞吐量**，等价于 MVUtensils 耗时 / neo-mv 耗时；**大于 1 表示 neo-mv 更快**。下列范围跨越 8/16 位和 AVX2/AVX-512 配置，不是置信区间。

| 函数 | 相对吞吐量 |
|---|---:|
| Super | 1.11–1.44× |
| Analyse | 0.71–0.81× |
| Recalculate | 0.75–0.84× |
| Compensate | 0.80–1.03× |
| Degrain1 | 0.88–1.19× |
| Degrain2 | 0.94–1.38× |
| VectorLengthMask | 1.17–1.83× |
| SADMask | 0.84–1.56× |
| OcclusionMask | 0.81–1.08× |
| Flow | 1.26–1.59× |
| FlowInter | 1.07–1.58× |
| FlowFPS | 1.03–1.67× |
| FlowBlur | 1.30–1.35× |
| DepanAnalyse | 0.73–0.93× |
| DepanCompensate （双线性） | 0.79–0.94× |
| DepanCompensate （双三次） | 1.15–1.49× |
| DepanEstimate | 0.62–0.81× |
| DepanStabilise | 1.06–1.42× |

这些范围来自 VapourSynth 中固定 1080p YUV420P8/P16 输入下的单线程独立滤镜测量。上游图像预先生成，因此不代表整条处理链的吞吐量。实际结果随输入、参数、硬件和线程数变化。

## 开发与贡献

维护者负责技术方向、变更审核和发布。欢迎问题报告、建议与贡献；修改数值语义、公开接口或重要架构前，建议先讨论目标和方案。

本项目使用 AI 辅助实现、测试和审查。贡献应说明问题、方案、验证方法和 AI 参与方式。报告问题请提供版本、系统、CPU、编译器、构建选项、输入输出格式及最小复现；性能报告还应包含尺寸、后端选择和测量方法。

## 致谢与许可证

感谢以下上游项目的作者与贡献者，他们的工作为 neo-mv 的接口与运动处理功能提供了基础：

- [MVUtensils](https://github.com/myrsloik/mvutensils)：neo-mv 的 API 与行为参考，也是性能对比对象。
- [VapourSynth-MVTools](https://github.com/dubhatervapoursynth/vapoursynth-mvtools)：MVUtensils 的前身，将 AviSynth MVTools 移植到 VapourSynth。
- [MVTools / MVTools2](https://github.com/pinterf/mvtools)：块运动估计、运动补偿及相关滤镜的历史来源。
- [DePan / DePanEstimate](https://github.com/pinterf/mvtools)：全局运动估计、补偿与稳定功能的历史来源。

neo-mv 还使用了以下计算库：

- [Google Highway](https://github.com/google/highway)：提供跨平台 SIMD 支持。
- [PocketFFT](https://github.com/mreineck/pocketfft)：用于 `DepanEstimate` 的 FFT 相关计算；块 DCT 匹配使用 neo-mv 自身的变换实现。
- [Boost.Multiprecision](https://github.com/boostorg/multiprecision) 和 [Boost.Config](https://github.com/boostorg/config)：为 DCT 舍入边界的整数区间细化及可移植宽整数提供仅头文件支持，采用 BSL-1.0 许可证；不需要 Boost 运行时库。

感谢参与测试、报告问题和改进的开发者与用户。

感谢 [烧饼论坛](https://sb.sb) 赞助本项目开发使用的 LLM 订阅。

neo-mv 采用 GNU 通用公共许可证第 2 版或更新版本（`GPL-2.0-or-later`），完整条款见 [LICENSE](LICENSE)。第三方组件保留各自的版权声明和许可条款。
