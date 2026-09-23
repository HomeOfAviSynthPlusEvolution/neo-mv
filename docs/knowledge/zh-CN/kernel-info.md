# KernelInfo：查询当前计算后端

## 1. 这个函数计算什么

`KernelInfo()` 返回当前选择的普通计算后端、指令集目标及 FFT 配置。它没有视频输入，不计算像素、运动或性能分数。

## 2. 计算对象与记号

VapourSynth 调用 `core.neomv.KernelInfo()`，返回四个字段：

| 字段 | 含义 |
| --- | --- |
| `backend` | `scalar` 或 `highway` |
| `target` | 普通计算核实际选中的目标名称；标量时为 `scalar` |
| `fft` | 当前公开选择下为 `pocketfft-scalar` 或 `pocketfft-native` |
| `fft_lanes` | FFT 后端的单精度向量通道数；标量为 1 |

三个名称是字符串数据，通道数是整数。通道数不代表线程数、CPU 核心数或性能倍数。

AviSynth 调用 `neo_mv_KernelInfo()`，返回顺序为 `[backend, target, fft, fft_lanes]` 的原生数组，下标 0–3 对应上表四个字段。

## 3. 总体计算流程

1. 确定普通计算核选择。
2. 查询该计算核的目标名称。
3. 由普通后端确定 FFT 使用标量或 native 配置。
4. 查询 FFT 名称与通道数，组成返回值。

## 4. 逐步计算

普通计算核的选择读取环境变量 `NEO_MV_KERNEL`：值为 `scalar` 时选标量，值为 `highway` 时要求构建包含 Highway。未设置时，有 Highway 的构建默认选择它，否则选标量。其他字符串，包括空字符串，均报错。名称大小写敏感。

该选择在首次成功求值后保存在进程中的静态状态；之后调用查询不会重新读环境变量来切换后端。`target` 则报告所选后端的目标名称。

标量普通后端对应标量 FFT。Highway 普通后端请求 native FFT：在支持相应构建的 x86 上，依次尝试运行环境支持的 AVX-512、AVX2、SSE2 FFT，每级还须实际具备多通道实现；其他目标使用所构建的 native 实现。无可用向量实现时回退标量。

native FFT 最终通道数大于 1 时，名字返回 `pocketfft-native`；否则返回 `pocketfft-scalar`。因此 `backend=highway` 与 `fft=pocketfft-scalar` 可以同时出现。普通核的 `target` 也不应被解释为 FFT 自己的精确指令集标签。

## 5. 一个完整的数值例子

在计算后端首次初始化前设置 `NEO_MV_KERNEL=scalar`，再调用查询，返回字段为：

```text
backend   = scalar
target    = scalar
fft       = pocketfft-scalar
fft_lanes = 1
```

这表明普通核和 FFT 都走标量配置，不表示只能请求一帧或只能由一个宿主工作线程运行。

## 6. 参数与计算步骤对照

此函数没有公开参数。`NEO_MV_KERNEL` 是进程环境配置，并非函数参数；查询不能替换已载入的构建，也不能启用构建时未包含的后端。

## 7. 边界、缺失数据与错误

环境变量取值不合法，或显式要求当前构建没有的 Highway，会产生错误。调用不依赖任何视频帧，也不触发 FFT 图像分析。返回值写入失败同样是错误。

## 8. 数值精度与结果确定性

本函数只查询名称和整数，没有浮点图像运算。它有助于解释 [DepanEstimate](depan-estimate.md) 等函数使用的计算配置，但不证明不同配置之间逐位相同。固定构建与运行环境中的查询结果反映同一选择状态。

[返回中文目录](README.md)
