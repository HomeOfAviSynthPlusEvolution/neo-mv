# KernelInfo

查询已选用的通用计算内核及 FFT 配置。函数不接收剪辑，也不处理视频。

## 调用方式

```text
core.neo_mv.KernelInfo()
neo_mv_KernelInfo()
```

## 参数

无。

## 返回值

VapourSynth 返回含下列键的映射（字符串数据可能以 Python bytes 返回）。AviSynth 返回原生数组 `[backend, target, fft, fft_lanes]`。

| 键 | AVS 索引 | 含义 |
| --- | --- | --- |
| `backend` | 0 | `scalar` 或 `highway`。 |
| `target` | 1 | 通用内核目标名称；标量执行为 `scalar`。 |
| `fft` | 2 | `pocketfft-scalar` 或 `pocketfft-native`。 |
| `fft_lanes` | 3 | FFT 单精度向量通道数，整数；标量为 1。 |

FFT 目标可能不同于通用内核目标。通道数不是线程数，也不是预计加速倍数。

## 后端选择与错误

在首次后端初始化前设置 `NEO_MV_KERNEL`。小写 `scalar` 选择标量，`highway` 要求构建包含 Highway。未设置时优先使用构建中的 Highway，否则使用标量。其他值（包括空字符串）会报错。

进程缓存首次成功选择的结果，之后修改环境变量不会切换后端。查询不能启用已加载构建中不存在的后端。

## 最短示例

替换插件路径即可运行。空白剪辑仅用于为脚本提供视频输出。

### VapourSynth

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")
print(core.neo_mv.KernelInfo())
core.std.BlankClip(width=64, height=48, length=12, format=vs.YUV420P8).set_output()
```

### AviSynth

```avs
LoadPlugin("/path/to/neo-mv.dll")
k = neo_mv_KernelInfo()
Assert(k[3] >= 1)
return BlankClip(width=64, height=48, length=12, pixel_type="YV12")
```

## 计算原理

详见 [KernelInfo 后端选择过程](../../knowledge/zh-CN/kernel-info.md)。

[API 目录](README.md)
