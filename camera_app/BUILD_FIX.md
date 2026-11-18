# `cmake --build build/ -j` 构建故障记录

## 1. 复现的现象

在本机直接执行 `cmake --build build/ -j` 时，`trtyolo` 目标失败，主要报错为：

```text
No rule to make target '/tmp/radar-trt/lib/libnvinfer.so', needed by 'lib/libtrtyolo.so'.
fatal error: NvInferPlugin.h: No such file or directory
fatal error: NvInferRuntime.h: No such file or directory
fatal error: NvInfer.h: No such file or directory
```

报错出现时其他并行目标仍在输出进度，因此看到的百分比（例如 81%）不是定位依据；失败目标和第一条依赖错误才是关键。

随后删除 `build/` 并直接运行 `cmake -B build`，又复现了配置错误：

```text
Could NOT find TensorRT (missing: TensorRT_INCLUDE_DIRS
TensorRT_LIBRARY_NVINFER TensorRT_LIBRARY_NVINFER_PLUGIN
TensorRT_LIBRARY_NVONNXPARSER)
```

这说明上一次依靠 `-D` 参数配置成功，只修好了那个构建目录；删除缓存后，普通配置仍不知道本机 TensorRT 的安装位置。

## 2. 排查过程与根因

1. 查看 `build/CMakeCache.txt`，发现 `TensorRT_INCLUDE_DIRS` 和三个 `TensorRT_LIBRARY_*` 均指向 `/tmp/radar-trt`。
2. 检查文件系统，`/tmp/radar-trt` 已不存在；系统中也没有通过 apt 安装的 TensorRT 开发包。这解释了链接目标缺失和所有 `NvInfer*.h` 报错。
3. 项目原来的 `trt/FindTensorRT.cmake` 使用 `find_path`/`find_library`。CMake 会信任已经写入缓存的值，即使对应文件后来被删除；原来的 `find_package_handle_standard_args` 也只看变量是否有值，没有验证这些路径存在。因此配置阶段曾成功，构建阶段才失败。
4. 本机另有 TensorRT 11.4 开发包，而项目 README 指定 TensorRT 10.x；同时已有 `tensorrt_cu13_libs` 10.16.1.11 的运行库。为使头文件与链接库版本一致，这次使用 TensorRT 10.16.1.11 的官方头文件包和现有的 10.16.1.11 运行库。
5. 上一次补齐的头文件在 `$HOME/.local/opt/radar-tensorrt-10.16.1`，运行库在 Miniforge 环境中；原查找器只搜索 `TensorRT_ROOT=/usr` 和 CUDA 路径。全新构建目录没有手工 `-D` 参数，因此无法发现它们。

## 3. 实施的修复

- 修改 `trt/FindTensorRT.cmake`：配置时删除指向不存在的缓存路径；搜索用户目录下的 `radar-tensorrt-*` 头文件；从头文件读取 TensorRT 主版本，再在 Miniforge/当前 Conda 环境中查找相同主版本的 `libnvinfer.so`、插件和 ONNX 解析器版本化库。三个库均为必需项，仍找不到时会在配置阶段明确失败。
- 从 NVIDIA Ubuntu 24.04 仓库取得与运行库同版本的 TensorRT 头文件包，解压到 `$HOME/.local/opt/radar-tensorrt-10.16.1`，没有修改系统包或项目源码中的第三方头文件。
- 现在不需要额外 `-D` 参数，直接 `cmake -B build` 就能在本机自动找到这些文件。

本机使用的依赖路径：

```text
头文件：$HOME/.local/opt/radar-tensorrt-10.16.1/usr/include/x86_64-linux-gnu
运行库：$HOME/miniforge3/envs/cud128/lib/python3.12/site-packages/tensorrt_libs
```

下面的命令可在具有相同 Python 环境的本机重做依赖准备，并从空目录配置、构建：

```bash
TRT_HOME="$HOME/.local/opt/radar-tensorrt-10.16.1"
TRT_LIB_DIR="$HOME/miniforge3/envs/cud128/lib/python3.12/site-packages/tensorrt_libs"
NVIDIA_REPO="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64"
mkdir -p "$TRT_HOME/debs"
for pkg in \
  'libnvinfer-headers-dev_10.16.1.11-1+cuda13.2_amd64.deb' \
  'libnvinfer-headers-plugin-dev_10.16.1.11-1+cuda13.2_amd64.deb' \
  'libnvonnxparsers-dev_10.16.1.11-1+cuda13.2_amd64.deb'; do
  curl -fL --retry 3 -o "$TRT_HOME/debs/$pkg" "$NVIDIA_REPO/$pkg"
  dpkg-deb -x "$TRT_HOME/debs/$pkg" "$TRT_HOME"
done

test -f "$TRT_HOME/usr/include/x86_64-linux-gnu/NvInfer.h"
test -f "$TRT_LIB_DIR/libnvinfer.so.10"
test -f "$TRT_LIB_DIR/libnvinfer_plugin.so.10"
test -f "$TRT_LIB_DIR/libnvonnxparser.so.10"

rm -rf build/
cmake -B build
cmake --build build/ -j
```

如果换机器或 Python 环境，应使用该机器上**同一 TensorRT 版本**的头文件与库；默认搜索不到时，可以通过 `TensorRT_ROOT` 或原有的四个 `-DTensorRT_*` 参数显式指定路径。不要把构建缓存固定到可能被清理的 `/tmp/radar-trt`。

## 4. 验证结果

- 在没有旧缓存的独立目录 `/tmp/radar-station-fresh-config` 中执行 `cmake -S . -B /tmp/radar-station-fresh-config` 成功，缓存自动指向本机 TensorRT 10.16 头文件与库。
- 在另一个新目录中故意传入已失效的 `/tmp/radar-trt` 路径，配置仍成功，并把缓存改为实际存在的 10.16 路径。
- 用户清空过的 `build/` 在上述配置错误后重新运行 `cmake -B build` 成功，显示 `Found TensorRT`；随后执行原命令 `cmake --build build/ -j`，从空目标目录构建到 `[100%] Built target radar`，退出码为 0。
- `ldd build/bin/radar` 找到了 `libnvinfer.so.10`、`libnvinfer_plugin.so.10`、`libnvonnxparser.so.10`，没有 `not found`。
- `ctest --test-dir build -N` 显示项目当前没有注册测试。构建中仍有弃用 API 与一般编译器警告，但没有编译或链接错误；本次未验证相机、模型和 GPU 推理运行。

## 5. 后续注意事项

当前全新构建的 `CMAKE_CUDA_ARCHITECTURES=52`，而本机 RTX 5060 Laptop 的计算能力是 12.0。编译通过不等于 CUDA 内核运行已经验证；若后续实际推理报告设备架构或 kernel image 错误，应再检查 CUDA 工具链和目标架构。这与本次缺失 TensorRT 文件导致的构建失败是两件事。
