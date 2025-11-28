### 安装rerun（在dora环境下）

同样的，先激活dora环境

```bash
conda activate dora
```

下载rerun_cpp_sdk：https://github.com/rerun-io/rerun/releases/download/0.26.2/rerun_cpp_sdk.zip，然后解压出来，在终端通过conda包来安装rerun_sdk：

```bash
conda install -c conda-forge librerun-sdk rerun-sdk
```

接着在conda环境中安装对应的libstc++，否则可能Cmake编译不过：

```bash
conda install -c conda-forge gcc_linux-aarch64=12 gxx_linux-aarch64=12
```

然后就可以在代码中添加rerun相关组件了
