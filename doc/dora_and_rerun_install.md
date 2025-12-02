整个安装过程在conda环境下执行的

### 1. 安装miniconda3

安装依赖项
```bash
sudo apt  install cargo clang
pip install dora-rs==0.3.9
```



https://repo.anaconda.com/miniconda/Miniconda3-py312_25.9.1-1-Linux-aarch64.sh

下载之后在对应目录的终端下输入下面指令进行安装：

```bash
bash Miniconda3-py312_25.9.1-1-Linux-aarch64.sh
```

安装完成后先source一下环境，让conda生效

```bash
source ~/.bashrc
```

然后创建一个dora的环境

```bash
conda create -n dora python=3.12
```

接着激活环境

```bash
conda activate dora
```

这样conda环境就创建好了

### 2. 安装DORA（在dora环境下）
#### 2.1  安装DORA
先安装命令行

```bash
pip install dora-rs-cli
```

接着把主仓库克隆下来，用于编译C/C++链接库

```bash
git clone https://github.com/dora-rs/dora.git
```

要是速度太慢，可以从自己上克隆下来后再传到板子上。

先进入到c库的文件夹中

```bash
cd dora/apis/c/node
```

可以看到，这里只有一个头文件，没有链接库。接着就是要使用cargo来编译生成对应的链接库，但是在操作之前先对相关的工具链进行处理

先将系统可能预装的Rust的东西给删掉

```bash
sudo apt remove --purge rustc cargo
sudo apt autoremove --purge
```

接着确认是否删干净，为空则没问题。

```bash
which rustc
which cargo
```

安装rustup：

```bash
export RUSTUP_DIST_SERVER=https://mirrors.tuna.tsinghua.edu.cn/rustup
export RUSTUP_UPDATE_ROOT=https://mirrors.tuna.tsinghua.edu.cn/rustup/rustup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

最后就是Cargo编译：

```bash
CARGO_REGISTRY=https://mirrors.tuna.tsinghua.edu.cn/git/crates.io-index.git cargo build --release
```

在执行完成之后，可以在dora/target/release目录下看到对应的静态链接库了，结合头文件，就可以在C/C++程序中使用了，记住这里编译的是c库，在cpp文件中引用头文件的时候，要显性标明是c库

```cpp
extern "C"{
#include "node_api.h"
}
```
#### 2.2 源码编译安装
若以上安装方式失效可以尝试从源码编译安装

参考连接：https://blog.csdn.net/crp997576280/article/details/135368894（Dora-rs 机器人框架学习教程（1）—— Dora-rs安装）

```bash
mkdir dora_project && cd dora_project

export DORA_VERSION=v0.3.9 # Check for the latest release
export ARCHITECTURE=$(uname -m)
wget https://github.com/dora-rs/dora/releases/download/${DORA_VERSION}/dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
unzip dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
pip install dora-rs==${DORA_VERSION} ## For Python API
export PATH=$PATH:$(pwd) >> ~/.bashrc
dora --help
```

完成上述步骤后把 PATH=$PATH:/home/xxx/dora_project  (例如：PATH="$PATH:/home/crp/dora_project" )加入到 .bashrc中最后一行

#### 2.3  API接口库

原文链接： https://blog.csdn.net/weixin_44112228/article/details/135607575 （Dora-rs 机器人框架学习教程（2）——从零开始编译C++节点）

编译c/c++ API接口库

```bash
cd ~/dora/examples/c++-dataflow
cargo run --example cxx-dataflow  # compile C++ node
cargo build -p dora-node-api-c --release  # compile dora-node-api-c 
```

编译ROS2的接口（有需要用到ROS2编译这个接口）

```
cd ../c++-ros2-dataflow
source /opt/ros/galactic/setup.bash
cargo run --example cxx-ros2-dataflow --features ros2-examples
```



### 3. 安装rerun（在dora环境下）

同样的，先激活dora环境

```bash
conda activate dora
```

下载rerun_cpp_sdk：https://github.com/rerun-io/rerun/releases/download/0.26.2/rerun_cpp_sdk.zip，然后解压出来，在终端通过conda包来安装rerun_sdk：

```bash
conda install -c conda-forge librerun-sdk rerun-sdk
```

接着在conda环境中安装对应的libstc++：

```bash
conda install -c conda-forge gcc_linux-aarch64=12 gxx_linux-aarch64=12
```

然后就可以在代码中添加rerun相关组件了

 
