### arm架构(conda环境)
#### conda

安装conda：

```shell
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh
source ~/.bashrc
conda --version
```

创建dora环境：

```shell
conda create -n dora python=3.12
```

后续操作在dora这个conda环境下进行:

```shell
conda activate dora
```

#### dora

安装dora命令行

```shell
pip install dora-rs-cli
```

编译dora的C链接库

```shell
git clone https://github.com/dora-rs/dora.git
cd dora/apis/c/node
rm -rf ~/.rustup
sudo apt remove rustc cargo -y
sudo apt autoremove -y
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.bashrc
rustc --version
cargo build --release
```

#### rerun

```shell
conda install -c conda-forge librerun-sdk rerun-sdk
conda install -c conda-forge gcc_linux-aarch64=12 gxx_linux-aarch64=12
```

#### vscode

```shell
wget https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64 -O vscode.deb
sudo dpkg -i vscode.deb
#如果报错就修复
sudo apt-get install -f -y
```

#### 安装第三方库

```shell
# json
conda install -c conda-forge nlohmann_json
# eigen
sudo apt install libeigen3-dev
# pcl
conda install -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/ pcl
# yaml
conda install -c conda-forge yaml-cpp
```

还有Livox驱动、ndt_omp算法库、串口库需要单独在third_party里面安装。

#### CH341串口驱动(针对OrangePi)

下载内核源码

```shell
# 方法1 直接去这个网址下载压缩包
https://github.com/orangepi-xunlong/linux-orangepi/tree/orange-pi-6.1-cix
# 方法2 git
git clone https://github.com/orangepi-xunlong/linux-orangepi.git -b orange-pi-6.1-cix
```

准备编译环境

```shell
cd orange-pi-6.1-cix
cp /boot/config-6.1.44-cix .config
make oldconfig
make modules_prepare
```

创建软连接

```shell
sudo ln -s $(pwd) /lib/modules/6.1.44-cix/build
```

下载CH341驱动: [CH341SER.ZIP - 南京沁恒微电子股份有限公司](https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html)

```shell
unzip CH341SER_LINUX.ZIP
cd CH341SER_LINUX
cd driver
make
```

如果编译成功，最后安装驱动(临时)

```shell
sudo insmod ch341.ko
# 查看是否成功
lsmod | grep ch341
```

### x86架构(非conda环境)
#### Dora安装
```shell
pip install dora-rs-cli #安装dora命令行
sudo apt install cargo  #安装cargo
rustup default stable   #编译可能报错，需要安装这个
git clone https://github.com/dora-rs/dora.git   #克隆仓库
cd dora/apis/c/node
cargo build --release   #编译
```
编译完成后可以在dora/target/release下看到libdora_node_api_c.a的链接库，说明编译成功。
#### 第三方库
```shell
# Livox-SDK
cd third_party/Livox-SDK2
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
###########
# ndt_omp
cd third_party/ndt_omp
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
#########
# serial
cd third_party/serial
mkdir build && cd build
cmake .. && make -j${nproc} && sudo make install
# 重启串口
sudo apt remove brltty
sudo systemctl stop brltty
sudo systemctl disable brltty
########
sudo apt install nlohmann-json3-dev #json库
```
#### rerun(源码编译安装)
```shell
#预先在系统中装好apache-arrow
wget https://github.com/apache/arrow/releases/download/apache-arrow-18.0.0/apache-arrow-18.0.0.tar.gz
tar -zxvf apache-arrow-18.0.0.tar.gz
cd apache-arrow-18.0.0/cpp
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DARROW_BUILD_TESTS=OFF \
  -DARROW_BUILD_EXAMPLES=OFF \
  -DARROW_BUILD_BENCHMARKS=OFF \
  -DARROW_FLIGHT=OFF \
  -DARROW_PYTHON=OFF \
  -DARROW_CUDA=OFF
make -j${nproc}
sudo make install
##########################
#接着再安装rerun
wget https://github.com/rerun-io/rerun/releases/download/0.31.2/rerun_cpp_sdk.zip   #下载rerun源码
unzip rerun_cpp_sdk.zip
cd rerun_cpp_sdk
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DRERUN_DOWNLOAD_AND_BUILD_ARROW=OFF
cmake --build build --config Release --target rerun_sdk
sudo cmake --install build
##########################
pip install rerun-sdk #安装命令行
```
#### 编译运行
```shell
mkdir build && cd build
cmake .. && make -j${nproc}
cd ..
dora run apps/xxx.yml #根据所需要的yml配置文件来选择
```
很神奇，arm的控制器编译各种报错，x86的控制器基本没什么报错。