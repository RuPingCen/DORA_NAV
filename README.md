# DORA_NAV

DORA_NAV是一个DORA环境下的开源导航框架，适用于差速、全向、Ackerman类型的移动机器人底盘。该框架集成了常用的传感器驱动、建图、定位、感知（地面去除、聚类、目标跟踪）、规划（纯跟踪、A*、tare_planner）、控制等算法包。可提供A点到B点的自主导航接口。

**目录说明**：在“~”目录下存放dora 运行文件 及 DORA_NAV文件夹

# 1  安装dora V0.3.2

## 1.1 安装依赖项
```bash
sudo apt  install cargo clang
pip install dora-rs==0.3.2
```
## 1.2 安装 dora V0.3.2   
参考连接：https://blog.csdn.net/crp997576280/article/details/135368894（Dora-rs 机器人框架学习教程（1）—— Dora-rs安装）

```bash
mkdir dora_project && cd dora_project

export DORA_VERSION=v0.3.2 # Check for the latest release
export ARCHITECTURE=$(uname -m)
wget https://github.com/dora-rs/dora/releases/download/${DORA_VERSION}/dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
unzip dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
pip install dora-rs==${DORA_VERSION} ## For Python API
export PATH=$PATH:$(pwd) >> ~/.bashrc
dora --help
```

完成上述步骤后把 PATH=$PATH:/home/xxx/dora_project  (例如：PATH="$PATH:/home/crp/dora_project" )加入到 .bashrc中最后一行


## 1.3下载 dora 源码       

原文链接： https://blog.csdn.net/weixin_44112228/article/details/135607575 （Dora-rs 机器人框架学习教程（2）——从零开始编译C++节点）


```bash
cd ~
git clone https://github.com/dora-rs/dora.git
cd dora/examples/c++-dataflow
cargo run --example cxx-dataflow  # compile C++ node
cargo build -p dora-node-api-c --release  # compile dora-node-api-c 

cd ../c++-ros2-dataflow
source /opt/ros/galactic/setup.bash
cargo run --example cxx-ros2-dataflow --features ros2-examples
```
# 2.下载 DORA_NAV

下载DORA_NAV master分支

```bash
cd ~
git clone https://github.com/RuPingCen/DORA_NAV.git
```

#  Demo Video
