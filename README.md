# DORA_NAV

DORA_NAV是一个DORA环境下的开源导航框架，适用于差速、全向、Ackerman类型的移动机器人底盘。该框架集成了常用的传感器驱动、建图、定位、感知（地面去除、聚类、目标跟踪）、规划（纯跟踪、A*、tare_planner）、控制等算法包。可提供A点到B点的自主导航接口。

**目录说明**：在“~”目录下存放dora 运行文件 及 DORA_NAV文件夹

# 1  安装dora 

dora 和 rerun的安装教程可参考doc文件夹下的  [dora_and_rerun_install.md](https://github.com/RuPingCen/DORA_NAV/blob/master/doc/dora_and_rerun_install.md)

# 2.使用 DORA_NAV

**step1** 下载DORA_NAV master分支

```bash
cd ~
git clone https://github.com/RuPingCen/DORA_NAV.git
```

**step2：建图**：使用 DORA_NAV\mapping 环境下的建图节点，利用激光雷达进行建图，并导出点云地图

**step3:准备路径文件。**

**step4：定位。** 室内环境下利用 DORA_NAV\localization\dora-hdl_localization 代码包进行定位，提供地图环境下的定位数据

**step5： 运行轨迹跟踪**。运行run.yaml文件实现对路径跟踪（运行时需确认使用的激光雷达型号、底盘型号，对yaml文件中的节点进行调换）
