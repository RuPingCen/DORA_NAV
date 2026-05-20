# NavigationFramework

基于 [Dora-rs](https://github.com/dora-rs/dora) 框架的移动机器人自主导航系统，支持激光雷达定位、全局路径规划、局部动态避障和实时可视化。

## 系统架构

本项目采用模块化设计，通过 Dora 数据流框架实现各模块间的解耦通信：

```
传感器层 → 定位层 → 规划层 → 执行层
   ↓         ↓        ↓        ↓
 Livox → HDL定位 → A*+DWA/pure_pursuit → 底盘控制
   ↓                   ↓
 IMU               Rerun可视化
```

### 核心模块

- **驱动模块** (`modules/drivers`)
  - `livox_driver`: Livox MID360 激光雷达驱动，支持点云和 IMU数据发布

- **定位模块** (`modules/localization`)
  - `hdl_localization`: 基于 NDT 的激光雷达定位，融合 IMU 数据

- **规划模块** (`modules/planner`)
  - `global_planner/astar_planner`: A* 全局路径规划器，支持路径平滑
  - `local_planner/pure_pursuit`: 纯跟踪路径跟踪算法
  - `local_planner/dwa_planner`: DWA 动态窗口法，支持动态避障和路径跟踪

- **底盘模块** (`modules/chassis`)
  - `dora_mickrobot`: mickrobot 底盘驱动，支持速度控制和里程计发布

- **可视化模块** (`modules/visualization`)
  - `rerun_visualizer`: 基于 Rerun 的实时 3D 可视化，显示点云地图、机器人位姿、全局路径和 DWA 预测轨迹

- **接口模块** (`modules/interface`)
  - `goal_publisher`: 目标点发布节点

### 工具

- **地图转换工具** (`tools/map_trans`)
  - PCD 点云地图转 PGM 栅格地图

## 快速开始

### 环境要求

- **操作系统**: Linux (x86_64 / ARM64)
- **编译器**: GCC 9+ / Clang 10+
- **CMake**: 3.10+
- **Python**: 3.8+ (用于 Dora CLI)

### 依赖安装

详细的环境配置请参考 [INSTALL.md](INSTALL.md)，主要依赖包括：

- Dora-rs (数据流框架)
- PCL 1.7+ (点云处理)
- Eigen3 (线性代数)
- Rerun SDK (可视化)
- Livox-SDK2 (激光雷达驱动)
- ndt_omp (NDT 配准加速)
- serial (串口通信)
- nlohmann_json (JSON 解析)
- yaml-cpp (配置文件解析)

### 编译

```bash
# 克隆仓库
git clone <repository_url>
cd NavigationFramework

# 编译第三方库（详见 INSTALL.md）
# 编译 Livox-SDK2, ndt_omp, serial 等

# 编译项目
mkdir build && cd build
cmake .. && make -j$(nproc)
```

### 运行

```bash
# 在项目根目录下运行
dora run apps/run.yml
# 发布测试目标点
cd modules/interface/goal_publisher && python3 test_udp_sender.py
```

运行后系统将启动完整的导航流程：
1. Livox 雷达采集点云和 IMU 数据
2. HDL 定位节点输出机器人位姿
3. 目标点发布后（目前是通过一个python脚本手动发布测试），A* 规划全局路径
4. DWA 局部规划器跟踪路径并动态避障或者纯跟踪算法用于单纯的轨迹跟踪
5. 底盘执行速度指令
6. Rerun 实时可视化所有数据

### 配置文件

- `apps/run.yml`: 完整导航系统配置
- `apps/mapping.yml`: 建图模式配置（目前尚未完善）
- `apps/test.yml`: 测试配置

各模块的参数配置文件位于对应模块的 `config/` 目录下。

## 项目结构

```
NavigationFramework/
├── apps/                   # Dora 应用配置文件
├── modules/                # 功能模块
│   ├── drivers/           # 传感器驱动
│   ├── localization/      # 定位模块
│   ├── planner/           # 规划模块
│   │   ├── global_planner/
│   │   └── local_planner/
│   ├── chassis/           # 底盘驱动
│   ├── visualization/     # 可视化
│   └── interface/         # 接口模块
├── tools/                  # 离线工具
│   └── map_trans/         # 地图转换工具
├── third_party/           # 第三方库
├── maps/                  # 地图文件
├── docs/                  # 文档
├── CMakeLists.txt         # 根 CMake 配置
├── INSTALL.md             # 安装指南
├── CHANGELOG.md           # 更新日志
└── README.md              # 本文件
```

### 编译选项

可以通过 CMake 选项控制模块编译：

```bash
cmake .. \
  -DBUILD_LIVOX_DRIVER=ON \
  -DBUILD_LOCALIZATION=ON \
  -DBUILD_GLOBAL_PLANNER=ON \
  -DBUILD_LOCAL_PLANNER=ON \
  -DBUILD_VISUALIZATION=ON
```

### 添加新模块

1. 在 `modules/` 下创建模块目录
2. 编写 CMakeLists.txt，链接 `${DORA_NODE_API_LIB}`
3. 在根 CMakeLists.txt 中添加 `add_subdirectory()`
4. 在 `apps/*.yml` 中配置数据流连接
