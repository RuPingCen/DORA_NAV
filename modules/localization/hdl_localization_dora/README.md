# HDL Localization Dora

HDL Localization 从 ROS 移植到 Dora 框架的版本。

## 项目概述

这是一个基于 UKF (Unscented Kalman Filter) 和 NDT 扫描匹配的 3D 激光雷达定位系统，已从 ROS 框架移植到 Dora 框架。

### 主要特性

- **UKF 位姿估计**: 16维状态向量，包含位置、速度、姿态和传感器偏差
- **NDT 扫描匹配**: 使用 ndt_omp 进行高效的点云配准
- **IMU 融合**: 支持 IMU 数据融合提高预测精度
- **JSON 通信**: 使用 JSON 格式进行数据序列化和传输
- **Dora 集成**: 与 Livox 驱动无缝集成

## 目录结构

```
hdl_localization_dora/
├── src/                                    # 源代码
│   ├── hdl_localization_dora_node.cpp     # 主节点
│   ├── json_serialization.cpp/h           # JSON 序列化工具
│   ├── config_manager.cpp/h               # 配置管理
│   └── pose_estimator.cpp                 # 位姿估计器
├── include/                                # 头文件
│   ├── hdl_localization/                  # 核心算法头文件
│   └── kkl/alg/                           # UKF 算法
├── config/                                 # 配置文件
│   └── hdl_localization_config.json       # 主配置
├── dora/                                   # Dora SDK
│   ├── include/node_api.h
│   └── lib/libdora_node_api_c.a
├── CMakeLists.txt                          # 构建配置
└── README.md                               # 本文档
```

## 依赖项

### 必需依赖

- **CMake** >= 3.10
- **C++17** 编译器
- **PCL** (Point Cloud Library) >= 1.7
- **Eigen3** >= 3.3
- **OpenMP** (多线程支持)
- **Boost** (system, filesystem)
- **nlohmann/json** (JSON 解析)
- **ndt_omp** (NDT 扫描匹配)

### 可选依赖

- **fast_gicp** (可选的配准方法)

## 构建步骤

### 1. 安装依赖

```bash
# Ubuntu/Debian
sudo apt-get install -y \
    cmake \
    build-essential \
    libeigen3-dev \
    libpcl-dev \
    libboost-all-dev \
    libomp-dev

# 安装 nlohmann/json (如果系统没有)
sudo apt-get install nlohmann-json3-dev
```

### 2. 安装 ndt_omp

```bash
cd /path/to/your/workspace
git clone https://github.com/koide3/ndt_omp.git
cd ndt_omp
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

### 3. 构建 hdl_localization_dora

```bash
cd hdl_localization_dora
mkdir build && cd build
cmake ..
make -j4
```

构建成功后，可执行文件位于 `build/hdl_localization_dora_node`。

## 配置

### 配置文件说明

配置文件位于 `config/hdl_localization_config.json`，主要参数包括：

#### 地图配置
```json
"map": {
  "pcd_file": "maps/map.pcd",           // 全局地图 PCD 文件路径
  "downsample_resolution": 0.1          // 地图下采样分辨率 (米)
}
```

#### 初始位姿
```json
"initial_pose": {
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
}
```

#### IMU 配置
```json
"imu": {
  "enabled": true,                      // 是否启用 IMU
  "invert_acceleration": false,         // 是否反转加速度
  "invert_gyroscope": false,            // 是否反转角速度
  "cool_time_duration": 2.0             // 冷却时间 (秒)
}
```

#### NDT 配置
```json
"ndt": {
  "method": "NDT_OMP",                  // 配准方法
  "resolution": 1.0,                    // NDT 分辨率 (米)
  "neighbor_search_method": "DIRECT7",  // 邻域搜索方法
  "neighbor_search_radius": 2.0,        // 搜索半径 (米)
  "transformation_epsilon": 0.01,       // 变换收敛阈值
  "max_iterations": 30,                 // 最大迭代次数
  "num_threads": 4                      // 线程数
}
```

#### UKF 配置
```json
"ukf": {
  "process_noise": {
    "position": 1.0,
    "velocity": 1.0,
    "orientation": 0.5,
    "acc_bias": 1e-6,
    "gyro_bias": 1e-6
  },
  "measurement_noise": {
    "position": 0.01,
    "orientation": 0.001
  }
}
```

## 使用方法

### 1. 准备地图文件

将全局地图 PCD 文件放置在 `maps/` 目录下，或在配置文件中指定路径。

### 2. 配置 run.yml

在项目根目录的 `run.yml` 中已经配置好了 hdl_localization 节点：

```yaml
nodes:
  - id: livox_driver
    path: livox_ros_driver2/build/livox_dora_driver
    inputs:
      timer: dora/timer/millis/100
    outputs:
      - pointcloud
      - imu

  - id: hdl_localization
    path: hdl_localization_dora/build/hdl_localization_dora_node
    inputs:
      pointcloud: livox_driver/pointcloud
      imu: livox_driver/imu
    outputs:
      - pose
      - aligned_points
```

### 3. 运行系统

```bash
# 在项目根目录运行
dora up
dora start run.yml
```

### 4. 查看输出

定位节点会输出以下话题：

- **pose**: 位姿估计结果 (JSON 格式)
  ```json
  {
    "header": {
      "frame_id": "map",
      "timestamp": 1234567890.123,
      "seq": 12345
    },
    "pose": {
      "position": {"x": 10.5, "y": 20.3, "z": 0.5},
      "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    },
    "twist": {
      "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
    }
  }
  ```

- **aligned_points**: 对齐到地图的点云 (可选，默认关闭)

## 数据流

```
Livox Driver → pointcloud/imu (JSON) → HDL Localization → pose (JSON)
```

## 性能优化

### 1. NDT 参数调优

- **resolution**: 增大可提高速度但降低精度，减小则相反
- **num_threads**: 根据 CPU 核心数调整
- **max_iterations**: 减少可提高速度但可能影响收敛

### 2. 点云下采样

- **scan_matching.downsample_resolution**: 增大可显著提高速度

### 3. 地图下采样

- **map.downsample_resolution**: 对大地图尤其重要

## 故障排查

### 问题 1: 编译错误 - 找不到 ndt_omp

**解决方案**: 确保 ndt_omp 已正确安装，或在 CMakeLists.txt 中指定路径：

```cmake
set(NDT_OMP_DIR /path/to/ndt_omp)
```

### 问题 2: 运行时错误 - 无法加载地图

**解决方案**:
- 检查配置文件中的地图路径是否正确
- 确保 PCD 文件格式正确
- 检查文件权限

### 问题 3: 定位不准确

**解决方案**:
- 检查初始位姿是否合理
- 调整 NDT 分辨率参数
- 调整 UKF 噪声参数
- 确保地图质量良好

### 问题 4: 性能问题

**解决方案**:
- 增大点云下采样分辨率
- 增大地图下采样分辨率
- 增加 NDT 线程数
- 减少 NDT 最大迭代次数

## 与原 ROS 版本的差异

### 主要变化

1. **通信机制**: ROS 话题 → Dora 事件驱动
2. **时间类型**: `ros::Time` → `double` (Unix 时间戳)
3. **数据格式**: ROS 消息 → JSON
4. **坐标变换**: TF → 静态变换 (简化)
5. **参数管理**: ROS 参数服务器 → JSON 配置文件

### 保持不变

1. **核心算法**: UKF、NDT 扫描匹配完全保留
2. **状态向量**: 16维状态向量定义不变
3. **系统模型**: 预测和更新模型不变

## 参考资料

- [原始 hdl_localization](https://github.com/koide3/hdl_localization)
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [Dora 框架](https://github.com/dora-rs/dora)

## 许可证

本项目基于原始 hdl_localization 项目，遵循相同的许可证。

## 贡献

欢迎提交 Issue 和 Pull Request。

## 联系方式

如有问题，请在 GitHub 上提交 Issue。
