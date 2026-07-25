# 环境版本说明

## 开发与运行环境

### 操作系统
- **Ubuntu**: 20.04.6 LTS (Focal Fossa)
- **架构**: x86_64 / amd64

### ROS 环境
- **ROS 版本**: Noetic Ninjemys 1.16.0
- **构建系统**: catkin_make
- **Python**: 3.8.10 (ROS Noetic 默认)
- **Gazebo**: Gazebo Classic 11.11.0

### 核心依赖包

| 包名 | 版本 | 用途 | 安装方式 |
|------|------|------|----------|
| rospy | 1.16.0 | ROS Python 客户端 | apt: ros-noetic-rospy |
| sensor_msgs | 1.13.1 | 传感器消息 | apt: ros-noetic-sensor-msgs |
| nav_msgs | 1.14.1 | 导航消息 | apt: ros-noetic-nav-msgs |
| geometry_msgs | 0.7.3 | 几何消息 | apt: ros-noetic-geometry-msgs |
| cv_bridge | 1.15.0 | OpenCV 桥接 | apt: ros-noetic-cv-bridge |
| tf2_ros | 0.7.6 | TF2 坐标变换 | apt: ros-noetic-tf2-ros |
| image_geometry | 1.16.2 | 相机模型 | apt: ros-noetic-image-geometry |
| std_srvs | 1.13.1 | 标准服务 | apt: ros-noetic-std-srvs |

### Python 依赖

| 库 | 版本 | 用途 |
|----|------|------|
| numpy | >= 1.17 | 数值计算 |
| opencv-python | 4.2.x | 图像处理（系统包） |
| PyYAML | 5.3.1 | 配置文件解析 |

### 仿真环境（SimEnv 官方）

| 组件 | 版本/说明 |
|------|-----------|
| SimEnv | 官方最新版（Gitee: guoyulun/SimEnv） |
| Unitree A1 控制器 | junior_ctrl (RL 控制器) |
| building_generator | 多楼层建筑生成器 |
| Livox Mid-360 仿真 | 点云仿真 |
| RealSense D415 仿真 | RGB + Depth 仿真 |

### 控制器模式

| 模式编号 | 功能 |
|----------|------|
| 2 | 站立 |
| 4 | RL 键盘行走 |
| 6 | RL /cmd_vel 模式（比赛使用） |
| 8 | 复位 |

## 硬件要求（推荐）

- **CPU**: 8 核以上（仿真 + 算法并行）
- **内存**: 16GB 以上
- **GPU**: NVIDIA GPU（CUDA 支持，libtorch 依赖）
- **显存**: 4GB 以上
- **硬盘**: 20GB 以上可用空间

## CUDA 与 libtorch

SimEnv 的 Unitree 控制器依赖 libtorch（C++ 版 PyTorch）：
- **CUDA**: >= 11.7
- **libtorch**: 与 CUDA 版本匹配的 C++ 版本
- 参考 SimEnv 官方文档中的环境配置步骤

## 编译与运行

### 编译命令
```bash
cd ~/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make -j$(nproc)
source ./devel/setup.bash
```

### 环境检查脚本
```bash
# 检查 ROS 版本
rosversion -d

# 检查 Python 版本
python3 --version

# 检查 Gazebo 版本
gazebo --version

# 检查核心包
rospack find danger_search_common
```

## 版本管理

### Git 仓库建议

```
main          # 稳定可运行版本
develop       # 开发分支
feature/xxx   # 各模块功能分支
  - feature/perception
  - feature/localization
  - feature/navigation
  - feature/exploration
```

### 提交规范

```
feat: 新功能
fix: 修复bug
docs: 文档更新
refactor: 重构
test: 测试相关
chore: 构建/工具链
```

## 已知兼容性说明

1. **ROS1 Noetic 仅支持 Python3**，所有 Python 脚本使用 `#!/usr/bin/env python3`
2. **Gazebo Classic 11** 是 ROS Noetic 的配套版本，不要升级到 Gazebo Ignition
3. **Livox 点云格式特殊**，原始话题是 `sensor_msgs/PointCloud`，需要转换为 `PointCloud2` 使用
4. **坐标系对齐**：确保所有模块使用统一的 map 坐标系（出发点为原点）
