# Ubuntu 26.04 Distrobox 容器 GPU 加速方案推荐配置与失败总结

## 核心结论与云电脑全环境配置清单

配置双系统难度较大，普通硬件难以达到GUI流畅运行要求。可选方案如下：

>**1.已经配置好环境的使用老款高性能硬件的ubuntu20系统云电脑**

>**2.已经配置好nvidia库等环境的20.04镜像使用新硬件的其他操作系统（搭建较复杂）**

根据quick-start.md的完整运行要求，结合 Ubuntu 20.04 的硬件兼容性限制，以下清单列出了云电脑上需要部署的全部环境组件及其推荐版本。

### ✅ 云电脑完整环境配置清单（Ubuntu 20.04 兼容版）

| 环境组件 | 推荐版本/规格 | 关键约束与说明 |
| :--- | :--- | :--- |
| **操作系统** | **Ubuntu 20.04.6 LTS** | 严格匹配比赛要求； |
| **GPU 型号** | **NVIDIA Tesla T4 (16GB)** 或 **RTX 3090 (24GB)** |  Ubuntu 20.04 内核 5.4 **对较新 GPU（如 RTX 40 系列及以上）支持不佳** |
| **NVIDIA 驱动** | **535.x 系列**（LTS 稳定版） | 确保驱动版本 ≥ CUDA Toolkit 所需最低版本（如 CUDA 12.4 需 ≥ 525.60.13） |
| **CUDA Toolkit** | **11.8**（最稳妥）或 **12.1/12.4** | **强烈不建议** CUDA 13.x；必须与 libtorch 版本严格对应 |
| **LibTorch (C++ GPU 版)** | 如`libtorch-cxx11-abi-shared-with-deps-*+cu118.zip` | 与 CUDA 版本完全匹配 |
| **cuDNN** | 与 CUDA 版本匹配（如 CUDA 11.8 搭配 cuDNN 8.x） | 云平台预装或通过 NVIDIA 官方源安装 |
| **NCCL / cuSPARSELt / NVSHMEM** | 与 CUDA 版本匹配 | 云平台预装可免去手动下载，这是选择云电脑的关键优势 |
| **ROS** | **ROS Noetic (ros-noetic-desktop-full)** | 必须使用 Ubuntu 20.04 官方源安装 |
| **Gazebo** | **Gazebo Classic 11.x**（随 ROS Noetic 一并安装） | 比赛指定版本 |
| **Python** | **Python 3.8.10**（系统自带） | 也可通过 conda 安装 3.10，但需确保不影响系统 ROS 依赖 |
| **CMake** | **≥ 3.18**（推荐 3.22+） | 满足新版 libtorch 的构建需求；Ubuntu 20.04 默认 3.16.3，需手动升级 |
| **LCM** | **1.3.1**（或最新稳定版） | 用于 Unitree 控制器通信 |
| **其他 Python 包** | `python3-yaml`、`numpy` | 用于评估脚本和场景生成 |
| **(如果选的话)云平台镜像选择** | 推荐预装 **CUDA + cuDNN + NCCL + Python** 的深度学习镜像（如 AutoDL、阿里云 AI 镜像） | 一键获得全部 GPU 依赖，大幅降低搭建时间 |

### 📋 部署后验证步骤（云电脑上）

```bash
# 1. 检查 GPU 与驱动支持的最高 CUDA 版本
nvidia-smi

# 2. 确认 CUDA Toolkit 版本
nvcc -V

# 3. 安装 ROS Noetic（如镜像未预装）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt install ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init && rosdep update

# 4. 安装其他依赖
sudo apt install python3-yaml python3-numpy cmake liblcm-dev

# 5. 升级 CMake（如版本不足）
sudo apt remove cmake   # 或使用源码编译安装 3.22+

# 6. 下载与 CUDA 匹配的 libtorch，解压至固定路径（如 ~/libtorch）
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.x.x%2Bcu118.zip
unzip libtorch-cxx11-abi-shared-with-deps-*.zip -d ~/

# 7. 修改 SimEnv 中 CMakeLists.txt 的 libtorch 路径（按实际路径调整）
# 8. 编译
source /opt/ros/noetic/setup.bash
catkin_make -j

# 9. 运行测试（无 GUI 模式快速验证）
GUI=false ./auto.sh
```

### 🗺️ 行动路线图

| 阶段 | 任务 |
| :--- | :--- |
| **目前** | 继续使用 CPU 版 libtorch 或 wsl 完成开发 |
| **云电脑就绪后** | 按上述清单选购实例（Ubuntu 20.04 或 预装好镜像的其他环境），一次性安装全部环境，编译并测试 GPU 推理 |
| **验证通过后** | 切换到 GPU 版libtroch |


---
---

## 1. 本地失败环境概览

### 1.1 宿主机环境
| 项目 | 详情 |
| :--- | :--- |
| 操作系统 | Ubuntu 26.04 |
| GPU | NVIDIA GeForce RTX 5060 Laptop (8GB VRAM) |
| NVIDIA 驱动 | 595.84 |
| CUDA 版本 | 13.2.86 |

### 1.2 容器环境
| 项目 | 详情 |
| :--- | :--- |
| 容器工具 | Distrobox（基于 Podman） |
| 容器内 OS | Ubuntu 20.04.6 LTS |

### 1.3 关键软件依赖状态
| 软件 | 版本 | 状态 |
| :--- | :--- | :--- |
| ROS Noetic + Gazebo Classic | 11.15.1 | ✅ 已安装 |
| libtorch (CPU) | 2.4.0+cpu | ✅ 编译通过、运行可用 |
| libtorch (GPU) | 2.7.1+cu128 / 2.13.0+cu132 | ❌ 目前均失败 |
| cuDNN | 9.10.2 | ✅ 已装 |
| NCCL / cuSPARSELt / NVSHMEM | — | ❌ 无法通过 apt 安装，后续繁琐 |

---

## 2. GPU 方案尝试历程（简述）

### 阶段一：libtorch 2.7.1+cu128（CUDA 12.8）
- 手动创建 `CUDA::nvToolsExt` 目标后编译通过。
- **运行结果**：启动 1-2 分钟后卡死，桌面环境注销。
- **原因**：宿主机 CUDA 13.2 与预期 12.8 不兼容，显存分配异常。

### 阶段二：libtorch 2.13.0+cu132（CUDA 13.2）
- **编译失败**：链接时缺失 `libcusparseLt.so.0`、`libnccl.so.2`、`libnvshmem_host.so.3`。
- **安装复杂**：上述库无法通过 apt 安装。新libtorch依赖的NVIDIA库需从 NVIDIA 官网手动注册下载，后续繁琐。
- **最终结果**：编译无法完成。

---

## 3. 失败根因分析

| 问题 | 说明 |
| :--- | :--- |
| **依赖库获取障碍** | cuSPARSELt、NCCL、NVSHMEM 无法通过 Ubuntu 20.04 默认源安装。 |
| **CUDA 版本不匹配** | 宿主机 13.2 vs libtorch 12.8，运行时异常。 |
| **显存不足** | 8GB VRAM 不足以支撑 Gazebo + 点云 + 神经网络推理。 |
| **容器限制** | Distrobox 无法完全透传所有 GPU 设备节点。 |

---

## 4. 总结

| 维度 | 结论 |
| :--- | :--- |
| **双系统当前状态** | CPU 版可用；GPU 版因依赖链与显存问题无法在本地 Distrobox 稳定部署。 |
| **推荐路径** | 本地仅作 CPU 版 libtorch 或 wsl 开发。后续按上述完整清单**配置云电脑**或**利用预装镜像**运行。 |


---

2026-08-05 KianDu