# Ubuntu 26.04 Distrobox 容器 GPU 加速方案失败总结报告

**日期**: 2026-08-05  
**项目**: SimEnv 仿真环境（Unitree A1 + ROS Noetic + Gazebo Classic）  
**目标**: 在 Distrobox 容器内使用 GPU 版 libtorch 加速控制器推理，解决运行时卡死问题

---

## 1. 环境概览

### 1.1 宿主机环境
| 项目 | 详情 |
|------|------|
| 操作系统 | Ubuntu 26.04（版本过低识别不了较新的硬件） |
| GPU | NVIDIA GeForce RTX 5060 Laptop (8GB VRAM) |
| NVIDIA 驱动 | 595.84 |
| CUDA 版本 | 13.2.86 |
| NVIDIA 驱动支持的最高 CUDA | 13.2 |

### 1.2 容器环境
| 项目 | 详情 |
|------|------|
| 容器工具 | Distrobox（基于 Podman） |
| 容器名称 | Ubuntu-20 |
| 容器内操作系统 | Ubuntu 20.04.6 LTS (Focal) |
| 内核 | 共享宿主机内核 |
| 容器创建命令 | `distrobox create --image ubuntu:20.04 --name Ubuntu-20 --additional-flags "--gpus all --device /dev/dri --volume /usr/local/cuda:/usr/local/cuda:ro"` |
| GPU 设备节点 | `/dev/nvidia*` 已挂载 |
| CUDA 工具包路径 | `/usr/local/cuda`（挂载自宿主机） |

### 1.3 软件依赖
| 软件 | 版本 | 状态 |
|------|------|------|
| ROS | Noetic | ✅ 已安装 |
| Gazebo | Classic 11.15.1 | ✅ 已安装 |
| Python | 3.8.10 | ✅ |
| CMake | 3.16.3 → 3.29.3 | ✅ 已升级 |
| LCM | 1.3.1（apt 安装） | ✅ |
| libtorch (CPU) | 2.4.0+cpu | ✅ 编译通过 |
| libtorch (GPU) | 2.7.1+cu128 → 2.13.0+cu132 | ❌ 编译失败 |
| cuDNN | 9.10.2（已安装） | ✅ 已装 |
| NCCL | 未安装 | ❌ |
| cuSPARSELt | 未安装 | ❌ |
| NVSHMEM | 未安装 | ❌ |

---

## 2. GPU 加速方案尝试历程

### 阶段一：CUDA 12.8 版 libtorch (2.7.1+cu128)
- **来源**：宿主机已有压缩包 `libtorch-cxx11-abi-shared-with-deps-2.7.1+cu128.zip`
- **尝试**：解压至 `~/libtorch`，修改 CMakeLists.txt 指向该路径
- **编译结果**：`find_package(Torch)` 成功，但链接时报 `CUDA::nvToolsExt` 目标缺失
- **解决**：手动创建 `CUDA::nvToolsExt` 目标，编译通过（100%）
- **运行结果**：启动后约 1-2 分钟卡死，导致整个桌面环境注销（回到登录界面）
- **结论**：编译成功但运行不稳定，疑似 CUDA 版本不匹配（12.8 vs 13.2）或显存问题

### 阶段二：CUDA 13.2 版 libtorch (2.13.0+cu132)
- **来源**：PyTorch 官方测试频道 `https://download.pytorch.org/libtorch/test/cu132/libtorch-shared-with-deps-2.13.0+cu132.zip`
- **尝试**：解压至 `~/libtorch_cu132`，软链接至 `~/libtorch`
- **CMake 问题**：新版 libtorch 需要 CMake 3.18+（系统为 3.16.3）
- **解决**：手动升级 CMake 至 3.29.3（覆盖安装）
- **编译问题**：`CheckLinkerFlag` 模块缺失 → 升级 CMake 后解决
- **链接问题**：缺失 `libcusparseLt.so.0`、`libnccl.so.2`、`libnvshmem_host.so.3`
- **尝试解决**：安装 cuDNN 9（`libcudnn9-cuda-12`）成功，但 cuSPARSELt、NCCL、NVSHMEM 无法通过 apt 安装（需从 NVIDIA 官网手动下载）
- **最终结果**：链接失败，编译无法完成（`undefined reference` 上百个错误）

### 阶段三：尝试安装缺失库
- **cuDNN**：✅ 成功安装 `libcudnn9-cuda-12`（来自 NVIDIA 官方源）
- **NCCL**：❌ 未找到直接可用的 apt 包（需手动下载）
- **cuSPARSELt**：❌ 未找到直接可用的 apt 包（需手动下载）
- **NVSHMEM**：❌ 未找到直接可用的 apt 包（需手动下载）
- **依赖库太多，安装繁琐**
---

## 3. 失败原因分析（按优先级）

### 3.1 根本原因：依赖库的获取障碍
预编译的 CUDA 13.2 版 libtorch 依赖以下库，但这些库**无法通过 Ubuntu 20.04 默认源或 NVIDIA 公共源直接安装**：
- `libcusparseLt.so.0`（稀疏矩阵运算）
- `libnccl.so.2`（多卡通信）
- `libnvshmem_host.so.3`（NVMe 共享内存）

上述库需要从 NVIDIA 官网注册账号后手动下载 `.deb` 包，或通过 `pip` 安装 PyTorch 时附带安装（但 libtorch 为 C++ 版，无法复用 Python 环境）。这导致依赖链陷入死循环。

### 3.2 CMake 版本不匹配
- 系统 CMake 3.16.3 不支持 libtorch 2.13.0 的 `CheckLinkerFlag` 模块
- **解决**：升级至 3.29.3 ✅

### 3.3 CUDA 版本不兼容
- 宿主机 CUDA 13.2 与 libtorch 2.7.1+cu128 的预期版本（12.8）不匹配
- 虽然 NVIDIA 声称向后兼容，但实际运行时出现显存分配异常
- **尝试**：换用 cu132 版 libtorch，但引入了新的依赖问题

### 3.4 GPU 显存限制
- RTX 5060 Laptop 为 8GB VRAM
- Gazebo + 点云处理 + 神经网络推理可能超过 8GB
- 显存耗尽可能导致 GPU 驱动崩溃，进而触发 X Server 重启（注销现象）

### 3.5 容器环境限制
- Distrobox 容器共享宿主机内核，但某些 GPU 相关的设备节点（`/dev/dri`）可能未正确传递
- 容器内无法运行 systemd，部分 CUDA 工具包的服务无法启动

---

## 4. 租用云服务器的建议

如果未来需要稳定的 GPU 加速环境，**建议直接租用云 GPU 服务器**，而非在本机容器内折腾依赖问题。

### 4.1 推荐的云服务商配置
| 服务商 | 推荐配置 | 理由 |
|--------|----------|------|
| **阿里云** | GPU 计算型（A100 / V100 / T4） | 预装 CUDA 驱动 + Docker 环境 |
| **腾讯云** | GPU 计算型（V100 / T4） | 支持一键部署深度学习镜像 |
| **AWS** | EC2 G4 / G5 实例 | NVIDIA 官方驱动已预装 |
| **AutoDL** | RTX 3090 / A100 | 国内用户友好，按小时租用 |

### 4.2 操作系统选择
- **推荐**：Ubuntu 20.04 LTS

### 4.3 环境部署方案
#### 方案 A（推荐）：基于 NVIDIA 官方 Docker 镜像
```bash
# 拉取预装了 CUDA + cuDNN 的镜像
docker pull nvidia/cuda:13.2-cudnn-devel-ubuntu20.04

# 在镜像内安装 ROS Noetic 和 SimEnv
```

优点：所有 CUDA 依赖库（cuDNN、NCCL、cuSPARSELt）已预装

缺点：需要熟悉 Docker 操作

#### 方案 B：使用云服务商的预装镜像
阿里云/腾讯云提供“深度学习镜像”，已预装 CUDA、cuDNN、Python、PyTorch 等

直接在镜像内安装 ROS Noetic 和编译 SimEnv
#### 方案 C：使用 Conda 环境管理 libtorch
- 虽然 libtorch 是 C++ 库，但可以通过 Conda 安装带 CUDA 支持的版本
- `conda install libtorch -c pytorch`（需确认 CUDA 版本）

### 4.4 预算参考
| GPU 类型 | 约每小时费用（人民币） | 适用场景 |
|----------|----------------------|----------|
| T4 (16GB) | 10-20 元 | 轻量推理、开发调试 |
| V100 (16GB) | 25-40 元 | 中等训练、仿真 |
| A100 (40GB) | 50-80 元 | 大规模训练 |
| RTX 3090 (24GB) | 15-25 元（AutoDL） | 性价比高 |

---

## 5. 后续发展建议

### 5.1 短期（本周内）
1. **切换回 CPU 版 libtorch**，确保开发环境稳定可用。
2. **在 CPU 版基础上排查卡死原因**（可能与内存、控制周期、传感器数据量有关）：
   - 使用 `GUI=false` 无头模式
   - 使用 `START_CONTROLLER=0` 关闭控制器
   - 使用 `ENABLE_SENSOR_DATA=0` 关闭传感器
   - 逐步定位哪个组件导致卡死
3. **完成核心功能开发**，不依赖 GPU 加速。

### 5.2 中期（1-2 周内）
1. **评估 GPU 需求是否真的必要**：如果 CPU 版能跑通且性能可接受，则无需 GPU。
2. **如果确认需要 GPU**，按第 4 节建议租用云服务器，在干净环境中重新部署。
3. **在云服务器上测试 GPU 版 libtorch**，验证是否能稳定运行。

### 5.3 长期（1 个月内）
1. **优化 SimEnv 的资源配置**：
   - 减少楼层/房间数量（`FLOOR_COUNT=1 ROOMS_PER_FLOOR=2`）
   - 降低点云频率或分辨率
   - 降低控制周期（`UNITREE_CTRL_DT=0.008`）
2. **考虑迁移到 ROS 2 + Gazebo Garden**（新版 Gazebo 对 GPU 支持更好），但需评估迁移成本。

---

## 6. 结论

### 当前状态
- ✅ CPU 版 libtorch 编译通过，仿真可启动
- ❌ GPU 版 libtorch 因依赖库缺失无法完成编译
- ❌ 即使编译通过，运行时也存在显存不足/驱动不稳定的卡死问题

### 核心教训
在 Distrobox 容器内手动部署 GPU 版 libtorch 的依赖链极其复杂，**不建议继续投入时间**。最干净的解决方案是使用 NVIDIA 官方 Docker 镜像或云服务器的预装环境。

### 下一步行动
1. **立即**：切换回 CPU 版，继续开发和调试。
2. **后续**：如需 GPU，租用云服务器或重构容器。

---

2026-08-05 KianDu