# danger_search_perception

危险源视觉检测与三维定位模块。

## 职责

1. 从 RealSense RGB 图像中检测红色球体（危险源）
2. 结合深度图像计算危险源在相机坐标系下的三维坐标
3. 通过 TF 转换到 map 坐标系（世界坐标系）
4. 发布统一格式的危险源检测结果

## 节点

### danger_detector.py
危险源检测主节点。

#### 订阅话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/detections` | `danger_search_common/DangerSourceArray` | 危险源检测结果 |

#### 依赖 TF
- 查找 `real_sense` → `map` 的坐标变换

## 配置参数

见 `config/default.yaml`，主要包括：
- HSV 红色阈值
- 轮廓面积/圆形度过滤阈值
- 深度有效范围
- 传感器话题名

## 开发说明

当前为骨架版本，仅保留接口和基本结构。核心算法待实现：
- [ ] HSV 颜色阈值分割
- [ ] 形态学去噪
- [ ] 轮廓提取与圆形度判断
- [ ] 深度图采样与三维点计算
- [ ] TF 坐标变换
- [ ] 置信度评估
