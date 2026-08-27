# danger_search_perception

基于 RealSense RGB 与深度数据的红色球体危险源检测模块，对齐 P1 分层地图接口。

## 职责

1. 同步接收 RGB、深度图像和相机内参；
2. 使用 OpenCV 提取红色圆形候选；
3. 通过深度点、球面拟合和投影半径验证红色球体；
4. 使用平面残差排除红色方块干扰；
5. 通过 TF 将球心转换到目标坐标系；
6. 根据建图健康状态和采集时刻的机器人高度校验当前楼层；
7. 在同一楼层内进行三维跨帧关联、位置平滑和有限确认；
8. 发布检测和检测器健康状态。

perception 负责短时/跨楼层返回后的观测身份和位置平滑；任务全程的最终空间去重、
结果冻结和文件写入仍由 `danger_search_mission` 负责。两层职责使用同一检测消息，
不会改变现有 mission 输入接口。

## ROS 接口

### 订阅

| 默认话题 | 类型 | 说明 |
|---|---|---|
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB 相机内参 |
| `/mapping/status` | `danger_search_common/MappingStatus` | 当前楼层及地图健康状态 |

三个输入使用近似时间同步。话题名称均可通过私有 ROS 参数修改。

### 发布

| 默认话题 | 类型 | 说明 |
|---|---|---|
| `/danger_detector/detections` | `danger_search_common/DangerSourceArray` | 分层危险源检测数组 |
| `/danger_detector/status` | `danger_search_common/DetectionStatus` | 输入和 TF 健康状态 |

每个红球检测会真实填写：

- `detection_id`：由图像时间戳与帧内候选序号生成；
- `class_id=CLASS_DANGER_RED_SPHERE`；
- `position`：包含采集时间和实际目标坐标系；
- `floor_id`：来自稳定的当前楼层，编号从 `0` 开始；
- `map_epoch`：采集时活动的分层地图 epoch；
- `confidence`：二维与三维验证的综合置信度；
- `track_id`：同楼层三维跨帧轨迹 ID；
- `confirmed`：轨迹达到配置的连续命中次数后为 `true`；
- `position_covariance`：由轨迹观测离散度和保守先验生成；
- `source_time`：原始 RGB 图像时间。

`localization_correction_version` 来自当前定位状态；版本改变会清除旧短时关联。
检测数组为空可能表示当前帧没有通过验证的红色球体，也可能表示正在换层或地图不稳定；
具体原因由 `/danger_detector/status.status_reason` 给出。

### 多楼层发布门控

完整系统默认要求 `/mapping/status` 新鲜且满足
`ready && stable && !lost`。节点还会按 RGB 采集时间查询 `map -> base`，将机器人相对
状态必须满足 `ready && stable && !lost && !transitioning`。检测绑定当前 `floor_id`、
`map_epoch` 和定位修正版本；任一版本改变都会隔离旧轨迹，因此电梯换层期间不会把目标
错标到旧层，不同楼层相同 XY 也不会合并。

### TF

完整系统默认输出到 `map`，需要存在：

```text
map -> odom -> base -> camera
```

没有全局定位时，可以将 `target_frame` 临时设为 `base`，仅用于单模块识别测试。

## 当前算法

- [x] RGB、深度和 CameraInfo 近似时间同步
- [x] HSV 红色双区间分割
- [x] 形态学去噪和轮廓提取
- [x] 面积、圆形度、长宽比和外接圆填充率筛选
- [x] 深度有效性和离群值过滤
- [x] 像素反投影为相机坐标系三维点
- [x] 已知半径约束的球心估计
- [x] 球面残差、平面残差和投影半径联合验证
- [x] 按图像时间戳查询 TF
- [x] 建图稳定性和传感器时刻楼层高度双重门控
- [x] 同楼层三维跨帧关联、轨迹确认和位置协方差
- [x] `DangerSourceArray` 和 `DetectionStatus` 发布

当前传统视觉实现是无需训练数据的可运行基线。阈值仍需使用多个随机场景继续
标定；若遮挡、距离和光照变化导致效果不足，再评估实例分割或 YOLO。

参数见 `config/default.yaml`。其中 `min_depth_m` / `max_depth_m` 是深度图的
硬处理范围；`reliable_min_range` / `reliable_max_range` 是相机到拟合球心的
三维可靠距离，超出该范围的候选不会发布。节点启动时会检查 HSV、面积、
深度、半径、置信度和距离等参数；非法配置会直接报错，避免静默产生错误结果。

## 编译

```bash
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

新终端运行节点前需要加载 ROS 和工作空间：

```bash
source /opt/ros/noetic/setup.bash
source ~/myProject/danger_search_ws/devel/setup.bash
```

修改普通 Python 算法代码后通常不必重新编译；修改消息、`CMakeLists.txt`、
`package.xml` 或安装脚本后需要重新执行 `catkin_make`。

## 启动

完整系统具备 `map` TF 时：

```bash
roslaunch danger_search_perception danger_detector.launch
```

尚未启动定位模块，只验证识别功能时：

```bash
rosrun danger_search_perception danger_detector.py _target_frame:=base
```

该命令没有全局建图输入，节点会自动关闭正式 map 门控。也可显式写为：

```bash
rosrun danger_search_perception danger_detector.py \
  _target_frame:=base \
  _require_stable_mapping:=false \
  _verify_floor_height:=false
```

总系统也会通过 `danger_search_bringup/launch/competition.launch` 启动本节点，
正式运行不需要逐节点手动启动。

## 调试

查看检测和状态：

```bash
rostopic echo /danger_detector/detections
rostopic echo /danger_detector/status
```

这些命令只供人工调试。任务总控会自动订阅检测话题。

持续输出空数组时依次检查：

1. 红球是否进入相机视场；
2. RGB、深度和 CameraInfo 是否均在发布；
3. 消息时间戳能否在 `sync_slop_s` 内匹配；
4. `target_frame` 到相机光学坐标系的 TF 是否存在；
5. 二维轮廓、深度、球面或置信度阈值是否过严。
