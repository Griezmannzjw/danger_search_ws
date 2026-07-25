# danger_search_bringup

系统集成启动包。

## 职责

1. 提供统一的启动入口（launch 文件）
2. 维护全局配置参数
3. 各模块的组合启动方案

## 启动文件

### competition.launch
**完整比赛启动文件**，启动所有模块。

```bash
roslaunch danger_search_bringup competition.launch
```

参数：
- `result_file`：结果输出路径
- `autostart`：是否自动开始任务（默认 false）

### perception_only.launch
仅启动感知模块，用于单独调试视觉识别。

### navigation_only.launch
仅启动导航 + 探索 + 定位 + 控制，用于单独调试运动。

## 全局配置

`config/global.yaml`：所有模块共享的基础参数
- 坐标系名称约定
- 传感器话题名称（与 SimEnv 对齐）
- 机器人初始位姿
- 结果输出路径

## 使用流程

### 1. 先启动仿真环境
```bash
cd ~/SimEnv
./auto.sh
# 终端按 2 站立，按 6 切换到 /cmd_vel 模式
```

### 2. 启动算法框架
```bash
cd ~/danger_search_ws
source devel/setup.bash
roslaunch danger_search_bringup competition.launch
```

### 3. 开始任务
```bash
rosservice call /danger_search/start "{}"
```

### 4. 结束任务并输出结果
```bash
rosservice call /danger_search/finish "{}"
```

结果自动写入 `results/detected_danger.json`。

## 模块启动顺序

1. localization（定位建图）
2. perception（危险源感知）
3. navigation（导航控制）
4. exploration（探索规划）
5. control（速度仲裁）
6. mission（任务总控）
