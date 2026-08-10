# danger_search_bringup

P0 系统集成启动包。`competition.launch` 一次装配 localization、perception、navigation、
exploration、control 和 mission，共九个运行节点。

## 默认目录约定

为让队员克隆后尽量零配置，默认约定两个仓库位于同一父目录：

```text
myProject/
├── SimEnv/
└── danger_search_ws/
```

launch 会从自身 ROS 包路径推导同级 `SimEnv`，结果默认写入：

```text
SimEnv/results/detected_danger.json
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `simenv_root` | 自动查找同级 `SimEnv` | 不同部署布局只需覆盖这一项 |
| `result_file` | `$(arg simenv_root)/results/detected_danger.json` | 可选的完整结果文件覆盖 |
| `autostart` | `false` | 为 true 时所有预检就绪后自动开始任务 |
| `open_main_entrance` | `true` | 调用官方服务实际打开主入口；仅隔离调试时关闭 |

零配置启动：

```bash
roslaunch danger_search_bringup competition.launch autostart:=true
```

赛事组若把 SimEnv 放在其他位置：

```bash
roslaunch danger_search_bringup competition.launch \
  autostart:=true simenv_root:=/absolute/path/to/SimEnv
```

## P0 推荐流程

1. 以单楼层、关闭 referee odom 和真值点云变换的方式启动 SimEnv；
2. 在 junior_ctrl 终端按 `2` 站立，再按 `6` 进入 `/cmd_vel` 模式；
3. 启动 `competition.launch autostart:=true`；
4. bringup 调用官方门服务打开 `main_entrance`；
5. mission 在门外记录出生点并自动进入建筑，再进入 `EXPLORING`；
6. exploration 收敛后自动进入 `RETURNING`，返回门外出生点；
7. 返回起点后自动写结果并进入 `FINISHED`。

如果 `autostart:=false`，第 3 步后手动调用一次：

```bash
rosservice call /danger_search/start "{}"
```

任务开始后无需调用 finish。`/danger_search/finish` 仅用于调试时提前结束探索，它也会先
执行返航，不会直接跳过返航写结果。

## 其他启动文件

- `perception_only.launch`：感知模块隔离调试；
- `navigation_only.launch`：定位、导航、探索和控制联调；
- `competition.launch`：P0 完整系统唯一正式入口。
