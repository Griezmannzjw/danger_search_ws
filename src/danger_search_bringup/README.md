# danger_search_bringup

P1 比赛系统集成启动包。`competition.launch` 与 `simulation_truth.launch` 共享内部装配，
统一启动 localization、perception、navigation、exploration、control、mission、入口门控制和
required preflight。正式定位链使用 GICP；仓库中的 FAST-LIO2 源码未接入本轮默认构建。

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
| `scene_info_file` | `$(arg simenv_root)/generated_building/team_scene_info.json` | 唯一允许的公开场景合同 |
| `result_coordinate_frame` | `auto` | 按公开合同选择 world，否则 start_relative |
| `autostart` | `false` | 为 true 时所有预检就绪后自动开始任务 |
| `open_main_entrance` | `true` | 调用官方服务实际打开主入口；仅隔离调试时关闭 |

`run_profile`、`competition_mode`、`multifloor_enabled`、`localization_backend`、
`gazebo_base_link` 和 `use_hector_correction` 不是公共入口参数：`competition.launch`
固定为 `formal / true / true / gicp` 并关闭兼容校正，避免误把测试运行认作正式验收。

零配置启动：

```bash
roslaunch danger_search_bringup competition.launch autostart:=true
```

赛事组若把 SimEnv 放在其他位置：

```bash
roslaunch danger_search_bringup competition.launch \
  autostart:=true simenv_root:=/absolute/path/to/SimEnv
```

隔离定位误差、测试其他模块时使用独立的真值入口：

```bash
roslaunch danger_search_bringup simulation_truth.launch autostart:=true
```

该入口固定为 `simulation_truth / false / true / gazebo_truth`，仅
`/gazebo_truth_odometry` 可以订阅 `/gazebo/link_states`，并检查指定
`a1_gazebo::base` link 的新鲜存在。它不会打开 referee odom、ground-truth 点云或危险源
真值，且独立写入 `SimEnv/results/detected_danger.simulation_truth.json`；该结果的
`official_eligible=false`，不得用于正式比赛验收。

## 正式推荐流程

1. 以 `ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0
   POINTCLOUD_USE_GROUND_TRUTH_ODOM=0` 启动 SimEnv；
2. 在 junior_ctrl 终端按 `2` 站立，再按 `6` 进入 `/cmd_vel` 模式；
3. 启动 `competition.launch autostart:=true`；
4. bringup 调用官方门服务打开 `main_entrance`；
5. mission 在门外记录出生点，以短目标滚动进入建筑，确认前向进度后再进入 `EXPLORING`；
6. exploration 完成全部 served floors 后，mission 必要时乘电梯回 0 层，再返回出生点；
7. 返回起点后自动写结果并进入 `FINISHED`。

官方生成场景当前把主入口设为 `initial_open: true`；`entrance_door` 仍会在
`competition.launch` 启动后调用 `/set_door_state` 再次确认打开。只有服务成功返回后才
发布锁存的 `/entrance/ready=true`，mission 的启动预检不会在此之前放行。

如果 `autostart:=false`，第 3 步后手动调用一次：

```bash
rosservice call /danger_search/start "{}"
```

任务开始后无需调用 finish。`/danger_search/finish` 仅用于调试时提前结束探索，它也会先
执行返航，不会直接跳过返航写结果。

## 其他启动文件

- `perception_only.launch`：感知模块隔离调试；
- `navigation_only.launch`：定位、导航、探索和控制联调；
- `competition.launch`：P1 多楼层唯一正式入口；preflight 失败时整个 launch 退出。
- `simulation_truth.launch`：非正式 Gazebo 连续位姿验证入口，保留与正式相同的多楼层和
  电梯接口合同。
