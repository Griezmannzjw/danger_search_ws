# GUI=false 启动与多楼层联调流程

本文档是 `danger_search_ws` 的可执行启动记录。仓库和 Git 历史中未找到此前提到的
`command_bringup_flow.md`/`Command_bringup_flow.md`，因此于 2026-09-01 按官方
`simenvnew` 实际接口补建。`simenvnew` 只作为环境事实源，不在本流程中修改。

## 1. 终端环境

所有终端先确保 ROS 使用 Ubuntu 20.04 的 Python 3.8，而不是登录 shell 中的 Miniconda
Python 3.14：

```bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
test "$(command -v python3)" = /usr/bin/python3
source /opt/ros/noetic/setup.bash
```

构建算法工作区：

```bash
cd /home/langan/danger_search_ws
catkin_make -j4
```

算法联调终端必须按官方环境在前、算法工作区在后的顺序叠加：

```bash
source /opt/ros/noetic/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
source /home/langan/simenvnew/devel/setup.bash
source /home/langan/danger_search_ws/devel/setup.bash
```

不要 source 旧 `/home/langan/SimEnv/devel/setup.bash`。

## 2. 正式 GUI=false 启动

终端 A 启动官方仿真。正式验证不要覆盖公开出生点、朝向或门梯坐标：

```bash
cd /home/langan/simenvnew
GUI=false \
ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=0 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
./auto.sh
```

在 `junior_ctrl` 交互终端中严格按以下时序操作：

1. 输入 `2`，等待机器人完成固定站立。
2. 等待至少 15 秒，确认 IMU 姿态稳定且未触发姿态安全门。
3. 输入 `6`，切换到接收 `/cmd_vel` 的 RL 控制模式。
4. 再启动算法栈。

算法运行期间禁止输入 `8` reset。reset 会改变机器人与定位参考，破坏本轮地图、任务起点和
正式验收语义。

终端 B 启动正式算法：

```bash
source /opt/ros/noetic/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
source /home/langan/simenvnew/devel/setup.bash
source /home/langan/danger_search_ws/devel/setup.bash
roslaunch danger_search_bringup competition.launch autostart:=true
```

正式结果固定检查：

```bash
rosrun danger_search_mission validate_result.py --official \
  /home/langan/simenvnew/results/detected_danger.json
```

## 3. 状态采集

启动后至少检查以下接口：

```bash
rosnode list
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /navigation/health
rostopic echo -n 1 /exploration/status
rostopic echo -n 1 /mission/status
rostopic echo -n 1 /danger_search/safety_stop
rostopic echo -n 1 /danger_search/posture_safety_reason
rostopic info /cmd_vel
```

正式闭环要求：

- `/cmd_vel` 只有 `/control` 发布。
- `move_base` 先输出 `/danger_search/move_base_cmd_vel`，再由
  `navigation_command_mux` 输出 `/danger_search/nav_cmd_vel`。
- 电梯门槛穿越由 `/danger_search/elevator_cmd_vel` 租约输入 control。
- 进梯完成不能只按累计位移判断。机器人中心必须沿厅门 `into_yaw` 方向越过门平面至少
  `elevator_entry_cabin_side_margin_m`；当前实验值为 `0.43 m`，对应后端机身
  `0.35 m` 加 `0.08 m` 足迹余量。未达到该条件时，即使累计位移超过
  `elevator_crossing_distance_m` 也不得关门；若扫掠足迹遇障则保持门开启并返回
  `ENTER_FAILED`。
- 换层期间 mapping/navigation 的 `transitioning=true`；恢复时两者的
  `current_floor/map_epoch/map_version` 一致。
- 新层至少产生配置要求的新地图版本并稳定后，exploration 才恢复普通目标。
- exploration 的稳定 `FAILED` 必须使 mission 进入 `ERROR`，不能长期停在
  `EXPLORING`。

手工触发一次换层 Action 只用于联调，不代表自主调度通过：

```bash
/usr/bin/python3 - <<'PY'
import actionlib
import rospy
from danger_search_common.msg import TransitFloorAction, TransitFloorGoal

rospy.init_node("manual_transit_floor_probe", anonymous=True)
client = actionlib.SimpleActionClient(
    "/danger_search/transit_floor", TransitFloorAction
)
assert client.wait_for_server(rospy.Duration(20.0))
client.send_goal(TransitFloorGoal(target_floor=1, exit_to_hall=True))
assert client.wait_for_result(rospy.Duration(420.0))
print(client.get_state(), client.get_result())
PY
```

固定失败码：`NO_HALL`、`UNREACHABLE_HALL`、`SERVICE_UNAVAILABLE`、
`SERVICE_REJECTED`、`SERVICE_TIMEOUT`、`ENTER_FAILED`、`FLOOR_MISMATCH`、
`MAP_NOT_STABLE`、`EXIT_FAILED`、`CANCELED`、`STALE_EPOCH`。

## 4. simulation_truth 隔离联调

该 profile 只用于隔离 GICP、入口和厅门发现问题。它使用仿真真值定位，固定厅参数也来自
隔离诊断，严禁作为正式规划输入或 S3/P3 通过证据。

2026-09-01、seed 42 的手工 Action 固定厅诊断使用以下算法命令：

```bash
roslaunch danger_search_bringup simulation_truth.launch \
  autostart:=false \
  entry_enabled:=false \
  fixed_elevator_hall_enabled:=true \
  fixed_elevator_hall_x:=1.15 \
  fixed_elevator_hall_y:=0.0 \
  fixed_elevator_hall_into_yaw:=0.0
```

对应仿真出生覆盖仅用于该隔离测试：`x=0.5 y=2.6 z=0.6 yaw=0`。固定厅 nominal
approach 距机器人不超过 `plan_tolerance` 时，exploration 可直接开始开门验证，不再因
局部未知栅格把“已经位于厅前”误报为 `UNREACHABLE_HALL`。门运动差分仍必须通过。

本轮真实服务链已达到：

```text
OPEN_CURRENT_START -> CAPTURE_OPEN_SCAN -> VALIDATE_CLOSE_START
-> CAPTURE_CLOSED_SCAN -> REOPEN_CURRENT_START -> ENTER
-> CLOSE_CURRENT -> CALL_TARGET -> SWITCH_FLOOR -> EXIT -> WAIT_STABLE -> DONE
```

门槛实验参数为最小穿越距离 `0.85 m @ 0.40 m/s`，进梯整机净空为门内 `0.43 m`。修复后
`WAIT_STABLE` 不再因目标层 active map 或 navigation health 短暂迟到而立即失败，仍由总换层 deadline 保持 fail-closed；残留普通导航
目标继续立即返回 `MAP_NOT_STABLE`。目标层离梯过程中也会持续检查厅门平面，机器人中心一旦
到达厅侧 `0.05 m` 就停止穿越并进入 `WAIT_STABLE`，不再只在离梯开始前检查一次。

`GUI=true` 完整重启复测中，旧逻辑曾在机器人中心仅进入门内约 `0.20 m` 时停止并关门，导致
机身尾部仍位于门缝并发生碰撞。修复后机器人到达 `x=1.670 m` 才进入关门阶段；固定厅门平面
为 `x=1.150 m`，实际门内净空 `0.520 m`，超过 `0.430 m` 阈值，随后 `0 -> 1` Action 返回
`success=true,current_floor=1,map_epoch=2`。另一运行从
首层直达三层，真实完成呼梯和地图切换到 `current_floor=2,map_epoch=2`，随后在加载动态厅侧
修复后完成 `2 -> 0` 返回，结果为 `success=true,current_floor=0,map_epoch=3`，并保留 0/2 层
独立地图。由于测试中途重启过 exploration 节点，不能把这组隔离证据解释为自主三层探索完成。

## 5. 2026-09-01 已知外部阻塞

RL 初始姿态异常不是必现：本轮固定站立约为 `roll=-0.05°、pitch=-0.42°`，输入 `6` 后约为
`roll=-0.29°、pitch=-1.24°`。此前 `pitch≈-19.4°` 应记录为运行不稳定样本，而非固定事实。

控制器输出 `Switched from passive to fixed stand` 不能单独证明真实站立。一次异常运行中，
执行 `8 -> 2` 后状态机报告 fixed stand，但 `a1_gazebo::base` 高度仅 `0.120 m`，实际仍趴地；
完整停止并重启 `auto.sh` 后，同一检查恢复为 `0.257 m`，切换 RL 后稳定在 `0.307 m`。因此每次
输入 `2` 后至少等待 10 秒，并在输入 `6` 前通过 GUI 或 `/gazebo/get_link_state` 确认 base 高度
和姿态；若 fixed stand 仍趴地，优先完整重启仿真，不要直接启动算法。

该站立失败不是“RL 未使用 GPU”导致：`State_FixedStand` 不加载或调用策略模型，GPU 只影响
输入 `4/6` 后的 `State_RL`。宿主 RTX 5060 可被 `nvidia-smi` 识别，但当前已确认
`libtorch-cu118` 与该 GPU 不兼容，正式基线仍使用 `UNITREE_RL_DEVICE=cpu`。开启
`UNITREE_LOG_WAIT_WARNINGS=1` 后，本次 GUI RL 运行观察到 4 ms 循环偶发耗时约
`4.5–6.6 ms`，另有 20 ms 线程耗时约 `22–42 ms`；这是步态实时性风险，但本次仍完成换层，
不能解释固定站立阶段的趴地。

Unitree 速度矩阵显示：平地 `+0.25` 可前进但门槛处静止后无法继续，`-0.25` 基本无效，
`+0.40/-0.40` 在首层平地均可执行，横移 `±0.25` 基本无效，原地转向 `±0.8` 有效。但目标层
重复测试中 `-0.40` 仍可能完全无位移，原地转向也可能随机长时间不收敛；转身后使用正向
`0.40` 又可退回厅前。主要外部阻塞因此是 Unitree RL 在门槛/接触状态和不同运行样本中的
运动可重复性，不是“所有负向速度都不响应”。`simenvnew` 未被修改；在公开出生、正式 GICP、
无固定厅条件下完成自主逐层探索前，仍不能宣称正式三层闭环或 S3/P3 通过。

## 6. 停止顺序

1. 先在算法 `roslaunch` 终端按一次 `Ctrl-C`，等待节点退出和 Action 取消。
2. 再在 `auto.sh`/`junior_ctrl` 终端按 `Ctrl-C` 停止控制器。
3. 若脚本提示 Gazebo 保留供检查，再按一次 `Ctrl-C` 停止 Gazebo。
4. 不用 `kill -9` 或运行中 reset 代替正常停止，除非进程已无法响应。

## 7. 回归门禁

```bash
cd /home/langan/danger_search_ws
source /opt/ros/noetic/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
source /home/langan/simenvnew/devel/setup.bash
catkin_make -j4
source devel/setup.bash
catkin_make run_tests -j4
catkin_test_results --all build/test_results
git diff --check
```

自动化通过只证明代码合同；正式多层验收仍需在 `GUI=false`、无 truth、无固定厅、公开出生
条件下完成每个 served floor 的独立地图、换层恢复、逐层收敛和 mission 终态。
