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

2026-09-01、seed 42 的一次固定厅诊断使用以下算法命令：

```bash
roslaunch danger_search_bringup simulation_truth.launch \
  autostart:=true \
  entry_enabled:=false \
  fixed_elevator_hall_enabled:=true \
  fixed_elevator_hall_x:=1.15 \
  fixed_elevator_hall_y:=0.0 \
  fixed_elevator_hall_into_yaw:=0.0
```

对应仿真出生覆盖仅用于该隔离测试：`x=0.5 y=2.6 z=0.6 yaw=0`。固定厅 nominal
approach 距机器人不超过 `plan_tolerance` 时，exploration 可直接开始开门验证，不再因
局部未知栅格把“已经位于厅前”误报为 `UNREACHABLE_HALL`。门运动差分仍必须通过。

本轮最佳真实服务链已达到：

```text
OPEN_CURRENT_START -> CAPTURE_OPEN_SCAN -> VALIDATE_CLOSE_START
-> CAPTURE_CLOSED_SCAN -> REOPEN_CURRENT_START -> ENTER
-> CLOSE_CURRENT -> CALL_TARGET -> SWITCH_FLOOR -> EXIT
```

该次运行成功得到 `current_floor=1, map_epoch=2` 和 0/1 层独立地图，但 Unitree 不执行
负向 `linear.x`，最终以 `EXIT_FAILED: elevator crossing timed out` 结束。门槛参数已改为
`1.00 m @ 0.40 m/s`，并增加目标层厅门平面判定：机器人中心若已在厅侧至少 `0.05 m`，
直接进入 `WAIT_STABLE`，不再强制倒车。该分支已有纯逻辑测试；后续重复实测又在入梯后以
`excessive_tilt` 安全取消，说明当前门槛动力学仍不稳定，尚未形成可重复的完整 Action 成功。

## 5. 2026-09-01 已知外部阻塞

官方默认出生朝向下，输入 `2` 固定站立时 IMU 约为 `roll=-0.1°、pitch=3.0°`；输入 `6`
后稳定到约 `roll=11.5°、pitch=-19.4°`。正式 profile 的 15°恢复阈值因此不能解除启动安全
门。仅隔离 profile 使用 25°恢复阈值，触发阈值仍保持 30°。

即使隔离 profile 安全门解除，Unitree 对低速和负向速度的执行仍不一致；`0.25 m/s` 基本
无位移，`0.40 m/s` 可入梯但重复运行存在翻倒。该现象位于官方 `simenvnew` 的 Unitree RL
运行条件/控制器与门槛动力学侧；本任务未修改 `simenvnew`。在该阻塞解决前，只能确认算法
纯逻辑、门服务、楼层切换和独立地图链，不能宣称正式公开出生三层闭环或 S3/P3 通过。

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
