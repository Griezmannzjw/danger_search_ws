#!/usr/bin/env python3
"""导航的 ROS 无关规划与状态辅助逻辑。

本模块只处理栅格、路径和目标状态，不导入 rospy。这样 `/move_base/make_plan`
和 Action 执行可以使用同一个规划入口，并且关键安全策略能够脱离 ROS 单测。
"""

from dataclasses import dataclass
import heapq
import math
import zlib


VALID_FAILURE_CODES = frozenset((
    "NONE",
    "SUCCEEDED",
    "UNREACHABLE",
    "CANCELED",
    "TIMEOUT",
    "CONTROL_FAILED",
    "SAFETY_STOP",
    "ROBOT_FALLEN",
    "LOCALIZATION_LOST",
))


def is_non_decreasing_stamp(previous_ns, candidate_ns):
    """只接受首帧或不早于当前快照的消息时间戳。"""
    return int(previous_ns) == 0 or int(candidate_ns) >= int(previous_ns)


def plan_route_variants(
    planner, start_xy, goal_xy, dynamic_blocked, obstacle_fresh
):
    """在一个不可变 planner 上区分静态无路和动态阻断。"""
    if planner is None:
        return None, None
    static_route = planner.plan_expanded(start_xy, goal_xy)
    if static_route is None:
        return None, None
    if not obstacle_fresh:
        return static_route, static_route
    return static_route, planner.plan_expanded(
        start_xy, goal_xy, dynamic_blocked
    )


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def zero_velocity():
    """返回非完整底盘的显式停车语义。"""
    return 0.0, 0.0


def map_snapshot_fingerprint(
    width, height, resolution, origin_x, origin_y, origin_yaw, data
):
    """Return a stable fingerprint for map geometry and occupancy values."""
    metadata = "{}:{}:{:.9f}:{:.9f}:{:.9f}:{:.9f}".format(
        int(width), int(height), float(resolution), float(origin_x),
        float(origin_y), float(origin_yaw),
    ).encode("ascii")
    encoded_cells = bytes((int(value) + 1) & 0xFF for value in data)
    return zlib.crc32(encoded_cells, zlib.crc32(metadata))


@dataclass
class DynamicObstacleWait:
    """Bounded zero-velocity wait while transient obstacles block a route."""

    timeout_s: float
    retry_interval_s: float
    blocked_since_s: float = None
    next_retry_s: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise ValueError("dynamic obstacle timeout must be positive")
        if not math.isfinite(self.retry_interval_s) or self.retry_interval_s <= 0.0:
            raise ValueError("dynamic obstacle retry interval must be positive")

    def begin(self, now_s):
        if self.blocked_since_s is None:
            self.blocked_since_s = float(now_s)
        self.next_retry_s = float(now_s) + self.retry_interval_s

    def retry_due(self, now_s):
        return self.blocked_since_s is not None and float(now_s) >= self.next_retry_s

    def record_retry(self, now_s):
        self.next_retry_s = float(now_s) + self.retry_interval_s

    def expired(self, now_s):
        return (
            self.blocked_since_s is not None
            and float(now_s) - self.blocked_since_s >= self.timeout_s
        )

    def clear(self):
        self.blocked_since_s = None
        self.next_retry_s = 0.0


def path_lengths(path):
    """返回各路径点到起点的累计长度。"""
    lengths = []
    total = 0.0
    for index, point in enumerate(path):
        if index:
            previous = path[index - 1]
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        lengths.append(total)
    return lengths


def path_progress(lengths, waypoint_index):
    """按已到达路径长度计算 0.0..1.0 的进度。"""
    if not lengths or lengths[-1] <= 1e-9:
        return 0.0
    index = min(max(0, int(waypoint_index)), len(lengths) - 1)
    return min(1.0, max(0.0, lengths[index] / lengths[-1]))


def goal_reached(current_xy_yaw, goal_xy_yaw, xy_tolerance, yaw_tolerance):
    """只有位置和最终朝向都满足容差时才表示成功。"""
    distance = math.hypot(
        current_xy_yaw[0] - goal_xy_yaw[0], current_xy_yaw[1] - goal_xy_yaw[1]
    )
    yaw_error = abs(normalize_angle(goal_xy_yaw[2] - current_xy_yaw[2]))
    return distance <= xy_tolerance and yaw_error <= yaw_tolerance


@dataclass
class GoalState:
    """供 ROS health 发布器读取的活动目标状态。"""

    active_goal_id: str = ""
    active: bool = False
    controller_active: bool = False
    stuck: bool = False
    failure_code: str = "NONE"
    failure_detail: str = ""
    progress: float = 0.0
    last_cmd_time: object = None

    def begin(self, goal_id):
        self.active_goal_id = str(goal_id)
        self.active = True
        self.controller_active = True
        self.stuck = False
        self.failure_code = "NONE"
        self.failure_detail = ""
        self.progress = 0.0

    def record_command(self, stamp):
        """仅由实际发布导航速度的路径调用。"""
        self.last_cmd_time = stamp

    def finish(self, failure_code, detail="", stuck=False):
        if failure_code not in VALID_FAILURE_CODES:
            raise ValueError("不支持的导航失败码: %s" % failure_code)
        self.active_goal_id = ""
        self.active = False
        self.controller_active = False
        self.stuck = bool(stuck)
        self.failure_code = failure_code
        self.failure_detail = str(detail)
        self.progress = 1.0 if failure_code == "SUCCEEDED" else min(1.0, max(0.0, self.progress))

    def cancel(self, detail="目标已取消"):
        self.finish("CANCELED", detail)
        return zero_velocity()


class InflatedOccupancyGrid:
    """不可变的保守占据栅格，静态与动态障碍共用同一膨胀半径。"""

    def __init__(
        self,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        origin_yaw,
        data,
        occupied_threshold=50,
        robot_radius=0.0,
        inflation_padding=0.0,
        allow_diagonal=True,
        max_expansions=200000,
    ):
        if int(width) != width or int(height) != height or width <= 0 or height <= 0:
            raise ValueError("地图宽高必须为正整数")
        if not math.isfinite(float(resolution)) or resolution <= 0.0:
            raise ValueError("地图分辨率必须为正数")
        if not all(math.isfinite(float(value)) for value in (origin_x, origin_y, origin_yaw)):
            raise ValueError("地图原点必须是有限数")
        if len(data) != int(width) * int(height):
            raise ValueError("地图数据长度与宽高不匹配")
        if not 1 <= int(occupied_threshold) <= 100:
            raise ValueError("占据阈值必须位于 1..100")
        if not math.isfinite(float(robot_radius)) or robot_radius < 0.0:
            raise ValueError("机器人半径必须为非负有限数")
        if not math.isfinite(float(inflation_padding)) or inflation_padding < 0.0:
            raise ValueError("膨胀余量必须为非负有限数")
        if int(max_expansions) != max_expansions or int(max_expansions) < 1:
            raise ValueError("最大搜索节点数必须为正整数")

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self._cos_origin_yaw = math.cos(self.origin_yaw)
        self._sin_origin_yaw = math.sin(self.origin_yaw)
        self.allow_diagonal = bool(allow_diagonal)
        self.max_expansions = int(max_expansions)
        self.inflation_radius = float(robot_radius) + float(inflation_padding)

        values = tuple(int(value) for value in data)
        # -1、100 以及任何非 0 单元都保守地不可通行。
        self.base_blocked = tuple(value != 0 for value in values)
        inflated = list(self.base_blocked)
        # unknown 和其他非自由格本身不可通行；只有真正占据的栅格需要
        # 做机器人半径膨胀。否则大面积 unknown 地图会在每次回调中产生
        # 数千万次重复膨胀运算，并把已知自由区边界全部吞掉。
        occupied_cells = (
            (index % self.width, index // self.width)
            for index, value in enumerate(values)
            if value >= int(occupied_threshold)
        )
        offsets = self._disk_offsets(self.inflation_radius)
        for cell_x, cell_y in occupied_cells:
            for dx, dy in offsets:
                nx, ny = cell_x + dx, cell_y + dy
                if self.in_bounds(nx, ny):
                    inflated[self.index(nx, ny)] = True
        self.inflated_blocked = tuple(inflated)

    def index(self, cell_x, cell_y):
        return int(cell_y) * self.width + int(cell_x)

    def in_bounds(self, cell_x, cell_y):
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    def _disk_offsets(self, radius_m):
        radius_cells = int(math.ceil(radius_m / self.resolution))
        return tuple(
            (dx, dy)
            for dy in range(-radius_cells, radius_cells + 1)
            for dx in range(-radius_cells, radius_cells + 1)
            if math.hypot(dx * self.resolution, dy * self.resolution) <= radius_m + 1e-9
        )

    def expanded_cells(self, cells):
        """按与静态占据栅格一致的规则膨胀临时动态障碍。"""
        expanded = set()
        offsets = self._disk_offsets(self.inflation_radius)
        for cell_x, cell_y in cells:
            for dx, dy in offsets:
                nx, ny = cell_x + dx, cell_y + dy
                if self.in_bounds(nx, ny):
                    expanded.add((nx, ny))
        return expanded

    def world_to_cell(self, x, y):
        """将 map 坐标转为单元坐标；负数使用 floor 语义。"""
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            return None
        dx, dy = float(x) - self.origin_x, float(y) - self.origin_y
        local_x = self._cos_origin_yaw * dx + self._sin_origin_yaw * dy
        local_y = -self._sin_origin_yaw * dx + self._cos_origin_yaw * dy
        cell_x = int(math.floor(local_x / self.resolution))
        cell_y = int(math.floor(local_y / self.resolution))
        return (cell_x, cell_y) if self.in_bounds(cell_x, cell_y) else None

    def cell_to_world(self, cell_x, cell_y):
        """返回旋转地图中指定栅格的中心 map 坐标。"""
        local_x = (int(cell_x) + 0.5) * self.resolution
        local_y = (int(cell_y) + 0.5) * self.resolution
        return (
            self.origin_x + self._cos_origin_yaw * local_x - self._sin_origin_yaw * local_y,
            self.origin_y + self._sin_origin_yaw * local_x + self._cos_origin_yaw * local_y,
        )

    def traversable(self, cell, dynamic_blocked=None):
        if cell is None or not self.in_bounds(cell[0], cell[1]):
            return False
        if self.inflated_blocked[self.index(cell[0], cell[1])]:
            return False
        return dynamic_blocked is None or cell not in dynamic_blocked

    def path_is_traversable(self, path, dynamic_cells=()):
        return self.path_is_traversable_expanded(
            path, self.expanded_cells(dynamic_cells)
        )

    def path_is_traversable_expanded(self, path, dynamic_blocked=()):
        """检查已膨胀动态障碍下的路径，供同一控制周期复用快照。"""
        return all(
            self.traversable(self.world_to_cell(point[0], point[1]), dynamic_blocked)
            for point in path
        )

    def plan(self, start_world, goal_world, dynamic_cells=()):
        """以 A* 搜索膨胀后栅格；起终点或搜索不可达时返回 None。"""
        return self.plan_expanded(
            start_world, goal_world, self.expanded_cells(dynamic_cells)
        )

    def plan_expanded(self, start_world, goal_world, dynamic_blocked=()):
        """使用已膨胀动态障碍运行 A*，避免一个周期内重复膨胀。"""
        start = self.world_to_cell(start_world[0], start_world[1])
        goal = self.world_to_cell(goal_world[0], goal_world[1])
        if not self.traversable(start, dynamic_blocked) or not self.traversable(goal, dynamic_blocked):
            return None
        if start == goal:
            return self._remove_duplicate_points([tuple(start_world), tuple(goal_world)])

        frontier = [(0.0, 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        expansions = 0
        while frontier:
            _, current_cost, current = heapq.heappop(frontier)
            if current_cost > g_score.get(current, float("inf")) + 1e-12:
                continue
            if current == goal:
                return self._reconstruct_path(came_from, start, goal, start_world, goal_world)
            expansions += 1
            if expansions > self.max_expansions:
                return None
            for neighbor, step_cost in self._neighbors(current, dynamic_blocked):
                tentative_cost = current_cost + step_cost
                if tentative_cost >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_cost
                heapq.heappush(
                    frontier,
                    (tentative_cost + self._heuristic(neighbor, goal), tentative_cost, neighbor),
                )
        return None

    def _neighbors(self, current, dynamic_blocked):
        cell_x, cell_y = current
        candidates = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            candidates.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])
        for dx, dy in candidates:
            neighbor = (cell_x + dx, cell_y + dy)
            if not self.traversable(neighbor, dynamic_blocked):
                continue
            if dx and dy:
                # 禁止斜向穿越两个相邻的阻塞格。
                if not self.traversable((cell_x + dx, cell_y), dynamic_blocked):
                    continue
                if not self.traversable((cell_x, cell_y + dy), dynamic_blocked):
                    continue
                yield neighbor, math.sqrt(2.0)
            else:
                yield neighbor, 1.0

    @staticmethod
    def _heuristic(current, goal):
        dx, dy = abs(current[0] - goal[0]), abs(current[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def _reconstruct_path(self, came_from, start, goal, start_world, goal_world):
        cells = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        path = [tuple(start_world)]
        path.extend(self.cell_to_world(cell[0], cell[1]) for cell in cells[1:-1])
        path.append(tuple(goal_world))
        return self._remove_duplicate_points(path)

    @staticmethod
    def _remove_duplicate_points(path):
        result = []
        for point in path:
            if not result or math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > 1e-9:
                result.append(point)
        return result
