#!/usr/bin/env python3
"""导航的 ROS 无关规划与状态辅助逻辑。

本模块只处理栅格、路径和目标状态，不导入 rospy。这样 `/move_base/make_plan`
和 Action 执行可以使用同一个规划入口，并且关键安全策略能够脱离 ROS 单测。
"""

from dataclasses import dataclass
from collections import deque
import heapq
import math

import cv2
import numpy as np


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


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def zero_velocity():
    """返回非完整底盘的显式停车语义。"""
    return 0.0, 0.0


def event_replan_required(
    route_blocked, route_endpoint_reached, requested, deviation,
    deviation_threshold,
):
    """Return whether an active route needs an event-driven replan.

    Time alone deliberately never appears here: an accepted route remains in
    use until it is invalid, it no longer reaches a requested goal, an
    operator asks for a refresh, or the robot has deviated materially.
    """
    return bool(
        route_blocked
        or route_endpoint_reached
        or requested
        or float(deviation) > float(deviation_threshold)
    )


def inflate_convex_polygon(points, padding):
    """Return a convex hull expanded by ``padding`` in every planar direction."""
    points = [(float(x), float(y)) for x, y in points]
    if len(points) < 3:
        raise ValueError("footprint 至少需要三个点")
    if not math.isfinite(float(padding)) or padding < 0.0:
        raise ValueError("footprint padding 必须为非负有限数")
    samples = []
    directions = tuple(
        (math.cos(index * math.pi / 8.0), math.sin(index * math.pi / 8.0))
        for index in range(16)
    )
    for point_x, point_y in points:
        samples.append((point_x, point_y))
        for direction_x, direction_y in directions:
            samples.append((
                point_x + padding * direction_x,
                point_y + padding * direction_y,
            ))
    hull = cv2.convexHull(np.asarray(samples, dtype=np.float32)).reshape((-1, 2))
    return tuple((float(point[0]), float(point[1])) for point in hull)


class FootprintHistory:
    """Short planar history used to conservatively cover a quadruped gait sweep."""

    def __init__(self, fallback, history_seconds=0.40, padding=0.04):
        if not math.isfinite(float(history_seconds)) or history_seconds < 0.0:
            raise ValueError("footprint history 必须为非负有限数")
        self.fallback = tuple((float(x), float(y)) for x, y in fallback)
        if len(self.fallback) < 3:
            raise ValueError("fallback footprint 至少需要三个点")
        self.history_seconds = float(history_seconds)
        self.padding = float(padding)
        self._history = deque()

    def reset(self):
        self._history.clear()

    def update(self, stamp, polygon):
        stamp = float(stamp)
        polygon = tuple((float(x), float(y)) for x, y in polygon)
        if not math.isfinite(stamp) or len(polygon) < 3:
            raise ValueError("动态 footprint 输入无效")
        self._history.append((stamp, polygon))
        while self._history and stamp - self._history[0][0] > self.history_seconds:
            self._history.popleft()
        return self.current(stamp)

    def current(self, stamp=None):
        if stamp is not None:
            stamp = float(stamp)
            while self._history and stamp - self._history[0][0] > self.history_seconds:
                self._history.popleft()
        points = [point for _, polygon in self._history for point in polygon]
        if not points:
            points = list(self.fallback)
        return inflate_convex_polygon(points, self.padding)


@dataclass(frozen=True)
class RecoveryCandidate:
    maneuver: str
    command_x: float
    command_y: float
    command_yaw: float
    distance: float
    min_clearance: float


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


class PoseProgressChecker:
    """ROS-independent staged pose progress checker."""

    IDLE = "IDLE"
    ROTATING = "ROTATING"
    TRANSLATING = "TRANSLATING"

    def __init__(
        self,
        progress_distance=0.05,
        progress_angle=0.20,
        time_allowance=8.0,
        command_speed_threshold=0.05,
        command_angle_threshold=0.05,
    ):
        values = (
            progress_distance, progress_angle, time_allowance,
            command_speed_threshold, command_angle_threshold,
        )
        if any(not math.isfinite(float(value)) or value <= 0.0 for value in values):
            raise ValueError("进展检查参数必须为正有限数")
        self.progress_distance = float(progress_distance)
        self.progress_angle = float(progress_angle)
        self.time_allowance = float(time_allowance)
        self.command_speed_threshold = float(command_speed_threshold)
        self.command_angle_threshold = float(command_angle_threshold)
        self.reset()

    def reset(self):
        self.mode = self.IDLE
        self.baseline_pose = None
        self.baseline_time = None
        self.paused_at = None

    def pause(self, now):
        now = float(now)
        if self.paused_at is None and math.isfinite(now):
            self.paused_at = now

    def resume(self, now):
        now = float(now)
        if self.paused_at is not None:
            if self.baseline_time is not None and math.isfinite(now):
                self.baseline_time += max(0.0, now - self.paused_at)
            self.paused_at = None

    def _command_mode(self, command):
        vx, vy, wz = (float(value) for value in command)
        if math.hypot(vx, vy) >= self.command_speed_threshold:
            return self.TRANSLATING
        if abs(wz) >= self.command_angle_threshold:
            return self.ROTATING
        return self.IDLE

    def update(self, pose, command, now):
        """Return true only after the active motion stage exceeds its allowance."""
        now = float(now)
        pose = tuple(float(value) for value in pose[:3])
        if not math.isfinite(now) or not all(math.isfinite(value) for value in pose):
            self.reset()
            return False
        self.resume(now)
        mode = self._command_mode(command)
        if mode == self.IDLE:
            self.reset()
            return False
        if mode != self.mode or self.baseline_pose is None:
            self.mode = mode
            self.baseline_pose = pose
            self.baseline_time = now
            return False
        if mode == self.TRANSLATING:
            progressed = math.hypot(
                pose[0] - self.baseline_pose[0], pose[1] - self.baseline_pose[1]
            ) >= self.progress_distance
        else:
            progressed = abs(normalize_angle(
                pose[2] - self.baseline_pose[2]
            )) >= self.progress_angle
        if progressed:
            self.baseline_pose = pose
            self.baseline_time = now
            return False
        return now - self.baseline_time >= self.time_allowance


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


class DynamicObstacleTemporalFilter:
    """确认重复观测到的动态栅格，并独立保留已确认栅格一段时间。"""

    def __init__(self, confirmation_frames=3, confirmation_hits=2, obstacle_ttl=0.50):
        if int(confirmation_frames) != confirmation_frames or confirmation_frames < 1:
            raise ValueError("动态障碍确认帧数必须为正整数")
        if (int(confirmation_hits) != confirmation_hits or confirmation_hits < 1
                or confirmation_hits > confirmation_frames):
            raise ValueError("动态障碍确认命中数必须位于 1..确认帧数")
        if not math.isfinite(float(obstacle_ttl)) or obstacle_ttl < 0.0:
            raise ValueError("动态障碍 TTL 必须为非负有限数")
        self.confirmation_frames = int(confirmation_frames)
        self.confirmation_hits = int(confirmation_hits)
        self.obstacle_ttl = float(obstacle_ttl)
        self.reset()

    def reset(self):
        """清除扫描历史和已确认障碍；地图几何改变时由调用方使用。"""
        self._frames = deque(maxlen=self.confirmation_frames)
        self._confirmed_last_seen = {}
        self._last_scan_time = None

    def observe(self, scan_time, cells):
        """记录一个新扫描时间戳的栅格集合，并返回当前已确认集合。"""
        scan_time = float(scan_time)
        if not math.isfinite(scan_time):
            raise ValueError("动态障碍扫描时间必须为有限数")
        # /scan 只保留最新消息；拒绝重复或倒退的时间戳，避免同一帧在
        # 控制循环和 make_plan 的重复查询中增加命中数。
        if self._last_scan_time is not None and scan_time <= self._last_scan_time:
            return self.confirmed_cells(scan_time)

        self.confirmed_cells(scan_time)
        frame = frozenset((int(cell_x), int(cell_y)) for cell_x, cell_y in cells)
        self._frames.append((scan_time, frame))
        self._last_scan_time = scan_time

        # 通过确认的格只要再次被看见，就以最新观测续期；不要求它在每个
        # 新滑窗中重新完成确认。
        for cell in frame:
            if cell in self._confirmed_last_seen:
                self._confirmed_last_seen[cell] = scan_time

        hits = {}
        last_seen = {}
        for frame_time, frame_cells in self._frames:
            for cell in frame_cells:
                hits[cell] = hits.get(cell, 0) + 1
                last_seen[cell] = frame_time
        for cell, count in hits.items():
            if count >= self.confirmation_hits:
                self._confirmed_last_seen[cell] = last_seen[cell]
        return self.confirmed_cells(scan_time)

    def confirmed_cells(self, now):
        """返回 TTL 未过期的已确认栅格，并移除过期项。"""
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("动态障碍查询时间必须为有限数")
        expired = [
            cell for cell, last_seen in self._confirmed_last_seen.items()
            if now - last_seen > self.obstacle_ttl
        ]
        for cell in expired:
            del self._confirmed_last_seen[cell]
        return set(self._confirmed_last_seen)


class InflatedOccupancyGrid:
    """不可变的保守占据栅格，静态和动态障碍可使用不同膨胀半径。"""

    _kernel_cache = {}

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
        dynamic_inflation_radius=None,
        clearance_soft_margin=0.15,
        clearance_cost_weight=4.0,
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
        if (dynamic_inflation_radius is not None
                and (not math.isfinite(float(dynamic_inflation_radius))
                     or dynamic_inflation_radius < 0.0)):
            raise ValueError("动态障碍膨胀半径必须为非负有限数")
        if (not math.isfinite(float(clearance_soft_margin))
                or clearance_soft_margin < 0.0
                or not math.isfinite(float(clearance_cost_weight))
                or clearance_cost_weight < 0.0):
            raise ValueError("净空代价参数必须为非负有限数")
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
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(robot_radius) + float(inflation_padding)
        self.dynamic_inflation_radius = (
            self.inflation_radius
            if dynamic_inflation_radius is None else float(dynamic_inflation_radius)
        )
        self.clearance_soft_margin = float(clearance_soft_margin)
        self.clearance_cost_weight = float(clearance_cost_weight)

        grid = np.asarray(data, dtype=np.int16).reshape((self.height, self.width))
        # Unknown space stays blocked, while observed cells below the configured
        # occupancy threshold remain traversable.  Treating every non-zero
        # probability as occupied made ``occupied_threshold`` ineffective and
        # rejected mildly noisy probabilistic maps.
        base_blocked = (grid < 0) | (grid >= int(occupied_threshold))
        occupied = (grid >= int(occupied_threshold)).astype(np.uint8)
        inflated = cv2.dilate(
            occupied,
            self._disk_kernel(self.resolution, self.inflation_radius),
            iterations=1,
        ) != 0

        # 未知格和达到占据阈值的栅格不可通行。OpenCV 在 C++ 中完成圆形
        # 膨胀，避免每次地图更新都用 Python 遍历整张栅格。
        # Bytes keep the immutable/read-only semantics used by the planner,
        # while avoiding millions of Python bool objects on every map update.
        self.unknown = (grid == -1).astype(np.uint8).ravel().tobytes()
        self.base_blocked = base_blocked.astype(np.uint8).ravel().tobytes()
        self.inflated_blocked = np.logical_or(
            base_blocked, inflated
        ).astype(np.uint8).ravel().tobytes()
        self._base_blocked_mask = base_blocked.astype(bool)
        self._inflated_blocked_mask = np.logical_or(base_blocked, inflated)
        free_for_distance = (~base_blocked).astype(np.uint8)
        self.clearance_m = cv2.distanceTransform(
            free_for_distance, cv2.DIST_L2, cv2.DIST_MASK_PRECISE
        ).astype(np.float32) * self.resolution

    @classmethod
    def _disk_kernel(cls, resolution, radius_m):
        """缓存与现有离散圆形偏移完全一致的 OpenCV 膨胀核。"""
        key = (float(resolution), float(radius_m))
        kernel = cls._kernel_cache.get(key)
        if kernel is not None:
            return kernel
        radius_cells = int(math.ceil(radius_m / resolution))
        yy, xx = np.ogrid[
            -radius_cells:radius_cells + 1,
            -radius_cells:radius_cells + 1,
        ]
        kernel = (
            (xx * xx + yy * yy) * resolution * resolution
            <= radius_m * radius_m + 1e-9
        ).astype(np.uint8)
        cls._kernel_cache[key] = kernel
        return kernel

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

    def expanded_cells(self, cells, dynamic_clear_world=None):
        """按动态半径膨胀，并可清除当前机器人 footprint 内的动态格。"""
        expanded = set()
        offsets = self._disk_offsets(self.dynamic_inflation_radius)
        for cell_x, cell_y in cells:
            for dx, dy in offsets:
                nx, ny = cell_x + dx, cell_y + dy
                if self.in_bounds(nx, ny):
                    expanded.add((nx, ny))
        if dynamic_clear_world is not None:
            clear_cell = self.world_to_cell(
                dynamic_clear_world[0], dynamic_clear_world[1]
            )
            if clear_cell is not None:
                for dx, dy in self._disk_offsets(self.robot_radius):
                    expanded.discard((clear_cell[0] + dx, clear_cell[1] + dy))
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

    def traversable(self, cell, dynamic_blocked=None, blacklist_cells=None):
        if cell is None or not self.in_bounds(cell[0], cell[1]):
            return False
        if self.inflated_blocked[self.index(cell[0], cell[1])]:
            return False
        if dynamic_blocked is not None and cell in dynamic_blocked:
            return False
        return blacklist_cells is None or cell not in blacklist_cells

    def clearance_at_cell(self, cell):
        if cell is None or not self.in_bounds(cell[0], cell[1]):
            return 0.0
        return float(self.clearance_m[cell[1], cell[0]])

    def clearance_at_world(self, point):
        return self.clearance_at_cell(self.world_to_cell(point[0], point[1]))

    def unknown_at_world(self, point):
        cell = self.world_to_cell(point[0], point[1])
        return (
            cell is not None
            and self.unknown[self.index(cell[0], cell[1])]
        )

    def path_is_traversable(
        self, path, dynamic_cells=(), dynamic_clear_world=None,
        blacklist_cells=(),
    ):
        dynamic_blocked = self.expanded_cells(dynamic_cells, dynamic_clear_world)
        blacklist_cells = set(blacklist_cells)
        escaped_blacklist = False
        for index, point in enumerate(path):
            cell = self.world_to_cell(point[0], point[1])
            if not self.traversable(cell, dynamic_blocked):
                return False
            inside = cell in blacklist_cells
            if index == 0:
                escaped_blacklist = not inside
            elif escaped_blacklist and inside:
                return False
            elif not inside:
                escaped_blacklist = True
        return True

    def plan(
        self, start_world, goal_world, dynamic_cells=(), dynamic_clear_world=None,
        blacklist_cells=(),
    ):
        """以 A* 搜索膨胀后栅格；起终点或搜索不可达时返回 None。"""
        start = self.world_to_cell(start_world[0], start_world[1])
        goal = self.world_to_cell(goal_world[0], goal_world[1])
        dynamic_blocked = self.expanded_cells(dynamic_cells, dynamic_clear_world)
        blacklist_cells = set(blacklist_cells)
        # A trap may be remembered around the robot's current cell.  Release
        # only that policy constraint while it exits; real obstacles remain
        # hard blocked and blacklisted goals are always rejected.
        if (not self.traversable(start, dynamic_blocked)
                or not self.traversable(goal, dynamic_blocked)
                or goal in blacklist_cells):
            return None
        if start == goal:
            return self._remove_duplicate_points([tuple(start_world), tuple(goal_world)])

        start_state = (start, start not in blacklist_cells)
        frontier = [(0.0, 0.0, start_state)]
        came_from = {}
        g_score = {start_state: 0.0}
        expansions = 0
        while frontier:
            _, current_cost, current_state = heapq.heappop(frontier)
            if current_cost > g_score.get(current_state, float("inf")) + 1e-12:
                continue
            current, escaped_blacklist = current_state
            if current == goal:
                states = [current_state]
                while states[-1] != start_state:
                    states.append(came_from[states[-1]])
                states.reverse()
                cells = [state[0] for state in states]
                points = [tuple(start_world)]
                points.extend(self.cell_to_world(*cell) for cell in cells[1:-1])
                points.append(tuple(goal_world))
                return self._remove_duplicate_points(points)
            expansions += 1
            if expansions > self.max_expansions:
                return None
            for neighbor, neighbor_escaped, step_cost in self._blacklist_neighbors(
                    current, escaped_blacklist, dynamic_blocked, blacklist_cells):
                neighbor_state = (neighbor, neighbor_escaped)
                clearance_cost = self._clearance_cost(neighbor)
                tentative_cost = current_cost + step_cost + clearance_cost
                if tentative_cost >= g_score.get(neighbor_state, float("inf")):
                    continue
                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = tentative_cost
                heapq.heappush(
                    frontier,
                    (tentative_cost + self._heuristic(neighbor, goal),
                     tentative_cost, neighbor_state),
                )
        return None

    def _blacklist_neighbors(
        self, current, escaped_blacklist, dynamic_blocked, blacklist_cells,
    ):
        """Yield A* neighbors with one-way blacklist escape semantics."""
        cell_x, cell_y = current
        candidates = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            candidates.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])

        def allowed(cell):
            return (
                self.traversable(cell, dynamic_blocked)
                and not (escaped_blacklist and cell in blacklist_cells)
            )

        for dx, dy in candidates:
            neighbor = (cell_x + dx, cell_y + dy)
            if not allowed(neighbor):
                continue
            if dx and dy:
                if not allowed((cell_x + dx, cell_y)):
                    continue
                if not allowed((cell_x, cell_y + dy)):
                    continue
            yield (
                neighbor,
                escaped_blacklist or neighbor not in blacklist_cells,
                math.sqrt(2.0) if dx and dy else 1.0,
            )

    def plan_toward_unknown(
        self, start_world, goal_world, dynamic_cells=(), dynamic_clear_world=None,
        blacklist_cells=(),
    ):
        """Plan to the furthest known-free point toward an unknown goal."""
        if not self.unknown_at_world(goal_world):
            return None
        distance = math.hypot(
            goal_world[0] - start_world[0], goal_world[1] - start_world[1]
        )
        if distance <= self.resolution:
            return None
        sample_step = max(0.20, self.resolution)
        sample_count = int(math.ceil(distance / sample_step))
        for index in range(sample_count - 1, 0, -1):
            ratio = float(index) / float(sample_count)
            candidate = (
                start_world[0] + ratio * (goal_world[0] - start_world[0]),
                start_world[1] + ratio * (goal_world[1] - start_world[1]),
            )
            route = self.plan(
                start_world, candidate, dynamic_cells, dynamic_clear_world,
                blacklist_cells,
            )
            if route is not None and math.hypot(
                route[-1][0] - start_world[0],
                route[-1][1] - start_world[1],
            ) >= sample_step:
                return route
        return None

    def _neighbors(self, current, dynamic_blocked, blacklist_cells=()):
        cell_x, cell_y = current
        candidates = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            candidates.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])
        for dx, dy in candidates:
            neighbor = (cell_x + dx, cell_y + dy)
            if not self.traversable(neighbor, dynamic_blocked, blacklist_cells):
                continue
            if dx and dy:
                # 禁止斜向穿越两个相邻的阻塞格。
                if not self.traversable(
                        (cell_x + dx, cell_y), dynamic_blocked, blacklist_cells):
                    continue
                if not self.traversable(
                        (cell_x, cell_y + dy), dynamic_blocked, blacklist_cells):
                    continue
                yield neighbor, math.sqrt(2.0)
            else:
                yield neighbor, 1.0

    def _clearance_cost(self, cell):
        if self.clearance_soft_margin <= 0.0 or self.clearance_cost_weight <= 0.0:
            return 0.0
        preferred = self.inflation_radius + self.clearance_soft_margin
        clearance = self.clearance_at_cell(cell)
        if clearance >= preferred:
            return 0.0
        ratio = max(0.0, preferred - clearance) / self.clearance_soft_margin
        return self.clearance_cost_weight * ratio * ratio

    def _world_to_continuous_cell(self, x, y):
        dx, dy = float(x) - self.origin_x, float(y) - self.origin_y
        local_x = self._cos_origin_yaw * dx + self._sin_origin_yaw * dy
        local_y = -self._sin_origin_yaw * dx + self._cos_origin_yaw * dy
        return local_x / self.resolution, local_y / self.resolution

    @staticmethod
    def transform_footprint(pose, footprint):
        cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
        return tuple(
            (
                pose[0] + cos_yaw * point_x - sin_yaw * point_y,
                pose[1] + sin_yaw * point_x + cos_yaw * point_y,
            )
            for point_x, point_y in footprint
        )

    def footprint_cells(self, pose, footprint):
        """Rasterize a convex base-frame footprint at a map-frame pose."""
        world_polygon = self.transform_footprint(pose, footprint)
        grid_polygon = np.asarray(
            [self._world_to_continuous_cell(x, y) for x, y in world_polygon],
            dtype=np.float32,
        )
        min_x = int(math.floor(float(np.min(grid_polygon[:, 0])))) - 1
        max_x = int(math.ceil(float(np.max(grid_polygon[:, 0])))) + 1
        min_y = int(math.floor(float(np.min(grid_polygon[:, 1])))) - 1
        max_y = int(math.ceil(float(np.max(grid_polygon[:, 1])))) + 1
        if min_x < 0 or min_y < 0 or max_x >= self.width or max_y >= self.height:
            return None
        local = grid_polygon - np.asarray((min_x, min_y), dtype=np.float32)
        mask = np.zeros((max_y - min_y + 1, max_x - min_x + 1), dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.rint(local).astype(np.int32), 1)
        return {
            (min_x + int(local_x), min_y + int(local_y))
            for local_y, local_x in np.argwhere(mask != 0)
        }

    def footprint_metrics(
        self, pose, footprint, dynamic_cells=(), blacklist_cells=(),
    ):
        """Return ``(safe, min_clearance, colliding_cells)`` for one pose."""
        safe, min_clearance, physical, blacklist = self.footprint_collision_details(
            pose, footprint, dynamic_cells, blacklist_cells
        )
        return safe, min_clearance, physical | blacklist

    def footprint_collision_details(
        self, pose, footprint, dynamic_cells=(), blacklist_cells=(),
    ):
        """Separate physical collisions from policy-only blacklist overlap."""
        cells = self.footprint_cells(pose, footprint)
        if cells is None or not cells:
            return False, 0.0, {(-1, -1)}, set()
        dynamic_cells = set(dynamic_cells)
        blacklist_cells = set(blacklist_cells)
        physical = {
            cell for cell in cells
            if self._base_blocked_mask[cell[1], cell[0]]
            or cell in dynamic_cells
        }
        blacklist = cells & blacklist_cells
        min_clearance = min(self.clearance_at_cell(cell) for cell in cells)
        return not physical and not blacklist, min_clearance, physical, blacklist

    def footprint_blacklist_overlap(self, pose, footprint, blacklist_cells=()):
        """Return continuous polygon/blacklist overlap in grid-cell area units."""
        blacklist_cells = set(blacklist_cells)
        if not blacklist_cells:
            return 0.0
        world_polygon = self.transform_footprint(pose, footprint)
        polygon = np.asarray(
            [self._world_to_continuous_cell(x, y) for x, y in world_polygon],
            dtype=np.float32,
        )
        min_x = int(math.floor(float(np.min(polygon[:, 0]))))
        max_x = int(math.floor(float(np.max(polygon[:, 0]))))
        min_y = int(math.floor(float(np.min(polygon[:, 1]))))
        max_y = int(math.floor(float(np.max(polygon[:, 1]))))
        overlap = 0.0
        for cell_y in range(min_y, max_y + 1):
            for cell_x in range(min_x, max_x + 1):
                if (cell_x, cell_y) not in blacklist_cells:
                    continue
                cell_polygon = np.asarray((
                    (cell_x, cell_y), (cell_x + 1.0, cell_y),
                    (cell_x + 1.0, cell_y + 1.0), (cell_x, cell_y + 1.0),
                ), dtype=np.float32)
                area, _ = cv2.intersectConvexConvex(polygon, cell_polygon)
                overlap += max(0.0, float(area))
        return overlap

    @staticmethod
    def rollout_pose(start_pose, command, duration, linear_step=0.025, angular_step=0.05):
        """Integrate a constant body-frame ``(vx, vy, wz)`` command."""
        vx, vy, wz = (float(value) for value in command)
        speed = math.hypot(vx, vy)
        steps = max(
            1,
            int(math.ceil(speed * duration / max(linear_step, 1e-6))),
            int(math.ceil(abs(wz) * duration / max(angular_step, 1e-6))),
        )
        dt = float(duration) / float(steps)
        x, y, yaw = (float(value) for value in start_pose)
        poses = [(x, y, yaw)]
        for _ in range(steps):
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            x += (cos_yaw * vx - sin_yaw * vy) * dt
            y += (sin_yaw * vx + cos_yaw * vy) * dt
            yaw = normalize_angle(yaw + wz * dt)
            poses.append((x, y, yaw))
        return poses

    def trajectory_metrics(
        self, poses, footprint, dynamic_cells=(), blacklist_cells=(),
        allow_escape=False, allow_blacklist_escape=False,
        require_blacklist_exit=False,
    ):
        """Check every sampled footprint, optionally allowing monotonic escape."""
        if not poses:
            return False, 0.0
        _, min_clearance, initial_physical, initial_blacklist = (
            self.footprint_collision_details(
                poses[0], footprint, dynamic_cells, blacklist_cells
            )
        )
        if initial_physical and not allow_escape:
            return False, min_clearance
        if initial_blacklist and not allow_blacklist_escape:
            return False, min_clearance
        previous_physical_count = len(initial_physical)
        initial_physical_count = previous_physical_count
        previous_blacklist_overlap = self.footprint_blacklist_overlap(
            poses[0], footprint, blacklist_cells
        )
        initial_blacklist_overlap = previous_blacklist_overlap
        blacklist_escaped = initial_blacklist_overlap <= 1e-6
        if initial_blacklist_overlap > 1e-6 and not allow_blacklist_escape:
            return False, min_clearance
        minimum = min_clearance
        for pose in poses[1:]:
            _, clearance, physical, blacklist = self.footprint_collision_details(
                pose, footprint, dynamic_cells, blacklist_cells
            )
            minimum = min(minimum, clearance)
            if physical and (
                    not allow_escape or physical - initial_physical
                    or len(physical) > previous_physical_count):
                return False, minimum
            previous_physical_count = len(physical)
            blacklist_overlap = self.footprint_blacklist_overlap(
                pose, footprint, blacklist_cells
            )
            if blacklist_escaped and blacklist_overlap > 1e-6:
                return False, minimum
            if blacklist_overlap > previous_blacklist_overlap + 1e-4:
                return False, minimum
            if blacklist_overlap <= 1e-6:
                blacklist_escaped = True
            previous_blacklist_overlap = blacklist_overlap
        if (allow_escape and initial_physical_count
                and previous_physical_count >= initial_physical_count):
            return False, minimum
        if (require_blacklist_exit and initial_blacklist_overlap > 1e-6
                and not blacklist_escaped):
            return False, minimum
        return True, minimum

    def command_is_safe(
        self, start_pose, command, duration, footprint, dynamic_cells=(),
        blacklist_cells=(), allow_escape=False, allow_blacklist_escape=False,
    ):
        poses = self.rollout_pose(start_pose, command, duration)
        return self.trajectory_metrics(
            poses, footprint, dynamic_cells, blacklist_cells, allow_escape,
            allow_blacklist_escape,
        )

    def swept_path_metrics(
        self, path, footprint, dynamic_cells=(), blacklist_cells=(),
        final_yaw=None, initial_yaw=None, allow_blacklist_escape=False,
    ):
        """Validate the footprint along a polyline and optional final rotation."""
        if not path:
            return False, 0.0
        if len(path) > 1:
            path_yaw = math.atan2(
                path[1][1] - path[0][1], path[1][0] - path[0][0]
            )
        else:
            path_yaw = 0.0 if final_yaw is None else float(final_yaw)
        escaping_blacklist = (
            allow_blacklist_escape
            and initial_yaw is not None
            and self.footprint_blacklist_overlap(
                (path[0][0], path[0][1], float(initial_yaw)),
                footprint,
                blacklist_cells,
            ) > 1e-6
        )
        current_yaw = float(initial_yaw) if escaping_blacklist else path_yaw
        poses = []
        if initial_yaw is not None and not escaping_blacklist:
            start_yaw = float(initial_yaw)
            yaw_delta = normalize_angle(current_yaw - start_yaw)
            yaw_samples = max(1, int(math.ceil(abs(yaw_delta) / 0.05)))
            for sample in range(yaw_samples + 1):
                poses.append((
                    path[0][0], path[0][1],
                    normalize_angle(start_yaw + yaw_delta * sample / yaw_samples),
                ))
        else:
            poses.append((path[0][0], path[0][1], current_yaw))
        for index in range(1, len(path)):
            previous, point = path[index - 1], path[index]
            distance = math.hypot(point[0] - previous[0], point[1] - previous[1])
            samples = max(1, int(math.ceil(distance / 0.025)))
            for sample in range(1, samples + 1):
                ratio = float(sample) / float(samples)
                poses.append((
                    previous[0] + ratio * (point[0] - previous[0]),
                    previous[1] + ratio * (point[1] - previous[1]),
                    current_yaw,
                ))
            if index + 1 < len(path):
                next_yaw = math.atan2(
                    path[index + 1][1] - point[1],
                    path[index + 1][0] - point[0],
                )
            else:
                next_yaw = current_yaw if final_yaw is None else float(final_yaw)
            yaw_delta = normalize_angle(next_yaw - current_yaw)
            yaw_samples = max(1, int(math.ceil(abs(yaw_delta) / 0.05)))
            rotation_poses = [(
                    point[0], point[1],
                    normalize_angle(current_yaw + yaw_delta * sample / yaw_samples),
                ) for sample in range(1, yaw_samples + 1)]
            if escaping_blacklist:
                # The base is holonomic.  While still inside a remembered trap,
                # keep its current heading and translate out first; an in-place
                # turn can increase overlap even though the A* centerline exits.
                rotation_clear = all(
                    self.footprint_blacklist_overlap(
                        pose, footprint, blacklist_cells
                    ) <= 1e-6
                    for pose in rotation_poses
                )
                if rotation_clear:
                    poses.extend(rotation_poses)
                    current_yaw = next_yaw
                    escaping_blacklist = False
            else:
                poses.extend(rotation_poses)
                current_yaw = next_yaw
        return self.trajectory_metrics(
            poses, footprint, dynamic_cells, blacklist_cells,
            allow_blacklist_escape=allow_blacklist_escape,
            require_blacklist_exit=allow_blacklist_escape,
        )

    def choose_recovery(
        self, start_pose, footprint, dynamic_cells=(), target_distance=0.40,
        minimum_distance=0.30, maximum_distance=0.50, speed=0.10,
        excluded_maneuvers=(), blacklist_cells=(),
    ):
        """Prefer a safe backup, otherwise choose the clearer lateral escape."""
        durations = []
        for distance in (target_distance, minimum_distance, maximum_distance):
            if minimum_distance <= distance <= maximum_distance and distance not in durations:
                durations.append(distance)

        def candidate(maneuver, command_x, command_y):
            best = None
            for distance in durations:
                command = (command_x, command_y, 0.0)
                safe, clearance = self.command_is_safe(
                    start_pose,
                    command,
                    distance / speed,
                    footprint,
                    dynamic_cells,
                    blacklist_cells,
                    allow_escape=True,
                    allow_blacklist_escape=True,
                )
                if safe:
                    value = RecoveryCandidate(
                        maneuver, command_x, command_y, 0.0, distance, clearance
                    )
                    if best is None or (
                            abs(distance - target_distance), -clearance
                    ) < (
                            abs(best.distance - target_distance), -best.min_clearance
                    ):
                        best = value
            return best

        excluded_maneuvers = set(excluded_maneuvers)
        backup = (
            None if "BACKUP" in excluded_maneuvers
            else candidate("BACKUP", -abs(speed), 0.0)
        )
        if backup is not None:
            return backup
        laterals = tuple(filter(None, (
            None if "STRAFE_LEFT" in excluded_maneuvers
            else candidate("STRAFE_LEFT", 0.0, abs(speed)),
            None if "STRAFE_RIGHT" in excluded_maneuvers
            else candidate("STRAFE_RIGHT", 0.0, -abs(speed)),
        )))
        if not laterals:
            return None
        return max(laterals, key=lambda value: (value.min_clearance, value.distance))

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
