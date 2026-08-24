#!/usr/bin/env python3
"""Drive the robot around an unknown facility until the 2D map stops growing.

Frontier exploration, the small version: a frontier is a known-free cell that
touches an unknown one, so the set of frontiers is exactly the border of what
has been seen. Send Nav2 at the nearest one, let SLAM Toolbox and RTAB-Map
widen the map from wherever the robot ends up, repeat. When no frontier is
left the building has been covered and the run stops.

The node owns no motion of its own - Nav2 is the only publisher on /cmd_vel -
so this composes with whatever started the SLAM stack, the WebUI included. It
needs a map (`/map`) and a navigation stack; `explore.launch.py` brings the
second one up next to a mapping session that is already running.

    ros2 launch hazard_guard_simulation explore.launch.py
    python3 scripts/auto_explore.py selftest      # frontier maths, no ROS
"""

from __future__ import annotations

import math
import sys
from typing import Iterator

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener


UNKNOWN = -1
# SLAM Toolbox writes occupancy as 0-100. Anything under this is floor the
# robot may stand on; the costmap keeps its own, stricter opinion.
FREE_BELOW = 25


class FrontierMap:
    """An OccupancyGrid, with the frontier arithmetic and nothing else.

    Kept free of ROS types so the clustering can be checked without a running
    system - see selftest().
    """

    def __init__(self, data, width: int, height: int, resolution: float,
                 origin_x: float, origin_y: float) -> None:
        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    @classmethod
    def from_message(cls, message: OccupancyGrid) -> "FrontierMap":
        info = message.info
        return cls(list(message.data), info.width, info.height,
                   info.resolution, info.origin.position.x,
                   info.origin.position.y)

    def value(self, x: int, y: int) -> int:
        return self.data[y * self.width + x]

    def to_world(self, x: int, y: int) -> tuple:
        # Cell centres, not corners: a goal on a corner sits on the boundary
        # between a free and an unknown cell, and the planner reads that as
        # unknown.
        return (self.origin_x + (x + 0.5) * self.resolution,
                self.origin_y + (y + 0.5) * self.resolution)

    def _neighbours(self, x: int, y: int, diagonal: bool) -> Iterator:
        steps = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if diagonal:
            steps += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in steps:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                yield nx, ny

    def is_frontier(self, x: int, y: int) -> bool:
        value = self.value(x, y)
        if value < 0 or value >= FREE_BELOW:
            return False
        # Edge-adjacency only. A diagonal touch leaves the robot aiming at a
        # corner it cannot see past, which Nav2 then fails to reach.
        return any(self.value(nx, ny) == UNKNOWN
                   for nx, ny in self._neighbours(x, y, diagonal=False))

    def clusters(self, minimum_cells: int = 6) -> list:
        """Frontier groups as (world_x, world_y, cell_count), largest first.

        The returned point is the cluster member nearest its centroid, never
        the centroid itself - a horseshoe-shaped frontier has its centroid in
        the middle of a wall.
        """
        seen = set()
        found = []
        for index, value in enumerate(self.data):
            if value < 0 or value >= FREE_BELOW or index in seen:
                continue
            x, y = index % self.width, index // self.width
            if not self.is_frontier(x, y):
                continue
            cells = []
            queue = [(x, y)]
            seen.add(index)
            while queue:
                cx, cy = queue.pop()
                cells.append((cx, cy))
                for nx, ny in self._neighbours(cx, cy, diagonal=True):
                    key = ny * self.width + nx
                    if key in seen or not self.is_frontier(nx, ny):
                        continue
                    seen.add(key)
                    queue.append((nx, ny))
            if len(cells) < minimum_cells:
                continue
            mean_x = sum(cell[0] for cell in cells) / len(cells)
            mean_y = sum(cell[1] for cell in cells) / len(cells)
            best = min(cells, key=lambda cell: (cell[0] - mean_x) ** 2
                       + (cell[1] - mean_y) ** 2)
            found.append(self.to_world(*best) + (len(cells),))
        found.sort(key=lambda item: item[2], reverse=True)
        return found


class AutoExplore(Node):
    """Send Nav2 at the nearest frontier until the map stops growing."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_auto_explore")
        self.declare_parameter("minimum_frontier_cells", 6)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("run_timeout_sec", 1200.0)
        # Two frontiers this close are the same doorway seen twice; a goal that
        # failed once blocks its neighbourhood rather than just its own cell.
        self.declare_parameter("blacklist_radius_m", 0.5)
        self.declare_parameter("robot_frame", "base_footprint")

        self._map: FrontierMap | None = None
        self._blacklist: list = []
        self._goal_handle = None
        self._goal_point: tuple | None = None
        self._goal_deadline = None
        self._started_at = None
        self.finished = False

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._navigator = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # SLAM Toolbox latches /map: without transient-local durability a late
        # subscriber waits for the next full publish, which can be a minute.
        self.create_subscription(
            OccupancyGrid, "map", self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(2.0, self._tick)
        self.get_logger().info("자동 탐사 대기 중 - /map 과 Nav2 를 기다립니다.")

    # --- inputs ------------------------------------------------------------

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = FrontierMap.from_message(message)

    def _robot_position(self) -> tuple | None:
        try:
            transform = self._buffer.lookup_transform(
                "map", self.get_parameter("robot_frame").value,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception:  # tf2 raises a family of lookup errors
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _blacklisted(self, point: tuple) -> bool:
        radius = self.get_parameter("blacklist_radius_m").value
        return any(math.dist(point, blocked) < radius
                   for blocked in self._blacklist)

    # --- the loop ----------------------------------------------------------

    def _tick(self) -> None:
        if self.finished:
            return
        now = self.get_clock().now()
        if self._started_at is None:
            self._started_at = now
        elapsed = (now - self._started_at).nanoseconds / 1e9
        if elapsed > self.get_parameter("run_timeout_sec").value:
            self._stop(f"제한 시간 {elapsed:.0f}s 초과 - 탐사를 멈춥니다.")
            return

        if self._goal_handle is not None:
            if self._goal_deadline is not None and now > self._goal_deadline:
                self.get_logger().warn("목표 시간 초과 - 취소하고 다음 프론티어로.")
                self._goal_handle.cancel_goal_async()
                self._finish_goal(reached=False)
            return

        if self._map is None:
            return
        if not self._navigator.server_is_ready():
            self.get_logger().info("Nav2 navigate_to_pose 를 기다리는 중...",
                                   throttle_duration_sec=10.0)
            return

        position = self._robot_position()
        if position is None:
            self.get_logger().info("map -> base_footprint TF 대기 중...",
                                   throttle_duration_sec=10.0)
            return

        candidates = [
            cluster for cluster in self._map.clusters(
                self.get_parameter("minimum_frontier_cells").value)
            if not self._blacklisted(cluster[0:2])
        ]
        if not candidates:
            self._stop("남은 프론티어 없음 - 지도가 닫혔습니다. 탐사 완료.")
            return

        target = min(candidates,
                     key=lambda cluster: math.dist(position, cluster[0:2]))
        self._send_goal(target, position)

    def _send_goal(self, target: tuple, position: tuple) -> None:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        # Face the way it travels, so the cameras see the new space on arrival
        # instead of the corridor already mapped.
        heading = math.atan2(target[1] - position[1], target[0] - position[0])
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)
        goal.pose = pose

        self._goal_point = target[0:2]
        self._goal_deadline = self.get_clock().now() + Duration(
            seconds=float(self.get_parameter("goal_timeout_sec").value))
        self.get_logger().info(
            f"프론티어 ({target[0]:.2f}, {target[1]:.2f}) 셀 {target[2]}개 "
            f"- 거리 {math.dist(position, target[0:2]):.2f} m")
        self._navigator.send_goal_async(goal).add_done_callback(self._on_accept)

    def _on_accept(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav2 가 목표를 거부했습니다 - 제외 목록에 넣습니다.")
            self._finish_goal(reached=False)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        status = future.result().status
        reached = status == GoalStatus.STATUS_SUCCEEDED
        if not reached:
            self.get_logger().warn(f"목표 실패 (status {status}) - 제외합니다.")
        self._finish_goal(reached=reached)

    def _finish_goal(self, *, reached: bool) -> None:
        # A reached frontier disappears on the next map update; a failed one
        # does not, so only failures need remembering or the node retries the
        # same unreachable corner forever.
        if not reached and self._goal_point is not None:
            self._blacklist.append(self._goal_point)
        self._goal_handle = None
        self._goal_point = None
        self._goal_deadline = None

    def _stop(self, reason: str) -> None:
        self.finished = True
        self.get_logger().info(reason)


# --- self-check --------------------------------------------------------------

def selftest() -> None:
    # A 6x4 map: left half free, right half unknown, wall along the top.
    # The frontier is the free column that touches the unknown one.
    width, height = 6, 4
    cells = []
    for y in range(height):
        for x in range(width):
            if y == height - 1:
                cells.append(100)
            elif x < 3:
                cells.append(0)
            else:
                cells.append(UNKNOWN)
    grid = FrontierMap(cells, width, height, 0.5, -1.0, -2.0)

    assert grid.is_frontier(2, 0), "미지 영역에 붙은 자유 셀이 프론티어여야 함"
    assert not grid.is_frontier(0, 0), "미지와 떨어진 자유 셀은 프론티어가 아님"
    assert not grid.is_frontier(3, 0), "미지 셀 자체는 프론티어가 아님"
    assert not grid.is_frontier(0, 3), "점유 셀은 프론티어가 아님"

    clusters = grid.clusters(minimum_cells=1)
    assert len(clusters) == 1, f"프론티어 무리는 하나여야 함: {clusters}"
    x, y, count = clusters[0]
    assert count == 3, f"프론티어 셀 3개여야 함: {count}"
    # Column 2, rows 0-2 -> centre cell (2, 1) -> world (-1 + 2.5*0.5, -2 + 1.5*0.5)
    assert abs(x - 0.25) < 1e-9 and abs(y - (-1.25)) < 1e-9, f"좌표 {x}, {y}"

    # Small specks are sensor noise, not somewhere to drive.
    assert grid.clusters(minimum_cells=4) == [], "최소 크기 필터가 동작해야 함"

    # A fully known map is a finished one: no frontier means the loop stops,
    # which is the only thing that ends the run normally.
    closed = FrontierMap([0] * (width * height), width, height, 0.5, 0.0, 0.0)
    assert closed.clusters(minimum_cells=1) == [], "미지 영역이 없으면 프론티어도 없음"

    print("selftest 통과 - 프론티어 검출, 무리 짓기, 최소 크기, 종료 조건")


def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        selftest()
        return
    rclpy.init()
    node = AutoExplore()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
