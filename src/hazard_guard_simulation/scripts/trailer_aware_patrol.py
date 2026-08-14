#!/usr/bin/env python3
"""Scan the plant equipment, then drive a fixed-dispenser-safe outer lap."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class RoutePoint:
    x: float
    y: float
    yaw: float

@dataclass(frozen=True)
class ScanStation:
    name: str
    equipment_id: str
    x: float
    y: float
    yaws: tuple[float, ...]



def normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def build_route(spacing: float = 0.065) -> list[RoutePoint]:
    """CCW rounded rectangle around the 4.70 x 2.23 m plant."""
    plant_x = 2.35
    plant_y = 1.115
    radius = 0.27
    start_x = 0.0975
    points: list[RoutePoint] = []

    def append(x: float, y: float, yaw: float) -> None:
        if points and math.hypot(x - points[-1].x, y - points[-1].y) < 1e-7:
            return
        points.append(RoutePoint(x, y, normalize(yaw)))

    def line(x0: float, y0: float, x1: float, y1: float, yaw: float) -> None:
        distance = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, math.ceil(distance / spacing))
        for step in range(steps + 1):
            ratio = step / steps
            append(x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio, yaw)

    def arc(cx: float, cy: float, start_yaw: float, end_yaw: float) -> None:
        arc_length = radius * abs(end_yaw - start_yaw)
        steps = max(6, math.ceil(arc_length / spacing))
        for step in range(steps + 1):
            yaw = start_yaw + (end_yaw - start_yaw) * step / steps
            append(cx + radius * math.sin(yaw), cy - radius * math.cos(yaw), yaw)

    bottom = -plant_y - radius
    right = plant_x + radius
    top = plant_y + radius
    left = -plant_x - radius
    line(start_x, bottom, plant_x, bottom, 0.0)
    arc(plant_x, -plant_y, 0.0, math.pi / 2.0)
    line(right, -plant_y, right, plant_y, math.pi / 2.0)
    arc(plant_x, plant_y, math.pi / 2.0, math.pi)
    line(plant_x, top, -plant_x, top, math.pi)
    arc(-plant_x, plant_y, math.pi, 3.0 * math.pi / 2.0)
    line(left, plant_y, left, -plant_y, -math.pi / 2.0)
    arc(-plant_x, -plant_y, 3.0 * math.pi / 2.0, 2.0 * math.pi)
    line(-plant_x, bottom, start_x, bottom, 0.0)
    if math.hypot(points[-1].x - points[0].x, points[-1].y - points[0].y) < 1e-7:
        points.pop()
    return points


class TrailerAwarePatrol(Node):
    def __init__(self) -> None:
        super().__init__("trailer_aware_patrol")
        self.declare_parameter("equipment_scan", True)
        self.points = build_route()
        self.pose: tuple[float, float, float] | None = None
        self.route_initialized = False
        self.progress = 0
        self.nominal_speed = 0.09
        self.lookahead = 0.13
        self.last_motion_pose: tuple[float, float] | None = None
        self.last_motion_time = time.monotonic()
        self.last_report_quarter = -1
        # The lower-right rounded corner has enough clearance for the fixed
        # rear dispenser. From here the four configured equipment heat
        # sources span about 48 degrees, so a slow deterministic yaw sweep can
        # inspect the whole plant without entering its narrow inner aisles.
        degrees = math.radians
        self.scan_stations = (
            ScanStation("hydraulic tank", "baler_hydraulic_tank", 1.130, -1.385, (degrees(90.0),)),
            ScanStation("shredder motor", "primary_shredder_motor", 2.541, -1.306, (
                degrees(122.0), degrees(148.0), degrees(174.0),
            )),
            ScanStation("processor pump", "secondary_processor_pump", 2.540, 1.300, (degrees(-150.0),)),
            ScanStation("waste pile", "bunker_waste_pile", -2.600, -0.700, (degrees(0.0),)),
        )
        self.scan_station_index = 0
        self.scan_trigger_distance = 0.11
        self.scan_index = 0
        self.scan_started = False
        self.scan_complete = not bool(self.get_parameter("equipment_scan").value)
        self.scan_hold_started: float | None = None
        self.scan_hold_seconds = 3.0
        self.scan_angular_speed = 0.20
        self.visit_started = False
        self.visit_start_requested = False
        self.lap_complete = False
        self.visit_record_requested = False
        self.done = False

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.inspection_pub = self.create_publisher(
            String, "/hazard_guard/thermal/inspection_control", 10
        )
        self.visit_start_client = self.create_client(
            Trigger, "/hazard_guard/thermal/start_visit"
        )
        self.visit_record_client = self.create_client(
            Trigger, "/hazard_guard/thermal/record_visit"
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, odom_qos)
        self.create_timer(0.05, self.control)
        self.get_logger().info(
            f"Loaded {len(self.points)} fixed-dispenser route points; waiting for odometry"
        )

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.pose = (position.x, position.y, yaw)
        if not self.route_initialized:
            nearest = min(
                range(len(self.points)),
                key=lambda index: math.hypot(
                    position.x - self.points[index].x,
                    position.y - self.points[index].y,
                ),
            )
            self.points = self.points[nearest:] + self.points[:nearest]
            self.route_initialized = True
            self.last_motion_pose = (position.x, position.y)
            self.last_motion_time = time.monotonic()
            point = self.points[0]
            self.get_logger().info(
                f"Route joined at ({point.x:.3f}, {point.y:.3f}), "
                f"yaw {math.degrees(point.yaw):.1f} deg"
            )

    def publish_inspection_control(
        self, action: str, equipment_id: str | None = None
    ) -> None:
        message = String()
        payload = {"action": action}
        if equipment_id:
            payload["equipment_id"] = equipment_id
        message.data = json.dumps(payload, separators=(",", ":"))
        self.inspection_pub.publish(message)

    def start_thermal_visit(self) -> None:
        if self.visit_started or self.visit_start_requested:
            return
        if not self.visit_start_client.service_is_ready():
            return
        self.visit_start_requested = True
        future = self.visit_start_client.call_async(Trigger.Request())

        def completed(done_future) -> None:
            response = done_future.result()
            if response is None or not response.success:
                self.get_logger().error("FAIL: could not start thermal patrol visit")
                self.done = True
                return
            self.visit_started = True
            self.get_logger().info("Thermal patrol visit started")

        future.add_done_callback(completed)

    def record_thermal_visit(self) -> None:
        if self.visit_record_requested:
            return
        if not self.visit_record_client.service_is_ready():
            return
        self.visit_record_requested = True
        future = self.visit_record_client.call_async(Trigger.Request())

        def completed(done_future) -> None:
            response = done_future.result()
            if response is None or not response.success:
                detail = response.message if response is not None else "no response"
                self.get_logger().error(
                    f"FAIL: thermal patrol visit was not recorded: {detail}"
                )
            else:
                self.get_logger().info(
                    f"Thermal patrol visit recorded once for this lap: {response.message}"
                )
                self.get_logger().info(
                    "PASS: one complete trailer-aware patrol lap finished"
                )
            self.done = True

        future.add_done_callback(completed)

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def target_index(self) -> int:
        distance = 0.0
        index = self.progress
        while distance < self.lookahead:
            current = self.points[index % len(self.points)]
            following = self.points[(index + 1) % len(self.points)]
            distance += math.hypot(following.x - current.x, following.y - current.y)
            index += 1
        return index

    def update_progress(self, x: float, y: float) -> None:
        count = len(self.points)
        offsets = range(0, min(31, count))
        best_offset = min(
            offsets,
            key=lambda offset: math.hypot(
                x - self.points[(self.progress + offset) % count].x,
                y - self.points[(self.progress + offset) % count].y,
            ),
        )
        self.progress += best_offset

    def control_equipment_scan(self, x: float, y: float, yaw: float) -> None:
        """Perform the fixed scan sequence regardless of detection results."""
        station = self.scan_stations[self.scan_station_index]
        if not self.scan_started:
            self.scan_started = True
            self.scan_index = 0
            self.scan_hold_started = None
            self.stop()
            self.publish_inspection_control(
                "focus_equipment", station.equipment_id
            )
            self.get_logger().info(
                f"Starting {station.name} scan at "
                f"({station.x:.3f}, {station.y:.3f})"
            )

        target_yaw = station.yaws[self.scan_index]
        yaw_error = normalize(target_yaw - yaw)
        if (
            self.scan_hold_started is None
            and abs(yaw_error) > math.radians(2.0)
        ):
            self.scan_hold_started = None
            turn_speed = clamp(
                1.25 * abs(yaw_error),
                0.10,
                self.scan_angular_speed,
            )
            command = Twist()
            command.angular.z = math.copysign(turn_speed, yaw_error)
            self.cmd_pub.publish(command)
            return

        self.stop()
        now = time.monotonic()
        if self.scan_hold_started is None:
            self.scan_hold_started = now
            self.get_logger().info(
                f"Holding equipment view at {math.degrees(target_yaw):.0f} deg"
            )
            return
        if now - self.scan_hold_started < self.scan_hold_seconds:
            return

        self.scan_index += 1
        self.scan_hold_started = None
        if self.scan_index < len(station.yaws):
            return

        self.scan_station_index += 1
        self.publish_inspection_control("clear_focus")
        self.scan_started = False
        self.last_motion_pose = (x, y)
        self.last_motion_time = now
        if self.scan_station_index >= len(self.scan_stations):
            self.scan_complete = True
            self.get_logger().info(
                "All equipment scans complete; resuming the outer patrol route"
            )
        else:
            self.get_logger().info(
                f"{station.name} scan complete; continuing to the next station"
            )

    def control(self) -> None:
        if self.done or self.pose is None or not self.route_initialized:
            return
        if not self.visit_started:
            self.stop()
            self.start_thermal_visit()
            return
        if self.lap_complete:
            self.stop()
            self.record_thermal_visit()
            return
        x, y, yaw = self.pose
        self.update_progress(x, y)
        count = len(self.points)

        scan_distance = math.inf
        if not self.scan_complete:
            station = self.scan_stations[self.scan_station_index]
            scan_distance = math.hypot(x - station.x, y - station.y)
        if not self.scan_complete and (
            self.scan_started or scan_distance < self.scan_trigger_distance
        ):
            self.control_equipment_scan(x, y, yaw)
            return

        if self.progress >= count:
            finish = self.points[0]
            if math.hypot(x - finish.x, y - finish.y) < 0.14:
                self.stop()
                self.lap_complete = True
                self.get_logger().info(
                    "One lap complete; recording one aggregated thermal visit"
                )
                return

        target = self.points[self.target_index() % count]
        dx = target.x - x
        dy = target.y - y
        target_bearing = math.atan2(dy, dx)
        bearing_error = normalize(target_bearing - yaw)
        yaw_error = normalize(target.yaw - yaw)
        speed_scale = 1.0 - 0.55 * min(abs(bearing_error) / 0.8, 1.0)
        speed = max(0.04, self.nominal_speed * speed_scale)
        angular = clamp(1.80 * bearing_error + 0.35 * yaw_error, -0.50, 0.50)

        command = Twist()
        command.linear.x = speed
        command.angular.z = angular
        self.cmd_pub.publish(command)

        now = time.monotonic()
        if self.last_motion_pose is not None:
            moved = math.hypot(x - self.last_motion_pose[0], y - self.last_motion_pose[1])
            if moved > 0.035:
                self.last_motion_pose = (x, y)
                self.last_motion_time = now
            elif now - self.last_motion_time > 7.0:
                self.stop()
                self.done = True
                self.get_logger().error(
                    f"FAIL: robot stalled near ({x:.3f}, {y:.3f}); possible collision"
                )
                return

        quarter = min(4, int(4.0 * self.progress / count))
        if quarter > self.last_report_quarter:
            self.last_report_quarter = quarter
            self.get_logger().info(
                f"Lap progress {quarter * 25}% at ({x:.3f}, {y:.3f}), "
                f"yaw {math.degrees(yaw):.1f} deg"
            )


def main() -> None:
    rclpy.init()
    node = TrailerAwarePatrol()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
