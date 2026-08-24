from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped
from hazard_guard_interfaces.msg import HazardDetection
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .coverage import MapGrid
from .environment import (
    load_heat_source_ids,
    load_world_assets,
    repository_commit,
    resolve_environment_root,
)
from .metrics import PoseSample
from .report import BenchmarkSession, default_storage_root


COLLECTING_STATES = {
    "preparing",
    "running",
    "executing",
    "aligning",
    "dwelling",
    "safety_paused",
}
TERMINAL_STATES = {"completed", "failed", "canceled"}


def _stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _yaw(orientation: Any) -> float:
    return math.atan2(
        2.0
        * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        ),
        1.0
        - 2.0
        * (
            float(orientation.y) * float(orientation.y)
            + float(orientation.z) * float(orientation.z)
        ),
    )


def _repository_from_workspace() -> Path | None:
    configured = os.getenv("HAZARD_GUARD_WORKSPACE", "").strip()
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if (candidate / ".git").exists():
            return candidate
    for parent in Path(__file__).resolve().parents:
        if (parent / ".git").exists():
            return parent
    return None


class PatrolBenchmarkNode(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_patrol_benchmark")
        self._declare_parameters()
        environment_root = resolve_environment_root(
            str(self.get_parameter("simulation_env_path").value)
        )
        self._assets = load_world_assets(
            environment_root,
            str(self.get_parameter("world_id").value),
        )
        self._map_grid = MapGrid.from_yaml(self._assets.map_path)
        self._expected_heat_sources = load_heat_source_ids(
            self._assets.heat_sources_path
        )
        storage_value = str(self.get_parameter("storage_path").value).strip()
        self._storage_root = (
            Path(storage_value).expanduser().resolve()
            if storage_value
            else default_storage_root()
        )
        self._storage_root.mkdir(parents=True, exist_ok=True)
        self._session: BenchmarkSession | None = None
        self._mission: dict[str, Any] = {}
        self._latest_ground_truth: PoseSample | None = None
        self._latest_estimate: PoseSample | None = None
        self._ground_truth_frame = ""
        self._estimate_frame = ""
        self._configure_subscriptions()
        self.get_logger().info(
            "Patrol benchmark ready: "
            f"world={self._assets.world_id}, map={self._assets.map_path.name}, "
            f"results={self._storage_root}"
        )

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "world_id": "real_factory",
            "simulation_env_path": "",
            "storage_path": "",
            "ground_truth_topic": "/odom",
            "amcl_pose_topic": "/amcl_pose",
            "collision_topic": "",
            "recovery_topic": "",
            "compare_localization": False,
            "ground_truth_offset_x": 0.0,
            "ground_truth_offset_y": 0.0,
            "ground_truth_offset_yaw": 0.0,
            "inspection_radius_m": 1.0,
            "robot_clearance_m": 0.0,
            "minimum_step_m": 0.01,
            "maximum_step_m": 2.0,
            "localization_max_age_sec": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _configure_subscriptions(self) -> None:
        mission_qos = QoSProfile(depth=1)
        mission_qos.reliability = ReliabilityPolicy.RELIABLE
        mission_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            "/hazard_guard/mission/status",
            self._on_mission,
            mission_qos,
        )
        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._on_ground_truth,
            sensor_qos,
        )
        self.create_subscription(
            HazardDetection,
            "/hazard_guard/thermal_detections",
            self._on_detection,
            20,
        )
        if bool(self.get_parameter("compare_localization").value):
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("amcl_pose_topic").value),
                self._on_amcl_pose,
                20,
            )
        collision_topic = str(self.get_parameter("collision_topic").value).strip()
        if collision_topic:
            self.create_subscription(Bool, collision_topic, self._on_collision, 20)
        recovery_topic = str(self.get_parameter("recovery_topic").value).strip()
        if recovery_topic:
            self.create_subscription(String, recovery_topic, self._on_recovery, 20)

    def _metadata(self) -> dict[str, Any]:
        robot_repository = _repository_from_workspace()
        return {
            "ground_truth_topic": str(
                self.get_parameter("ground_truth_topic").value
            ),
            "ground_truth_frame": self._ground_truth_frame or None,
            "localization_comparison_enabled": bool(
                self.get_parameter("compare_localization").value
            ),
            "reproducibility": {
                "robot_commit": repository_commit(robot_repository)
                if robot_repository
                else None,
                "simulation_env_commit": repository_commit(
                    self._assets.repository_root
                ),
                "world_path": str(
                    self._assets.world_path.relative_to(
                        self._assets.repository_root
                    )
                ),
                "map_path": str(
                    self._assets.map_path.relative_to(
                        self._assets.repository_root
                    )
                ),
                "spawn": self._assets.spawn,
            },
        }

    def _new_session(self, mission: dict[str, Any]) -> BenchmarkSession:
        return BenchmarkSession(
            self._storage_root,
            mission,
            self._assets,
            self._map_grid,
            self._expected_heat_sources,
            self._metadata(),
            inspection_radius_m=max(
                0.0, float(self.get_parameter("inspection_radius_m").value)
            ),
            robot_clearance_m=max(
                0.0, float(self.get_parameter("robot_clearance_m").value)
            ),
            minimum_step_m=max(
                0.0, float(self.get_parameter("minimum_step_m").value)
            ),
            maximum_step_m=max(
                0.01, float(self.get_parameter("maximum_step_m").value)
            ),
        )

    def _on_mission(self, message: String) -> None:
        try:
            mission = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(mission, dict):
            return
        status = str(mission.get("status") or "idle")
        mission_id = str(mission.get("mission_id") or "")
        current_id = str(self._mission.get("mission_id") or "")
        if self._session is not None and mission_id and current_id != mission_id:
            summary = self._session.finalize("interrupted")
            self.get_logger().warning(
                f"Mission changed; finalized benchmark {summary['id']}"
            )
            self._session = None
        self._mission = mission
        if self._session is None and mission_id and status in COLLECTING_STATES:
            self._session = self._new_session(mission)
            self.get_logger().info(f"Benchmark started: {mission_id}")
        if self._session is None:
            return
        self._session.observe_mission(mission)
        if status in TERMINAL_STATES:
            summary = self._session.finalize(status)
            self.get_logger().info(
                "Benchmark completed: "
                f"{summary['id']} ({summary['time']['simulation_sec']} sec, "
                f"{summary['coverage']['coverage_percent']}%)"
            )
            self._session = None

    def _on_ground_truth(self, message: Odometry) -> None:
        if self._session is None:
            return
        self._ground_truth_frame = str(message.header.frame_id)
        timestamp = _stamp_seconds(message.header.stamp)
        if timestamp <= 0.0:
            timestamp = self.get_clock().now().nanoseconds / 1_000_000_000.0
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        raw_x = float(position.x)
        raw_y = float(position.y)
        raw_yaw = _yaw(orientation)
        offset_x = float(self.get_parameter("ground_truth_offset_x").value)
        offset_y = float(self.get_parameter("ground_truth_offset_y").value)
        offset_yaw = float(self.get_parameter("ground_truth_offset_yaw").value)
        cosine = math.cos(offset_yaw)
        sine = math.sin(offset_yaw)
        sample = PoseSample(
            timestamp_sec=timestamp,
            x=offset_x + cosine * raw_x - sine * raw_y,
            y=offset_y + sine * raw_x + cosine * raw_y,
            yaw=raw_yaw + offset_yaw,
        )
        self._latest_ground_truth = sample
        self._session.add_ground_truth(sample)
        if not bool(self.get_parameter("compare_localization").value):
            return
        estimate = self._latest_estimate
        if estimate is None:
            return
        maximum_age = max(
            0.0, float(self.get_parameter("localization_max_age_sec").value)
        )
        if abs(sample.timestamp_sec - estimate.timestamp_sec) <= maximum_age:
            self._session.add_localization(sample, estimate)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._estimate_frame = str(message.header.frame_id)
        pose = message.pose.pose
        self._latest_estimate = PoseSample(
            timestamp_sec=_stamp_seconds(message.header.stamp),
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=_yaw(pose.orientation),
        )

    def _on_detection(self, message: HazardDetection) -> None:
        if self._session is not None and bool(message.simulated):
            self._session.add_detection(str(message.detection_id))

    def _on_collision(self, message: Bool) -> None:
        if self._session is not None:
            self._session.observe_collision(bool(message.data))

    def _on_recovery(self, _message: String) -> None:
        if self._session is not None:
            self._session.observe_recovery()

    def destroy_node(self) -> bool:
        if self._session is not None:
            self._session.finalize("interrupted")
            self._session = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PatrolBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
