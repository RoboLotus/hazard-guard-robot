"""ROS 2 entry point for profile-driven rosbag2 recording."""

from __future__ import annotations

from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .metrics import DriveMetrics
from .preflight import PreflightError, run_preflight
from .profiles import ProfileError, load_profile_document, resolve_profile
from .session import BagSession, SessionError, create_session_paths
from .summary import summarize_sqlite_bag


def default_storage_root() -> Path:
    return Path.home() / ".local" / "share" / "hazard_guard" / "bags"


class BagSessionManager(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_bag_session_manager")
        package_share = Path(get_package_share_directory("hazard_guard_bag_recorder"))
        self.declare_parameter("profile", "navigation-core")
        self.declare_parameter("profiles_path", str(package_share / "config" / "profiles.json"))
        self.declare_parameter("storage_root", str(default_storage_root()))
        self.declare_parameter("session_name", "field-session")
        self.declare_parameter("storage_id", "sqlite3")
        self.declare_parameter("minimum_free_gb", 2.0)
        self.declare_parameter("max_duration_seconds", 0.0)
        self.declare_parameter("max_size_gb", 0.0)
        self.declare_parameter("allow_experimental", False)
        self.declare_parameter("auto_start", False)

        self._session: BagSession | None = None
        self._finalized = True
        self._metrics = DriveMetrics()
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_subscription(String, "/hazard_guard/mission/status", self._on_mission_status, 10)
        self.create_service(Trigger, "/hazard_guard/bag/start", self._on_start)
        self.create_service(Trigger, "/hazard_guard/bag/stop", self._on_stop)
        self._status_publisher = self.create_publisher(String, "/hazard_guard/bag/status", 10)
        self.create_timer(1.0, self._on_tick)
        self.publish_status("idle")
        if bool(self.get_parameter("auto_start").value):
            self._start_session()

    def _on_odometry(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1_000_000_000
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        self._metrics.observe_odometry(position.x, position.y, velocity.x, velocity.y, stamp)

    def _on_mission_status(self, message: String) -> None:
        self._metrics.latest_mission_status = message.data[:512]

    def _available_topic_names(self) -> set[str]:
        return {name for name, _types in self.get_topic_names_and_types()}

    def _start_session(self) -> tuple[bool, str]:
        if self._session is not None and self._session.is_running():
            return False, "ROS Bag session is already recording"
        try:
            profile_name = str(self.get_parameter("profile").value)
            profile = resolve_profile(
                profile_name,
                load_profile_document(Path(str(self.get_parameter("profiles_path").value))),
            )
            preflight = run_preflight(
                profile,
                self._available_topic_names(),
                Path(str(self.get_parameter("storage_root").value)),
                minimum_free_bytes=int(float(self.get_parameter("minimum_free_gb").value) * 1024**3),
                allow_experimental=bool(self.get_parameter("allow_experimental").value),
            )
            if not preflight.can_start:
                return False, f"required topics missing: {', '.join(preflight.missing_required)}"
            self._metrics = DriveMetrics()
            self._session = BagSession(
                create_session_paths(
                    Path(str(self.get_parameter("storage_root").value)),
                    str(self.get_parameter("session_name").value),
                ),
                profile.name,
                preflight,
                storage_id=str(self.get_parameter("storage_id").value),
            )
            self._session.start()
        except (ProfileError, PreflightError, SessionError, ValueError) as exc:
            self.get_logger().error(str(exc))
            return False, str(exc)
        self._finalized = False
        self.publish_status("recording")
        return True, f"recording: {self._session.paths.session_dir}"

    def _stop_session(self, reason: str) -> tuple[bool, str]:
        if self._session is None:
            return False, "no active ROS Bag session"
        try:
            manifest = self._session.stop(reason)
            metadata = {
                "drive_metrics": self._metrics.as_dict(),
                "bag_summary": summarize_sqlite_bag(self._session.paths.bag_dir)
                if self._session.storage_id == "sqlite3"
                else {"storage_summary": "mcap summary is not available in v1", "topic_metrics": []},
            }
            self._session.update_final_metadata(**metadata)
        except SessionError as exc:
            return False, str(exc)
        self._finalized = True
        self.publish_status(manifest["status"])
        return True, f"saved: {self._session.paths.session_dir}"

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._start_session()
        return response

    def _on_stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._stop_session("operator-stop")
        return response

    def _on_tick(self) -> None:
        if self._session is None or self._finalized:
            return
        if not self._session.is_running():
            self._stop_session("recorder-exited")
            return
        limit = self._session.enforce_limits(
            max_duration_seconds=float(self.get_parameter("max_duration_seconds").value),
            max_size_bytes=int(float(self.get_parameter("max_size_gb").value) * 1024**3),
        )
        if limit is not None:
            self._stop_session(limit)

    def publish_status(self, state: str) -> None:
        self._status_publisher.publish(String(data=state))

    def destroy_node(self) -> bool:
        if self._session is not None and not self._finalized:
            self._stop_session("node-shutdown")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BagSessionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
