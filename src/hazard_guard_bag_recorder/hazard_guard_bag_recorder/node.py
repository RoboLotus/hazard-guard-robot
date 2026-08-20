"""ROS 2 entry point for profile-driven rosbag2 recording."""

from __future__ import annotations

from pathlib import Path
import json
import os
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from hazard_guard_interfaces.srv import BagRecorderControl

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
        self.declare_parameter("max_duration_seconds", 1800.0)
        self.declare_parameter("max_size_gb", 10.0)
        self.declare_parameter("allow_experimental", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("enable_control_services", False)

        self._session: BagSession | None = None
        self._finalized = True
        self._active_profile: str | None = None
        self._last_manifest: dict | None = None
        self._storage_root = Path(
            str(self.get_parameter("storage_root").value)
        ).expanduser().resolve()
        self._metrics = DriveMetrics()
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_subscription(String, "/hazard_guard/mission/status", self._on_mission_status, 10)
        if bool(self.get_parameter("enable_control_services").value):
            self.create_service(Trigger, "/hazard_guard/bag/start", self._on_start)
            self.create_service(Trigger, "/hazard_guard/bag/stop", self._on_stop)
            self.create_service(
                BagRecorderControl,
                "/hazard_guard/bag/control",
                self._on_control,
            )
        else:
            self.get_logger().info("manual ROS Bag control services are disabled")
        self._status_publisher = self.create_publisher(String, "/hazard_guard/bag/status", 10)
        self._status_json_publisher = self.create_publisher(
            String, "/hazard_guard/bag/status_json", 10
        )
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

    def _start_session(
        self,
        *,
        profile_name: str | None = None,
        session_name: str | None = None,
        allow_experimental: bool | None = None,
    ) -> tuple[bool, str]:
        if self._session is not None and self._session.is_running():
            return False, "ROS Bag session is already recording"
        try:
            profile_name = profile_name or str(self.get_parameter("profile").value)
            session_name = session_name or str(self.get_parameter("session_name").value)
            effective_experimental = (
                bool(self.get_parameter("allow_experimental").value)
                and (True if allow_experimental is None else bool(allow_experimental))
            )
            max_duration_seconds = float(self.get_parameter("max_duration_seconds").value)
            max_size_bytes = int(float(self.get_parameter("max_size_gb").value) * 1024**3)
            profile = resolve_profile(
                profile_name,
                load_profile_document(Path(str(self.get_parameter("profiles_path").value))),
            )
            minimum_free_bytes = int(float(self.get_parameter("minimum_free_gb").value) * 1024**3)
            preflight = run_preflight(
                profile,
                self._available_topic_names(),
                self._storage_root,
                minimum_free_bytes=minimum_free_bytes,
                allow_experimental=effective_experimental,
            )
            if not preflight.can_start:
                return False, f"required topics missing: {', '.join(preflight.missing_required)}"
            self._metrics = DriveMetrics()
            self._session = BagSession(
                create_session_paths(
                    self._storage_root,
                    session_name,
                ),
                profile.name,
                preflight,
                storage_id=str(self.get_parameter("storage_id").value),
                minimum_free_bytes=minimum_free_bytes,
                max_duration_seconds=max_duration_seconds,
                max_size_bytes=max_size_bytes,
            )
            self._session.start()
        except (ProfileError, PreflightError, SessionError, ValueError) as exc:
            self.get_logger().error(str(exc))
            return False, str(exc)
        self._finalized = False
        self._active_profile = profile.name
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
            self._last_manifest = self._session.update_final_metadata(**metadata)
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

    def _on_control(
        self, request: BagRecorderControl.Request, response: BagRecorderControl.Response
    ) -> BagRecorderControl.Response:
        command = request.command.strip().lower()
        if command == "start":
            response.accepted, response.message = self._start_session(
                profile_name=request.profile.strip() or None,
                session_name=request.session_name.strip() or None,
                allow_experimental=bool(request.allow_experimental),
            )
        elif command == "stop":
            response.accepted, response.message = self._stop_session("operator-stop")
        elif command == "status":
            response.accepted, response.message = True, "status"
        elif command == "list":
            response.accepted, response.message = True, "sessions"
        else:
            response.accepted, response.message = False, "unsupported bag recorder command"
        response.status_json = json.dumps(
            self.status_payload(include_sessions=command == "list"), ensure_ascii=False
        )
        return response

    def _on_tick(self) -> None:
        if self._session is None or self._finalized:
            return
        if not self._session.is_running():
            self._stop_session("recorder-exited")
            return
        limit = self._session.enforce_limits()
        if limit is not None:
            self._stop_session(limit)

    def publish_status(self, state: str) -> None:
        # Keep the original compact status topic stable for CLI users.
        self._status_publisher.publish(String(data=state))
        self._status_json_publisher.publish(
            String(data=json.dumps(self.status_payload(state), ensure_ascii=False))
        )

    def status_payload(self, state: str | None = None, *, include_sessions: bool = False) -> dict:
        active = self._session is not None and self._session.is_running() and not self._finalized
        current_state = state or (
            "recording" if active else str((self._last_manifest or {}).get("status", "idle"))
        )
        payload = {
            "state": current_state,
            "recording": active,
            "profile": self._active_profile or (self._last_manifest or {}).get("profile"),
            "control_enabled": bool(self.get_parameter("enable_control_services").value),
            "updated_at_unix": round(time.time(), 3),
        }
        if self._session is not None:
            payload["session_id"] = self._session.paths.session_dir.name
            payload["storage_id"] = self._session.storage_id
            payload["elapsed_seconds"] = (
                round(time.monotonic() - self._session._started_monotonic, 1)
                if active and self._session._started_monotonic is not None
                else float((self._last_manifest or {}).get("duration_seconds", 0.0))
            )
        if include_sessions:
            payload["sessions"] = self._recent_sessions()
        return payload

    def _recent_sessions(self) -> list[dict]:
        root = self._storage_root
        sessions = []
        candidates = []
        try:
            with os.scandir(root) as entries:
                for entry in entries:
                    if len(candidates) >= 200:
                        break
                    if entry.is_symlink() or not entry.is_dir(follow_symlinks=False):
                        continue
                    manifest_path = Path(entry.path) / "session.json"
                    try:
                        details = manifest_path.lstat()
                        resolved = manifest_path.resolve(strict=True)
                        if manifest_path.is_symlink() or root not in resolved.parents:
                            continue
                        if details.st_size > 1_000_000:
                            continue
                        candidates.append((details.st_mtime, manifest_path, entry.name))
                    except OSError:
                        continue
        except OSError:
            return sessions
        for _modified, manifest_path, directory_name in sorted(candidates, reverse=True)[:50]:
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                if not isinstance(manifest, dict):
                    continue
                summary = manifest.get("bag_summary", {})
                if not isinstance(summary, dict):
                    summary = {}
                topic_metrics = summary.get("topic_metrics", [])
                if not isinstance(topic_metrics, list):
                    topic_metrics = []
                normalized_metrics = []
                for item in topic_metrics[:50]:
                    if not isinstance(item, dict):
                        continue
                    normalized_metrics.append({
                        "name": str(item.get("name", ""))[:160],
                        "type": str(item.get("type", ""))[:160],
                        "messages": int(item.get("messages", 0) or 0),
                        "duration_seconds": float(item.get("duration_seconds", 0) or 0),
                        "average_rate_hz": item.get("average_rate_hz"),
                    })
                sessions.append({
                    "session_id": directory_name,
                    "profile": str(manifest.get("profile", "unknown")),
                    "status": str(manifest.get("status", "unknown")),
                    "started_at": manifest.get("started_at"),
                    "duration_seconds": manifest.get("duration_seconds", 0.0),
                    "bag_size_bytes": manifest.get("bag_size_bytes", 0),
                    "end_reason": manifest.get("end_reason"),
                    "bag_summary": {
                        "storage_summary": str(summary.get("storage_summary", ""))[:160],
                        "topic_metrics": normalized_metrics,
                    },
                })
            except (OSError, ValueError, json.JSONDecodeError):
                continue
        return sessions

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
