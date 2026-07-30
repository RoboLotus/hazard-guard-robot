from __future__ import annotations

from dataclasses import dataclass
from math import sin


@dataclass
class RobotState:
    robot_id: str = "rosmaster-m1-mock"
    mode: str = "patrol"
    controller_enabled: bool = False
    battery_percent: float = 78.0
    network_rssi_dbm: int = -48
    lidar_hz: float = 10.2
    lidar_status: str = "normal"
    measured_speed_mps: float | None = None

    def apply_command(self, command: str, enabled: bool = False) -> tuple[bool, str]:
        normalized = command.strip().lower()

        if normalized == "pause":
            if self.mode == "stopped":
                return False, "정지 상태에서는 순찰을 일시정지할 수 없습니다."
            self.mode = "paused"
            return True, "순찰을 일시정지했습니다."

        if normalized == "resume":
            if self.mode == "stopped":
                return False, "정지 상태는 안전 확인 없이 해제할 수 없습니다."
            self.mode = "patrol"
            return True, "순찰을 재개했습니다."

        if normalized == "stop":
            self.mode = "stopped"
            self.controller_enabled = False
            return True, "mock robot을 정지 상태로 전환했습니다."

        if normalized in {"controller", "controller_on", "controller_off"}:
            next_enabled = enabled
            if normalized == "controller_on":
                next_enabled = True
            elif normalized == "controller_off":
                next_enabled = False
            self.controller_enabled = next_enabled
            return True, f"컨트롤러 입력을 {'활성화' if next_enabled else '비활성화'}했습니다."

        return False, f"지원하지 않는 mock 명령입니다: {command}"

    def snapshot(self, elapsed_seconds: float) -> dict[str, object]:
        speed = (
            self.measured_speed_mps
            if self.measured_speed_mps is not None
            else (0.32 if self.mode == "patrol" else 0.0)
        )
        temperature = 63.0 + 1.8 * sin(elapsed_seconds / 5.0)
        battery = max(0.0, self.battery_percent - elapsed_seconds / 7200.0)
        return {
            "robot_id": self.robot_id,
            "mode": self.mode,
            "battery_percent": round(battery, 2),
            "speed_mps": speed,
            "network_quality": "good" if self.network_rssi_dbm >= -65 else "poor",
            "network_rssi_dbm": self.network_rssi_dbm,
            "lidar_status": self.lidar_status,
            "lidar_hz": self.lidar_hz,
            "max_temperature_c": round(temperature, 2),
            "alert_level": "warning" if temperature >= 60.0 else "normal",
            "controller_enabled": self.controller_enabled,
            "mock": True,
        }
