import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class PhysicalProfileContractTests(unittest.TestCase):
    def test_profile_keeps_physical_actuation_disabled(self):
        profile = (PACKAGE_ROOT / "config" / "dispenser_physical.yaml").read_text(
            encoding="utf-8"
        )
        # 서보 혼을 보드의 전원 인가 각도(90)가 챔버 닫힘이 되도록 맞춰 끼웠다.
        # 이 값들이 그 기구 상태와 짝이다. 혼을 다시 끼우지 않는 한 바뀌면 안 된다.
        self.assertIn("angle_home: 90", profile)
        self.assertIn("angle_dump: 150", profile)
        self.assertIn("servo_min_angle: 90", profile)
        self.assertIn("servo_max_angle: 165", profile)
        self.assertIn("enable_physical_drop: false", profile)
        self.assertIn("require_person_safety_clear: false", profile)
        self.assertIn("battery_report_sec: 10.0", profile)
        self.assertIn("battery_stale_sec: 180.0", profile)
        # 5초로는 큐브 광고를 놓친다 (실측).
        self.assertIn("ble_scan_seconds: 15.0", profile)
        # 0 = 자동 부분 탐색 없음. 웹UI 새로고침 버튼으로 그때그때 찾는다.
        self.assertIn("ble_partial_rescan_sec: 0.0", profile)
        self.assertIn("ble_manual_scan_seconds: 30.0", profile)
        self.assertIn("ble_battery_refresh_sec: 30.0", profile)
        self.assertIn("ble_connection_stable_sec: 3.0", profile)

    def test_launch_loads_packaged_physical_profile(self):
        launch = (PACKAGE_ROOT / "launch" / "dispenser.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"dispenser_physical.yaml"', launch)
        self.assertIn('default_value="false"', launch)


if __name__ == "__main__":
    unittest.main()
