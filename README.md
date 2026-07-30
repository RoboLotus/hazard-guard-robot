# HazardGuard Robot

산업 현장을 순찰하며 화재·과열 징후를 탐지하는 ROSMASTER-M1 기반
HazardGuard 로봇의 ROS 2 워크스페이스입니다. 현재 저장소는 실제 하드웨어 없이
Gazebo Fortress, SLAM Toolbox, Nav2와 WebUI 연동을 검증할 수 있는 개발 기반을
제공합니다.

## 현재 범위

- ROS 2 Humble
- Gazebo Fortress (`ros_gz`)
- ROSMASTER-M1 Superior Kit 기반 로봇 모델
- RGB, Depth, 열화상, 2D LiDAR, IMU 센서 시뮬레이션
- SLAM Toolbox 기반 지도 작성
- AMCL 기반 위치 추정
- Nav2 단일 목적지·다중 웨이포인트 주행
- 하드웨어 없이 사용하는 mock telemetry·열원 탐지
- FastAPI WebUI bridge에서 사용할 ROS 토픽과 액션

Jetson 전용 CUDA·TensorRT, 실제 ROSMASTER 하드웨어 드라이버, 경고장치 제어는
아직 포함하지 않습니다.

## 패키지 구성

```text
src/
├─ hazard_guard_interfaces/    메시와 서비스 정의
├─ hazard_guard_mock_robot/    mock 상태·명령·열원·검증 노드
├─ hazard_guard_bringup/       기본 mock bringup
└─ hazard_guard_simulation/    Fortress 모델, 월드, SLAM, Nav2
```

## 권장 개발 환경

- Ubuntu 22.04
- ROS 2 Humble
- x86_64: WSL2 개발·시뮬레이션
- aarch64: 추후 Jetson Orin Nano Super 배포

팀 공용 Docker 기반 환경은 별도 `RoboLotus/slam-jetson-env` 저장소에서
관리합니다. 이 저장소는 ROS 패키지와 시뮬레이션 소스만 관리합니다.

## 빌드

ROS 2와 팀 워크스페이스가 준비된 셸에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
test -f /opt/rosmaster_ws/install/setup.bash \
  && source /opt/rosmaster_ws/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

## 시뮬레이션 실행

GUI 포함:

```bash
ros2 launch hazard_guard_simulation simulation.launch.py \
  gui:=true \
  simulation_mode:=kinematic
```

GUI 없이:

```bash
ros2 launch hazard_guard_simulation simulation.launch.py \
  gui:=false \
  simulation_mode:=kinematic
```

`kinematic` 모드는 SLAM·Nav2·WebUI 통합 개발에 사용하는 안정화 모드입니다.
`physics` 모드는 메카넘 휠 접촉과 마찰 계수를 시험하는 실험 모드이며, 실제
주행 정확도를 보장하지 않습니다.

## 지도 작성

```bash
ros2 launch hazard_guard_simulation slam.launch.py gui:=true
```

로봇을 수동 주행시켜 공간을 탐색한 뒤 지도를 저장합니다.

```bash
mkdir -p runtime/maps
ros2 run nav2_map_server map_saver_cli \
  -t map \
  -f "$(pwd)/runtime/maps/facility" \
  --ros-args -p save_map_timeout:=10.0
```

`runtime/maps`의 생성 지도는 Git에 포함하지 않습니다.

## 저장된 지도로 Localization·Nav2 실행

```bash
ros2 launch hazard_guard_simulation localization.launch.py \
  gui:=true \
  simulation_mode:=kinematic \
  map:="$(pwd)/runtime/maps/facility.yaml"
```

launch 파일은 Gazebo 시작 위치를 AMCL 초기 위치로 자동 전달합니다.

SLAM을 실행한 상태에서 Nav2를 함께 시험하려면 다음을 사용합니다.

```bash
ros2 launch hazard_guard_simulation navigation.launch.py gui:=true
```

## WebUI 운용 모드 연동

`hazard-guard-console` 백엔드의 모드 제어를 활성화하면 WebUI `지도` 탭에서
다음 launch 구성을 선택할 수 있습니다.

- `맵 생성 / SLAM`: `slam.launch.py`
- `순찰 / AMCL·Nav2`: 저장 지도를 사용하는 `localization.launch.py`

WebUI에서 모드를 관리하는 동안에는 같은 launch를 별도 터미널에서 동시에
실행하지 않습니다. 맵 생성 모드에서는 목적지 이동과 웨이포인트 순찰 명령이
차단되며, 순찰 모드가 준비된 후 사용자가 별도로 순찰 시작 명령을 내려야
로봇이 이동합니다.

## 주요 ROS 인터페이스

| 구분 | 이름 |
|---|---|
| SLAM 지도 | `/map` |
| 로봇 위치·오도메트리 | `/tf`, `/odom` |
| 주행 명령 | `/cmd_vel` |
| LiDAR | `/scan` |
| RGB | `/camera/image_raw` |
| Depth | `/depth_camera/image_raw` |
| 열화상 | `/thermal_camera/image_raw` |
| IMU | `/imu/data_raw` |
| 로봇 상태 | `/hazard_guard/telemetry` |
| 열원 탐지 | `/hazard_guard/thermal_detections` |
| 단일 목적지 | `/navigate_to_pose` |
| 웨이포인트 | `/follow_waypoints` |

현재 열화상과 열원 값은 시뮬레이션 데이터이며 실제 화재 판정 성능을 의미하지
않습니다.

## 검증

```bash
colcon test
colcon test-result --verbose
```

가까운 자유 공간으로 Nav2 명령을 보내는 스모크 테스트:

```bash
ros2 run hazard_guard_mock_robot nav2_smoke_test \
  --ros-args \
  -p use_sim_time:=true \
  -p target_x:=0.45 \
  -p target_y:=0.68
```

## 저장소 정책

- `build/`, `install/`, `log/`, 생성 지도와 rosbag은 커밋하지 않습니다.
- 제조사 원본 CAD와 Sketchfab 원본 파일은 커밋하지 않습니다.
- AGENTS, Codex 상태, 로컬 하네스와 오케스트레이션 문서는 커밋하지 않습니다.
- 비밀키·토큰·개인 절대 경로를 저장하지 않습니다.
- 실제 하드웨어 명령은 안전 계층이 완성될 때까지 mock으로 취급합니다.

외부 메시의 출처와 재배포 주의사항은
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)를 확인하십시오.
