# HazardGuard Robot

산업 현장을 순찰하며 화재·과열 징후를 탐지하는 ROSMASTER-M1 기반
HazardGuard 로봇의 ROS 2 워크스페이스입니다. 현재 저장소는 실제 하드웨어 없이
Gazebo Fortress, SLAM Toolbox, Nav2와 WebUI 연동을 검증할 수 있는 개발 기반을
제공합니다.

## 현재 범위

- ROS 2 Humble
- Gazebo Fortress (`ros_gz`)
- ROSMASTER-M1 Superior Kit 기반 로봇 모델
- RGB, Depth, ThermoEye TMC160B 사양 기반 합성 열화상, 2D LiDAR, IMU 센서 시뮬레이션
- SLAM Toolbox 기반 지도 작성
- RTAB-Map 기반 RGB-D 컬러 3D 지도 실험
- AMCL 기반 위치 추정
- Nav2 단일 목적지·다중 웨이포인트 주행
- 하드웨어 없이 사용하는 mock telemetry·열원 탐지
- ROS 2 Action 기반 다중 웨이포인트 임무 관리자
- FastAPI WebUI bridge에서 사용할 ROS 토픽과 액션

Jetson 전용 CUDA·TensorRT, 실제 ROSMASTER 하드웨어 드라이버, 경고장치 제어는
아직 포함하지 않습니다.

## 패키지 구성

```text
src/
├─ hazard_guard_interfaces/    메시·서비스·순찰 Action 정의
├─ hazard_guard_mission_manager/ Nav2 순찰 임무 실행 노드
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

## 월드

기본 월드는 `worlds/demo_facility.sdf`입니다. 강의실에서 실기 시연이 가능하도록
재활용 처리장을 **8 m × 6 m** 방에 담은 데모 공간입니다.

**배치는 `RoboLotus/gazebo-simulator` 원본을 따릅니다.** 같은 모델 8종을 같은
include 순서로, 아래 명시한 한 건을 제외하고 전부 공통 월드 원점에 회전 없이
배치합니다. 균등 축척은 `0.13201`이며, 이는 60.60 m × 35.60 m 시설을
8 m × 6 m 방에 넣을 수 있는 최대값입니다. 통로 폭이 축척에 비례하므로 클수록
유리합니다.

원본 블록아웃에는 설비끼리 바닥에서 겹치는 곳이 여러 군데 있습니다.

| 겹치는 쌍 | 면적 |
|---|---|
| secondary_processor ↔ baler | 0.394 m² |
| baler ↔ bale_storage | 0.128 m² |
| bunker ↔ primary_shredder | 0.072 m² |
| sorting_line ↔ secondary_processor | 0.068 m² |
| primary_shredder ↔ sorting_line | 0.038 m² |

원본에서 벗어난 곳은 두 군데입니다.

- **`control_room` 제외** — 로봇이 주행할 수 있는 유일한 남측 통로의 남동쪽
  모서리를 차지하고 있었고, 사람이 상주하는 관리실은 순찰 대상도 아닙니다.
- **`bale_storage` 이동** — 관리실이 비운 자리로 옮깁니다(90° 회전, 중심
  2.80, −1.90). 압축기와의 겹침도 함께 해소됩니다.

나머지 겹침은 원본 그대로 둡니다. 그것까지 손대려면 설비 한 개를 옮기는
수준이 아니라 시설을 재설계해야 합니다. 변경 지점은
`tools/gen_demo_world.py`의 `OVERRIDES`에 모아두었습니다. 생성기는 실행할
때마다 남은 겹침을 출력하므로, 이동이 되돌려지거나 새 겹침이 생기면 바로
드러납니다.

Fortress 변환은 원본이 제공할 수 없는 세 가지로 한정합니다.

- 시스템 플러그인 이름 `gz-sim-*` → `ignition-gazebo-*`
- 로봇 IMU 를 위한 `ignition-gazebo-imu-system`
- 이전 월드들이 쓰던 ODE physics 튜닝

### 주행 범위

원본 처리장은 벙커부터 베일 창고까지 설비가 벽에서 벽까지 이어져 있어
**어느 축척에서도 순환로가 생기지 않습니다.** 순찰 경로는 남측 통로이며 왕복
주행입니다. 원본 배치 유지와 순환로 확보는 양립하지 않습니다.

| 항목 | 값 |
|---|---|
| 주행 가능 면적 | 4.81 m² (방의 10%) |
| 주행 범위 | x −1.81 … 3.29 m, y −1.79 … 0.55 m |
| 최대 통로 폭 | 2.64 m |
| M1 최소 요구 | 1.01 m (풋프린트 0.31 m + 인플레이션 0.35 m) |

레이아웃은 `tools/gen_demo_world.py`가 생성하고 동시에 검증합니다. 방 크기가
다르면 그 스크립트의 `HALL_X`/`HALL_Y`만 바꾸고 다시 실행하면 축척과
웨이포인트가 함께 재계산됩니다. 월드 SDF는 생성물이므로 직접 편집하지
마십시오.

### 데모 설비 치수 (실물 제작용)

| 설비 | 가로 | 세로 | 높이 | 중심 X | 중심 Y |
|---|---|---|---|---|---|
| hall_shell | 8.00 m | 4.70 m | 1.62 m | 0.00 | 0.00 |
| bunker | 1.56 m | 3.53 m | 0.95 m | −3.11 | −0.13 |
| primary_shredder | 1.47 m | 1.84 m | 0.90 m | −1.76 | 0.35 |
| sorting_line | 2.50 m | 1.72 m | 0.66 m | 0.07 | 1.17 |
| secondary_processor | 1.39 m | 1.88 m | 0.93 m | 1.76 | 0.55 |
| baler | 1.23 m | 2.44 m | 0.71 m | 2.63 | 0.77 |
| bale_storage \* | 2.27 m | 0.71 m | 0.61 m | 2.80 | −1.90 |

`*` 표시는 원본 배치에서 옮긴 설비입니다. `control_room`은 제외되어 표에
없습니다.

원점은 방 중앙이고 X가 8 m 변, Y가 6 m 변입니다. 실물은 라이다가 보는 높이만
막으면 되므로 표의 가로·세로만 맞추면 됩니다. `hall_shell`은 방의 실제 벽으로
대체해도 됩니다.

### 순찰 웨이포인트

남측 통로를 서에서 동으로 훑고 되돌아오는 4점입니다. 기본 스폰 위치는 통로가
가장 넓은 P2 입니다.

| 지점 | X | Y | 통로 여유 |
|---|---|---|---|
| P1 | −1.29 | −1.07 | 1.01 m |
| P2 | 0.13 | −0.99 | 2.64 m |
| P3 | 1.51 | −0.99 | 1.16 m |
| P4 | 2.93 | −0.99 | 1.12 m |

### 다른 월드

원본 60 m × 35 m 처리장(`recycling_facility.sdf`)과 초기 3.6 m × 2.4 m 테스트
아레나(`facility_map.sdf`)도 남아 있습니다. `world`와 `world_name`은 항상 같이
넘겨야 하며, `config/nav2.yaml`과 `config/slam.yaml`은 데모 공간 기준으로
맞춰져 있으므로 큰 월드에서는 costmap 범위와 라이다 사거리를 함께 올려야
합니다.

```bash
ros2 launch hazard_guard_simulation simulation.launch.py \
  world:="$(ros2 pkg prefix hazard_guard_simulation)/share/hazard_guard_simulation/worlds/recycling_facility.sdf" \
  world_name:=recycling_facility \
  spawn_x:=0.0 spawn_y:=-10.0
```

WebUI를 함께 쓸 때는 백엔드의 `HAZARD_GUARD_SIMULATION_WORLD_MARKER`를 선택한
월드 파일 이름으로 맞춰야 WebUI가 시뮬레이터 프로세스를 인식하고 종료할 수
있습니다. 데모 월드를 쓸 때는 `demo_facility.sdf`입니다.

## 자동 순찰

수동 주행 없이 웨이포인트를 자동으로 도는 방법입니다. Nav2 와
`hazard_guard_mission_manager` 가 함께 떠 있어야 하므로 `navigation.launch.py`
또는 `localization.launch.py` 를 사용합니다. `simulation.launch.py` 나
`slam.launch.py` 만으로는 Nav2 가 없어 액션 서버가 뜨지 않습니다.

터미널 1 - SLAM + Nav2 + 임무 관리자:

```bash
ros2 launch hazard_guard_simulation navigation.launch.py gui:=false
```

터미널 2 - 순찰 시작 (Nav2 가 활성화될 때까지 40초 정도 기다린 뒤):

```bash
tools/run_patrol.sh
```

`P1 → P2 → P3 → P4` 를 돌고 출발점으로 복귀합니다. 각 지점에서 2초 정차하며,
정차 시간은 `PATROL_DWELL_SECONDS` 로 조정합니다. 진행 상황은 액션 피드백과
`/hazard_guard/mission/status` 로 확인할 수 있습니다.

지도를 만들면서 순찰해도 됩니다. `config/nav2.yaml` 의 플래너가
`allow_unknown: true` 라 아직 관측하지 않은 공간으로도 경로를 세우고, SLAM 이
주행하는 동안 지도를 채웁니다.

중단하려면 액션을 취소하거나 다음 서비스를 호출합니다.

```bash
ros2 service call /hazard_guard/mission/cancel std_srvs/srv/Trigger
```

임무 관리자를 거치지 않고 Nav2 에 직접 넣으려면 `/follow_waypoints` 를 쓸 수
있지만, 구간 경로 사전 검증과 목표 방향 정렬은 임무 관리자에만 있습니다.

WebUI 를 쓰는 경우 `지도` 탭의 웨이포인트 패널이 같은 액션을 호출하므로 이
스크립트를 따로 실행할 필요가 없습니다.

## 지도 작성

```bash
ros2 launch hazard_guard_simulation slam.launch.py gui:=true
```

로봇을 수동 주행시켜 공간을 탐색한 뒤 지도를 저장합니다.

지도 작성 프로필은 두 가지입니다.

| 프로필 | launch 인자 | 생성 결과 | 권장 용도 |
|---|---|---|---|
| 2D 표준 | `enable_rtabmap:=false` | SLAM Toolbox 점유 지도 | Nav2 순찰용 지도 작성, 빠른 반복 검증 |
| 2D + RGB-D 3D | `enable_rtabmap:=true` | 동일한 2D 점유 지도 + RTAB-Map DB·컬러 포인트클라우드 | 3D 공간 및 향후 열화상 융합 실험 |

```bash
ros2 launch hazard_guard_simulation slam.launch.py \
  gui:=false \
  enable_rtabmap:=true \
  rtabmap_database_path:="$(pwd)/runtime/maps/rtabmap.db"
```

두 프로필 모두 `/map`과 `map → odom` TF는 SLAM Toolbox만 발행합니다.
RTAB-Map은 별도 `rtabmap_map` 좌표계와 `/rtabmap/grid_map`을 사용하므로
2D 지도와 TF를 중복 발행하지 않습니다. 따라서 3D 수집을 켜도 Nav2가 사용할
2D 지도 생성 방식은 바뀌지 않습니다.

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
같은 launch에서 `hazard_guard_mission_manager`가 시작되어 WebUI가 전달한
다중 웨이포인트를 사전 검증한 뒤 Nav2에 순차 전달합니다.

SLAM을 실행한 상태에서 Nav2를 함께 시험하려면 다음을 사용합니다.

```bash
ros2 launch hazard_guard_simulation navigation.launch.py gui:=true
```

## RTAB-Map RGB-D 3D 지도 실험

이 기능은 기존의 `SLAM Toolbox + Nav2` 2D 운용 경로를 교체하지 않는 별도
시뮬레이션 실험입니다. 로봇은 바닥의 X/Y/Yaw로 이동하면서 RGB와 Depth를
결합해 높이(Z)와 색상이 포함된 포인트클라우드를 누적합니다.

RViz에서 직접 확인하면서 안전 구간 자동 수집 경로를 실행하려면 다음을
사용합니다.

```bash
ros2 launch hazard_guard_simulation rtabmap_sim.launch.py \
  gui:=false \
  rviz:=true \
  demo_route:=true
```

`demo_route:=true`는 `demo_facility` 남측 통로에서만 실행되는 제한된 검증
경로입니다. 가장 여유가 넓은 기본 스폰 지점 P2에서만 360° 관측하고, 통로의
동·서쪽 안전 구간은 차체 방향을 유지한 채 전진·후진으로 왕복합니다. 좁은
양 끝에서 제자리 회전하다 설비와 접촉하는 상황을 피하며, 실제 로봇에서는
실행하지 않습니다.

RTAB-Map 자체의 `/rtabmap/cloud_map`은 이 구성에서 Z=0인 장애물 점유 셀을
나타냅니다. 컬러 표면 지도는 RGB·Depth로 프레임별 포인트클라우드를 만든 뒤
RTAB-Map의 `map` 좌표계에 누적하여
`/hazard_guard/rtabmap/cloud_surface`로 발행합니다. 각 점은 X/Y/Z와 RGB를
포함하며 WebUI 백엔드가 이를 다운샘플링해 브라우저로 전송합니다. RViz는 이
데이터를 보는 도구일 뿐 WebUI의 데이터 원본은 아닙니다.

실제 로봇 전환 시 RTAB-Map 알고리즘 코드를 다시 만들 필요는 없지만,
Gazebo 카메라 토픽 대신 실제 RGB·Depth·CameraInfo·Odometry·TF를 연결해야
합니다. 특히 RGB 카메라와 Depth 카메라의 내부 파라미터 및 센서와
`base_link` 사이 외부 캘리브레이션이 선행되어야 합니다.

이 컬러 클라우드는 센서가 관측한 표면의 3D 복원 결과이므로 가려진 면과 아직
주행하지 않은 구역은 포함하지 않습니다. Gazebo SDF의 모든 벽과 메시를 완전한
형태로 표시하는 것은 SLAM 검증과 구분되는 시뮬레이터 원본 디지털 트윈
기능입니다.

## WebUI 운용 모드 연동

`hazard-guard-console` 백엔드의 모드 제어를 활성화하면 WebUI `지도` 탭에서
다음 launch 구성을 선택할 수 있습니다.

- `맵 생성 / SLAM`: `slam.launch.py`
  - `2D 표준`: SLAM Toolbox만 실행
  - `2D + RGB-D 3D`: SLAM Toolbox와 RTAB-Map을 함께 실행
- `순찰 / AMCL·Nav2`: 저장 지도를 사용하는 `localization.launch.py`

WebUI가 새 지도 세션을 시작하면 세션 디렉터리를 만들고 2D 지도는
`map.yaml`·`map.pgm`, 3D 수집 프로필은 추가로 `rtabmap.db`에 저장합니다.
RTAB-Map DB는 장시간 주행할수록 커질 수 있으므로 세션 목록에서 용량을
확인하고 필요한 결과만 보관합니다.

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
| RTAB-Map 컬러 3D 지도 | `/hazard_guard/rtabmap/cloud_surface` |
| 열화상 | `/thermal_camera/image_raw` |
| IMU | `/imu/data_raw` |
| 로봇 상태 | `/hazard_guard/telemetry` |
| 열원 탐지 | `/hazard_guard/thermal_detections` |
| 단일 목적지 | `/navigate_to_pose` |
| HazardGuard 순찰 임무 | `/hazard_guard/run_patrol` |
| 순찰 상태 | `/hazard_guard/mission/status` |
| 순찰 강제 취소 서비스 | `/hazard_guard/mission/cancel` |

열화상 시뮬레이션은 ThermoEye TMC160B의 160×120 해상도, 수평 57° FOV,
8.7 Hz를 반영합니다. 지도에 보이는 5 m 부채꼴 길이는 화면 표현을 위한
시뮬레이션 경계이며 제조사가 보장하는 측정거리가 아닙니다. 현재 열화상과
열원 값은 합성 데이터이므로 실제 화재 판정 성능을 의미하지 않습니다.

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
