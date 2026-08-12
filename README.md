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

기본 월드는 `worlds/demo_facility_scaled.sdf`입니다. 강의실에서 실기 시연이
가능하도록 재활용 처리장을 **6 m × 4 m** 방(홀 내부 실측 5.90 m × 3.42 m)에
담은 데모 공간입니다. 설비를 홀 한가운데 정렬해 둘레를 통로로 비웠습니다.

| 대상 | 축척 | 결과 |
|---|---|---|
| `hall_shell` | `0.0990099` | 60.60 m × 35.60 m 시설 외곽을 방에 넣는 최대값 |
| 공정 라인 5종 | `0.07474982` | 한 덩어리로 축소, 컨베이어 연결 관계 보존 |
| `bale_storage` | `0.09919193` | 180° 회전, 압축기 뒤 동쪽 끝 |

설비 섬은 4.70 m × 2.23 m 이고 홀 중앙에 놓이므로 사방에 통로가 남습니다.
**서 0.610 / 동 0.590 / 남 0.609 / 북 0.585 m** 입니다. 로봇 풋프린트가
0.600 m × 0.310 m 이므로 통로와 나란히 서면 양쪽에 0.14 m 씩 남고, 외곽을
한 바퀴 도는 **닫힌 순환로**가 성립합니다. 자세한 제약은
[순찰 웨이포인트](#순찰-웨이포인트)를 보십시오.

`worlds/demo_facility.sdf` 는 같은 문제를 다르게 푼 대안입니다. 축척을 홀과
설비 둘로만 나눠(`0.09901` / `0.04951`) 통로를 훨씬 넓게(동·서 1.52 m,
남·북 0.97 m) 확보하는 대신 설비를 더 작게 만듭니다. 아래 절은 그쪽 월드의
설계 근거이며, 기본 월드에는 그대로 적용되지 않습니다.

**배치는 `RoboLotus/gazebo-simulator` 원본 그대로입니다.** 모델 7종을 같은
include 순서로, 서로에 대한 위치와 회전을 하나도 바꾸지 않고 넣습니다. 옮기거나
돌린 설비는 없고, `control_room` 만 제외합니다 — 시설의 남쪽 최외곽(원본 y −16.8)
이라 섬 깊이를 3.19 m 로 늘리는데, 깊이가 축소율의 병목이라 이것만 빼도 나머지
설비를 8% 키울 수 있습니다. 사람이 상주하는 관리실은 순찰 대상도 아닙니다.

### 축척을 둘로 나눈 이유

| 대상 | 축척 | 결과 |
|---|---|---|
| `hall_shell` | `0.09901` | 60.60 m × 35.60 m 시설 외곽을 6 m × 4 m 방에 넣는 최대값 |
| 나머지 설비 전부 | `0.04951` (홀의 **0.50배**) | 설비 섬을 통째로 축소해 홀 중앙에 배치 |

설비를 한 덩어리(rigid group)로 축소·평행이동하므로 **시설 내부의 상대 거리는
완전히 보존**되고, 바뀌는 것은 시설 대 로봇의 비율뿐입니다.

이 축소가 순찰 경로를 만들어냅니다. 축소하지 않으면 설비가 벽에서 벽까지 꽉 차
로봇이 남측 막다른 통로 하나만 쓸 수 있습니다. 안쪽으로 당기면 둘레가 전부
통로가 되어 **닫힌 순환로**가 생깁니다.

축소율별 실측값입니다(홀 내부 5.900 m × 3.420 m, 로봇 회전 필요 폭 0.877 m).

| 축소율 | 설비 섬 크기 | 동·서 통로 | 남·북 통로 | 순환로 |
|---|---|---|---|---|
| 0.65 | 3.73 × 1.92 m | 1.09 m | 0.75 m | 없음 |
| 0.60 | 3.44 × 1.77 m | 1.23 m | 0.83 m | 없음 |
| 0.575 | 3.30 × 1.69 m | 1.30 m | 0.86 m | 있음 (기하학적 한계) |
| 0.55 | 3.16 × 1.62 m | 1.37 m | 0.90 m | 있음 (**실주행 실패**) |
| **0.50** | **2.87 × 1.47 m** | **1.52 m** | **0.97 m** | **있음 (8/8 완주)** |
| 0.45 | 2.58 × 1.33 m | 1.66 m | 1.05 m | 있음 (8/8 완주) |

홀이 시설보다 비율상 얕아서 항상 **남·북이 병목**이고, 동·서는 필요 이상으로
남습니다.

**순환로가 닫히는 것과 로봇이 실제로 도는 것은 별개였습니다.** 8점 순찰을 실제로
돌려 확인한 결과입니다.

| 축소율 | 남·북 통로 | Gazebo + Nav2 8점 실주행 |
|---|---|---|
| 0.55 | 0.90 m | P6 에서 갇힘 (2.82 m 이탈), `Failed to make progress` 13회 |
| **0.50** | **0.97 m** | **8/8 완주**, 최대 오차 0.08 m, 재시도 3회 |
| 0.45 | 1.05 m | 8/8 완주, 재시도 0회 |

기하학적 한계 0.575 와 실사용 한계 0.50 의 차이는 **레이아웃이 아니라 컨트롤러**
때문입니다. `config/nav2.yaml` 의 DWB 가 카메라와 주행 방향을 맞추려고
`max_vel_y` 를 0.08 m/s 로 묶어 두어, 메카넘인데도 좁은 통로에서 옆으로 빠져나갈
수 없었습니다. 이 값으로는 **0.45 조차 실패**했습니다(P4·P5 이탈).

`max_vel_y` 를 **0.25 m/s 로 올린 뒤** 0.45 가 재시도 0회로 완주했고, 0.50 도
완주합니다. 설비를 더 키우려면 `SHRINK` 가 아니라 이 컨트롤러 쪽을 더 손봐야
합니다. 대가는 로봇이 가끔 옆걸음(crab)으로 이동해 차체와 진행 방향이 어긋나
보이는 것입니다.

Fortress 변환은 원본이 제공할 수 없는 세 가지로 한정합니다.

- 시스템 플러그인 이름 `gz-sim-*` → `ignition-gazebo-*`
- 로봇 IMU 를 위한 `ignition-gazebo-imu-system`
- 이전 월드들이 쓰던 ODE physics 튜닝

### 주행 범위

| 항목 | 값 |
|---|---|
| 주행 가능 면적 | 6.39 m² (방의 27%) |
| 주행 범위 | x −2.53 … 2.53 m, y −1.29 … 1.29 m |
| 최대 통로 폭 | 1.76 m |
| 순환로 | **있음 — 설비 섬을 한 바퀴** |
| M1 회전 최소 요구 | 0.88 m (외접 반경 0.438 m × 2) |

| 변 | 통로 폭 (최소 / 최대) | 면적 |
|---|---|---|
| 남측 | 0.88 / 1.76 m | 3.42 m² |
| 동측 | 0.88 / 1.76 m | 3.32 m² |
| 북측 | 0.88 / 1.60 m | 2.51 m² |
| 서측 | 0.88 / 1.60 m | 2.73 m² |

최소값 0.88 m 는 네 모서리 안쪽에서만 나오고, 각 변의 직선 구간은 남·북
0.97 m, 동·서 1.52 m 입니다. 순환로가 닫혀 있으므로 ㄷ 자 경로는 물론 원하는
어떤 부분 경로도 만들 수 있습니다.

판정 기준은 로봇 **외접 반경 0.438 m** 입니다. 풋프린트
`[-0.410, ±0.155] ~ [0.190, ±0.155]` 의 외접 반경이며 통로로 환산하면 0.877 m
입니다. 각 웨이포인트에서 설비 쪽으로 제자리 회전해야 하므로 자세와 무관한 이
값을 씁니다. `half_width + inflation_radius` 를 쓰지 않는 이유는, 플래너를
실제로 막는 것은 **내접** 반경(0.155 m)이고 `inflation_radius` 는 비용만 만들기
때문입니다. `inflation_radius` 는 0.35 를 유지합니다.

레이아웃은 `tools/gen_demo_world.py`가 생성하고 동시에 검증합니다. 방 크기는
`HALL_X`/`HALL_Y`, 설비 축소율은 `SHRINK` 로 바꾸고 다시 실행하면 통로 폭과
웨이포인트가 함께 재계산됩니다. 월드 SDF는 생성물이므로 직접 편집하지 마십시오.

### 데모 설비 치수 (실물 제작용)

| 설비 | 가로 | 세로 | 높이 | 중심 X | 중심 Y |
|---|---|---|---|---|---|
| hall_shell | 6.00 m | 3.52 m | 1.22 m | 0.00 | 0.00 |
| bunker | 0.59 m | 1.32 m | 0.36 m | −1.15 | −0.09 |
| primary_shredder | 0.55 m | 0.69 m | 0.34 m | −0.65 | 0.10 |
| sorting_line | 0.94 m | 0.65 m | 0.25 m | 0.04 | 0.40 |
| secondary_processor | 0.52 m | 0.70 m | 0.35 m | 0.68 | 0.17 |
| baler | 0.46 m | 0.91 m | 0.27 m | 1.00 | 0.26 |
| bale_storage | 0.27 m | 0.85 m | 0.23 m | 1.29 | 0.20 |

원점은 방 중앙이고 X가 6 m 변, Y가 4 m 변입니다. 실물은 라이다가 보는 높이만
막으면 되므로 표의 가로·세로만 맞추면 됩니다. `hall_shell`은 방의 실제 벽으로
대체해도 됩니다.

원본 블록아웃이 원래 갖고 있던 바닥 겹침은 그대로 남습니다. 축소된 만큼 면적도
줄어듭니다.

| 겹치는 쌍 | 면적 |
|---|---|
| secondary_processor ↔ baler | 0.064 m² |
| baler ↔ bale_storage | 0.023 m² |
| bunker ↔ primary_shredder | 0.018 m² |
| sorting_line ↔ secondary_processor | 0.017 m² |

### 순찰 웨이포인트

기본 월드(`demo_facility_scaled.sdf`)의 외곽 통로를 반시계 방향으로 한 바퀴
도는 8점입니다. 좌표는 통로 중심선 위이고, 기본 스폰 위치는 남측 통로 중앙
(0.0975, −1.4121, yaw 0)입니다.

**yaw 는 설비 쪽이 아니라 통로와 나란한 방향입니다.** 통로가 0.585 ~ 0.610 m
인데 로봇 길이가 0.600 m 라서, 통로를 가로질러 서면 길이가 통로 폭을 그대로 다
먹습니다. 남·서(0.609 / 0.610 m)는 여유가 1 cm 미만이고 북·동(0.585 / 0.590 m)
은 아예 들어가지 않습니다. 즉 **링 위 어느 지점에서도 제자리에서 설비를 마주
볼 수 없습니다.** 통로와 나란히 서면 폭 0.310 m 만 쓰므로 양쪽에 0.14 m 씩
남습니다.

| 지점 | 이름 | X | Y | yaw | 통로 | 통로 폭 |
|---|---|---|---|---|---|---|
| P1 | 남동 | 2.645 | −1.417 | 0° | 남 | 0.609 m |
| P2 | 동측 | 2.645 | 0.000 | +90° | 동 | 0.590 m |
| P3 | 북동 | 2.645 | 1.405 | +90° | 동 | 0.590 m |
| P4 | 북측 | 0.000 | 1.405 | 180° | 북 | 0.585 m |
| P5 | 북서 | −2.655 | 1.405 | 180° | 서 | 0.610 m |
| P6 | 서측 | −2.655 | 0.000 | −90° | 서 | 0.610 m |
| P7 | 남서 | −2.655 | −1.417 | −90° | 남 | 0.609 m |
| P8 | 남측 | 0.098 | −1.417 | 0° | 남 | 0.609 m |

여덟 지점 모두 yaw 가 통로와 나란하므로 로봇이 통로를 가로지르는 치수는 전부
0.310 m 이고, 남는 여유는 0.275 ~ 0.300 m (한쪽당 0.14 ~ 0.15 m)입니다.

여덟 자세 전부와 구간 연결성은 로봇 풋프린트 기준 배위공간(heading 32단계,
격자 0.02 m, 홀로노믹 이동 + 1단계 회전)에서 검증했습니다. P1 → … → P8 → P1
이 끊김 없이 이어져 순환로가 닫힙니다.

ㄷ 자로만 돌고 싶으면 `tools/run_patrol.sh` 에서 뒤쪽 웨이포인트를 지우면
됩니다.

열원 4종은 각각 자기 설비의 남쪽 면에 있습니다.

| ID | 설비 | 온도 | 위치 |
|---|---|---|---|
| `sim-waste-pile` | 벙커 폐기물 더미 | 71.3 ℃ | −1.5098, −0.7180 |
| `sim-hot-motor` | 파쇄기 모터 | 84.6 ℃ | −1.1418, −0.4065 |
| `sim-pump-block` | 2차 처리기 펌프 | 68.4 ℃ | 0.8514, −0.3046 |
| `sim-tank-block` | 압축기 유압 탱크 | 48.2 ℃ | 1.3440, −0.3386 |

같은 값이 `config/heat_sources/demo_facility_scaled.json` 에도 있지만 아직
그 파일을 읽는 코드가 없습니다. 실제로 쓰이는 것은
`thermal_detector_mock.py` 의 `HEAT_SOURCES` 상수이므로, 로더가 생기기 전까지는
둘을 같이 고쳐야 합니다.

카메라는 전방 고정(화각 57°)이고 순찰 중 로봇은 통로와 나란히 서므로, 열원은
정면이 아니라 **접근하는 동안 화각 가장자리에 스쳐 들어옵니다.** 예를 들어
남측 통로를 동쪽으로 달릴 때 파쇄기 모터(y 방향으로 1.01 m 떨어짐)는 약
1.86 m 앞에서부터 화각에 들어오고, 가까워질수록 화각 밖으로 빠집니다. 사거리
5 m 안이라 탐지 자체는 됩니다. mock 탐지기는 가림(occlusion) 판정을 하지
않으므로 설비 뒤에 있어도 잡힙니다.

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
있습니다. 기본 데모 월드를 쓸 때는 `demo_facility_scaled.sdf`입니다.

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

`P1 → … → P8` 로 설비 섬을 한 바퀴 돌고 출발점으로 복귀합니다. 각 지점에서 2초 정차하며,
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
| RGB | `/camera/image_raw`, `/camera/camera_info` |
| Depth | `/depth_camera/image_raw`, `/depth_camera/camera_info`, `/depth_camera/points` |
| 열화상 | `/thermal_camera/image_raw`, `/thermal_camera/camera_info` |
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

### 열화상 카메라 사양과 장착 위치

제조사 사양표 기준입니다(`hazard_guard_sensor_config` 의 `TMC160B`).

| 항목 | 값 |
|---|---|
| 센서 | 비냉각 VOx 마이크로볼로미터, 8~14 µm, 픽셀 피치 12 µm |
| 해상도 / 프레임 | 160×120 / 8.7 Hz |
| 화각 | **57°** (95° 렌즈 옵션도 있음 — 다른 렌즈면 URDF와 프로필을 함께 고쳐야 함) |
| NETD | ≤ 50 mK |
| 측정 범위 | High Gain −10~140 ℃ / Low Gain −10~450 ℃ |
| 정확도 | High Gain ±5 ℃ 또는 ±5% / Low Gain ±10 ℃ 또는 ±10% |
| 인터페이스 | USB-FS (UVC, CDC ACM) |
| 크기 | 보드 38×38 mm / 하우징 45×45×45 mm |

### 카메라 스트림 보기

시뮬레이션이 떠 있는 상태에서 창을 띄웁니다. 기본은 열화상과 뎁스 2개이고,
`show_rgb:=true` 로 RGB도 함께 볼 수 있습니다.

```bash
ros2 launch hazard_guard_simulation camera_view.launch.py
ros2 launch hazard_guard_simulation camera_view.launch.py show_rgb:=true
```

열화상 창은 처음에 평평하게 보입니다. mono16 에 밝기가 아니라 온도가 실려
있어서(켈빈 ×100, 29315 = 20.0 ℃) 툴바의 **Dynamic range** 를 켜야 대비가
생깁니다.

### 카메라 토픽 구조

Fortress 는 이미지 토픽의 마지막 경로 조각을 떼고 `/camera_info` 를 붙여
CameraInfo 토픽을 만듭니다. 그래서 SDF 의 `<topic>` 은 반드시 한 단계 아래에
두어야 합니다.

| SDF `<topic>` | gz CameraInfo | 겹침 |
|---|---|---|
| `/camera`, `/depth_camera`, `/thermal_camera` | 전부 `/camera_info` | 세 카메라의 intrinsics 가 한 토픽을 덮어씀 |
| `/camera/image`, `/depth_camera/image`, `/thermal_camera/image` | `/camera/camera_info` 등 | 카메라별로 분리 |

열화상-뎁스 캘리브레이션은 카메라별 intrinsics 가 있어야 하므로 후자를
씁니다. ROS 쪽 이름은 브리지에서 `<이름>/image_raw` 로 되돌려 기존과 같습니다.
`test/test_camera_topics.py` 가 URDF 의 카메라 토픽과 브리지 목록이 어긋나면
실패합니다.

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
