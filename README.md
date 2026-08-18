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
- YOLO11n 사람 탐지와 RGB-D 거리 추정
- Nav2 SpeedLimit 감속, 최종 `cmd_vel` 정지 게이트, 순찰 일시정지·재개
- FastAPI WebUI bridge에서 사용할 ROS 토픽과 액션

Jetson 전용 CUDA·TensorRT, 실제 ROSMASTER 하드웨어 드라이버, 경고장치 제어는
아직 포함하지 않습니다.

## 패키지 구성

```text
src/
├─ hazard_guard_interfaces/    메시·서비스·순찰 Action 정의
├─ hazard_guard_mission_manager/ Nav2 순찰 임무 실행 노드
├─ hazard_guard_person_detection/ YOLO11n 사람 탐지·RGB-D 거리
├─ hazard_guard_safety_supervisor/ 안전 상태·Nav2 감속·최종 속도 게이트
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

## 사람 탐지와 안전 제어

사람 탐지는 Nav2 장애물 회피를 대체하지 않습니다. Nav2가 경로 계획과 회피를
계속 담당하고, YOLO11n과 Depth가 사람의 의미·거리를 제공하여 감속 또는 최종
정지를 추가합니다. 기본 launch에서는 비활성화되어 있으며 모델과 Jetson 환경을
검증한 뒤 `use_person_safety:=true`로 켭니다.

- 구조·토픽·실행·실물 검증: [PERSON_SAFETY_ARCHITECTURE.md](PERSON_SAFETY_ARCHITECTURE.md)
- YOLO·Jetson 환경 설치와 기록: [YOLO_JETSON_SETUP.md](YOLO_JETSON_SETUP.md)

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

### 반복·예약 순찰

`RunPatrol` 액션은 1회, 지정 횟수, 실제 종료 시각까지 반복, 수동 종료까지 반복을
지원합니다. 예약 시작·종료 시각은 Unix 밀리초로 전달되므로 WebUI의 시간대와
무관하게 같은 순간을 가리킵니다. 회차 사이에는 `repeat_interval_sec`만큼
대기하며, 대기 중에도 취소할 수 있습니다. 종료 시각에 도달하면 현재 Nav2
이동을 취소하고 임무를 정상 종료로 기록합니다.

예약과 반복은 브라우저가 아닌 `hazard_guard_mission_manager`가 처리합니다.
따라서 WebUI 새로고침이나 노트북 네트워크 단절에도 Jetson 노드가 살아 있는 한
순찰은 계속됩니다. 실제 시간 예약을 사용하기 전에는 Jetson의 시간대와 NTP
동기화 상태를 확인하십시오. 인터페이스가 변경되었으므로 기존 설치에서는
`hazard_guard_interfaces`와 `hazard_guard_mission_manager`를 다시 빌드해야 합니다.

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
나타냅니다. 저장 2D 지도 기반의 두 번째 주행에서는 RGB·Depth를 RTAB-Map
노드별로 기록하고, `map_assembler`가 AMCL에서 받은 노드 pose로 컬러 표면
지도를 다시 만듭니다. 기본 프로필은 RTAB visual/ICP odometry와 loop closure를
사용하지 않으므로 이미 누적한 점군을 RTAB-Map이 독자적으로 재배치하지 않습니다.
결과는 `map` 좌표계의
`/hazard_guard/rtabmap/cloud_surface`로 발행합니다. RTAB-Map의 TF 발행은
끄므로 `map -> odom`은 계속 AMCL만 담당하며 Nav2 좌표계와 경쟁하지 않습니다.
각 점은 X/Y/Z와 RGB를 포함하며 WebUI 백엔드가 이를 다운샘플링해 브라우저로
전송합니다. RViz는 이 데이터를 보는 도구일 뿐 WebUI의 데이터 원본은 아닙니다.

실제 로봇 전환 시 RTAB-Map 알고리즘 코드를 다시 만들 필요는 없지만,
Gazebo 카메라 토픽 대신 실제 RGB·Depth·CameraInfo·Odometry·TF를 연결해야
합니다. 특히 RGB 카메라와 Depth 카메라의 내부 파라미터 및 센서와
`base_link` 사이 외부 캘리브레이션이 선행되어야 합니다.

이 컬러 클라우드는 센서가 관측한 표면의 3D 복원 결과이므로 가려진 면과 아직
주행하지 않은 구역은 포함하지 않습니다. Gazebo SDF의 모든 벽과 메시를 완전한
형태로 표시하는 것은 SLAM 검증과 구분되는 시뮬레이터 원본 디지털 트윈
기능입니다.

### 실제 Jetson 3D 지도 부하 제어

실제 로봇의 `physical_mapping.launch.py`에는 주행 계층과 독립된 3D cloud
guard가 포함됩니다. 실기 비교에서 9,000 points/frame, decimation 2,
3 cm voxel이 CPU/RAM 평균 약 50%를 유지하면서 12,000점 설정과 체감 품질
차이가 작아 기본값으로 선정되었습니다. 정상 상태에서는 프레임당 최대 9,000점과
8 Hz로 누적 입력을 제한하고, 누적 지도는 3 cm voxel 및 10 cm/6° keyframe 조건을
사용합니다. 누적 결과는 첫 keyframe부터 발행하며 정지 중 중복 프레임은
지도에 추가하지 않습니다. CPU/GPU/RAM/온도/지도 저장 디스크 부하가 지속되면
4,500점과 4 Hz로 낮추며, 임계 부하에서는 3D 표면 누적만 일시 중지합니다. 이때
SLAM Toolbox, Nav2, RTAB-Map 위치 추정 및 RTAB-Map DB 기록 경로는 계속
동작합니다.

여기서 `9,000`은 **누적 프레임 수가 아니라 입력 한 프레임의 최대 포인트
수**입니다. `cloud_voxel_size:=0.03`의 단위는 미터이므로 **0.03 m = 3 cm**이며
0.03 cm가 아닙니다. 최적화 후의 전체 누적 지도는 9,000점을 넘을 수 있습니다.
WebUI 백엔드는 그 전체 지도에 복셀화를 다시 적용하지 않고, 기본 최대 20,000점을
균일 간격으로 골라 전송하므로 Robot의 3 cm 공간 해상도를 중복 변경하지 않습니다.

누적 지도는 WebUI 호환 토픽인
`/hazard_guard/rtabmap/cloud_surface`를 유지하며 최대 1 Hz로 전달됩니다.
현재 managed WebUI의 legacy 토픽 설정과 호환되도록 같은 누적 지도를
`/hazard_guard/rtabmap/cloud_frame_raw`에도 발행합니다. 실제 센서 프레임은
내부 토픽 `/hazard_guard/rtabmap/cloud_frame_generated`에서 확인할 수 있습니다.
현재 품질 모드와 부하, 입출력 포인트 수는 다음 토픽에서 확인합니다.

```bash
ros2 topic echo /hazard_guard/rtabmap/cloud_guard/status
```

실제 로봇 launch의 기본값은 필요하면 인자로 조정할 수 있습니다.

```bash
ros2 launch hazard_guard_simulation physical_mapping.launch.py \
  cloud_normal_points:=9000 \
  cloud_high_load_points:=4500 \
  cloud_normal_input_hz:=8.0 \
  cloud_high_load_input_hz:=4.0 \
  cloud_decimation:=2 \
  cloud_voxel_size:=0.03
```

### 포인트클라우드 Jetson 부하 테스트

`tools/run_pointcloud_benchmark.sh`는 선택한 포인트 밀도 프로필로 FastAPI만
실행합니다. 실제 mapping stack과 세션 DB는 WebUI가 관리합니다. WebUI에서
`2D + RGB-D 3D`를 선택하고 `새 맵 생성`을 누른 뒤 같은 경로를 주행하면서
3D 화면과 Jetson CPU/RAM을 수동 기록합니다.

소스를 변경한 뒤 최초 한 번 패키지를 다시 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/RoboLotus/hazard-guard-robot
colcon build --symlink-install --packages-select hazard_guard_simulation
source install/setup.bash
```

운영 기본 프로필의 백엔드를 실행합니다. 프로필을 생략하면 자동으로
`pc-9000`, voxel 3 cm, decimation 2를 사용합니다. 스크립트는 Tailscale
IPv4를 자동으로 찾으며, 필요하면 `--host`로 직접 지정할 수 있습니다.

```bash
./tools/run_pointcloud_benchmark.sh

# 또는 Tailscale IP를 명시
./tools/run_pointcloud_benchmark.sh --host 100.107.60.123
```

비교 또는 회귀 시험에서는 프로필과 voxel 크기를 명시적으로 덮어쓸 수 있습니다.

```bash
./tools/run_pointcloud_benchmark.sh pc-3000 --voxel-size 0.08
./tools/run_pointcloud_benchmark.sh pc-6000 --voxel-size 0.05
./tools/run_pointcloud_benchmark.sh pc-12000 --voxel-size 0.03
```

voxel이 작을수록 가까운 점들이 덜 병합되어 누적 지도 포인트, CPU 및 RAM 사용량이
증가합니다. 프로필 또는 voxel을 바꾸기 전에는 현재 WebUI mapping과 FastAPI를
완전히 종료한 뒤 새 세션을 시작합니다.

지원 프로필은 다음과 같습니다.

| 프로필 | 포인트/프레임 | 입력률 | decimation | voxel |
|---|---:|---:|---:|---:|
| `pc-3000` | 3,000 | 8 Hz | 4 | 실행 옵션(기본 3 cm) |
| `pc-6000` | 6,000 | 8 Hz | 4 | 실행 옵션(기본 3 cm) |
| `pc-9000` | 9,000 | 8 Hz | 2 | 실행 옵션(기본 3 cm) |
| `pc-12000` | 12,000 | 8 Hz | 2 | 실행 옵션(기본 3 cm) |

백엔드가 실행되면 노트북 프론트엔드를 해당 Tailscale 주소에 연결합니다.

```bash
HAZARD_GUARD_BACKEND_URL=http://100.107.60.123:8000 npm run dev
```

프로필을 바꾸기 전 WebUI에서 `지도 저장 후 종료`로 현재 mapping을 끝내고,
FastAPI 터미널에서 `Ctrl+C`를 누릅니다. 다음 프로필로 스크립트를 다시 실행한 뒤
WebUI에서 새 3D 맵 세션을 시작합니다. 서로 다른 프로필의 ROS stack을 동시에
실행하지 않습니다. 안전 장치는 계속 활성화되며 high-load에서는 포인트와 입력률을
절반으로 낮추고 critical에서는 3D 누적 입력을 중지합니다.

스크립트는 cloud guard 상태가 수신되는 3D mapping 구간만 1초 간격으로 측정합니다.
FastAPI 터미널에서 `Ctrl+C`를 누르면 백엔드와 WebUI 관리 ROS stack을 정상 종료한
후 CPU 평균, RAM 평균 점유율, 설정 포인트/Hz/voxel, 실제 출력 포인트와 guard
모드별 시간을 터미널에 요약합니다. WebUI에서 3D mapping을 시작하지 않았다면
측정 결과 없음으로 표시됩니다.

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
| RGB | `/camera/image_raw`, `/camera/camera_info` |
| Depth | `/depth_camera/image_raw`, `/depth_camera/camera_info`, `/depth_camera/points` |
| RTAB-Map 컬러 3D 지도 | `/hazard_guard/rtabmap/cloud_surface` |
| 3D 지도 부하 상태 | `/hazard_guard/rtabmap/cloud_guard/status` |
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

장착은 **RGB-D 유닛 옆**입니다. 위에 얹을 수 없습니다 — LiDAR 스캔 평면이
base_link 기준 z 0.14712 인데 카메라 하우징 윗면이 이미 0.1197 이라, 38 mm
보드를 그 위에 올리면 0.158 까지 올라가 전방 스캔을 가립니다.

옆에 두면 보드가 0.087~0.125 구간에 머물러 스캔 평면 아래로 빠지고, 두 광학
프레임이 x·z 는 같고 y 만 68 mm 벌어진 **순수 횡방향 베이스라인**이 됩니다.
캘리브레이션 결과를 눈으로 검산하기 가장 쉬운 배치입니다. 하우징과는 4 mm,
차체 옆면보다 6.5 mm 바깥이며 Nav2 풋프린트(반폭 0.155 m) 안입니다.

장착 위치는 xacro 인자입니다. 캘리브레이션이 알려진 오차를 복원하는지
시험할 때 일부러 틀어놓는 용도입니다.

```bash
xacro ... thermal_mount_y:=0.073 thermal_mount_yaw:=0.035   # 5 mm, 2° 틀기
```

`test/test_thermal_mount.py` 가 LiDAR 평면 침범, 하우징 겹침, 베이스라인
정렬을 검사합니다.

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

카메라마다 SDF 에 `<camera_info_topic>` 을 **명시**합니다. 이걸 빼면 Fortress
가 이미지 토픽의 마지막 경로 조각을 떼고 `/camera_info` 를 붙여 만드는데,
세 카메라 토픽이 모두 루트라 전부 `/camera_info` 하나로 접힙니다.

| 설정 | gz CameraInfo | 결과 |
|---|---|---|
| `<camera_info_topic>` 없음 | 셋 다 `/camera_info` | 마지막에 발행한 카메라 값만 남음 |
| `<camera_info_topic>` 명시 | `/camera/camera_info` 등 | 카메라별로 분리 |

브리지도 정상 동작하고 토픽도 계속 갱신되므로 **조용히 틀린 값이 흐릅니다.**
열화상-뎁스 캘리브레이션은 카메라별 intrinsics 가 있어야 성립하므로 명시가
필수입니다. `test/test_camera_topics.py` 가 `<camera_info_topic>` 누락, 토픽
충돌, 브리지 목록 누락을 검사합니다.

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
