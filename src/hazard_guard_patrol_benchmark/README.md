# HazardGuard Patrol Benchmark

Gazebo 순찰 임무의 시간, 실제 이동 거리, 웨이포인트 완료율, 열원 관측률과
지도 커버리지를 수집하는 실험용 ROS 2 패키지다. 제품 런타임과 WebUI에는
의존하지 않으며 기존 임무 상태를 읽기만 한다.

`Simulation_env`는 Robot 저장소와 나란한 별도 저장소로 유지한다. 경로는
`simulation_env_path` launch 인자 또는 `HAZARD_GUARD_SIMULATION_ENV` 환경변수로
전달한다. 실행 결과는 기본적으로
`HAZARD_GUARD_WORKSPACE/runtime/benchmarks`에 저장된다.

## 권장 실험 프로필

`real_factory`의 순찰 커버리지와 합성 열원 관측률을 측정할 때는 실제 영상 분석
파이프라인 대신 외부 열원 프로필을 읽는 mock detector를 사용한다. 이 설정은
`Simulation_env`의 `hot-01` 같은 기준 ID와 관측 ID를 일치시키며, 순찰 성능 측정에
불필요한 RGB-D 처리 부하도 줄인다.

```bash
export IGN_GAZEBO_RESOURCE_PATH=/workspace/Simulation_env/gazebo/models:${IGN_GAZEBO_RESOURCE_PATH:-}

ros2 launch hazard_guard_simulation localization.launch.py \
  gui:=false \
  world:=/workspace/Simulation_env/gazebo/worlds/real_factory.sdf \
  world_name:=real_factory \
  map:=/workspace/Simulation_env/gazebo/maps/real_factory.yaml \
  spawn_x:=0.0 spawn_y:=3.45 spawn_z:=0.05 spawn_yaw:=0.0 \
  include_dispenser:=false \
  heat_source_profile:=/workspace/Simulation_env/gazebo/config/heat_sources/real_factory.json \
  use_person_safety:=false \
  use_thermal_pipeline:=false \
  enable_rgbd_mapping:=false \
  use_performance_monitor:=false
```

Gazebo GUI는 환경을 눈으로 확인하는 단일 실행에서만 `gui:=true`로 켜고, 반복 측정은
`gui:=false`로 수행한다. Real Time Factor가 낮은 환경에서는 Nav2의 벽시계 기반
대기시간이 먼저 끝날 수 있으므로 결과의 `real_time_factor`와 실패 사유를 함께 본다.

```bash
export HAZARD_GUARD_SIMULATION_ENV=/workspace/Simulation_env
ros2 launch hazard_guard_patrol_benchmark patrol_benchmark.launch.py \
  world_id:=real_factory
```

Gazebo와 Nav2를 먼저 실행한 뒤 전체 순찰을 반복하려면 다음 명령을 사용한다.

```bash
ros2 run hazard_guard_patrol_benchmark patrol_benchmark_run \
  --world-id real_factory --runs 10
```

이 실행기는 Gazebo를 재시작하지 않는다. `real_factory` 순찰 스크립트가 시작점으로
복귀한다는 계약을 이용해 같은 조건을 반복한다.

## 결과 파일

각 임무는 날짜와 임무 ID로 구분된 디렉터리에 다음 파일을 생성한다.

- `metadata.json`: 월드, 지도, spawn과 두 저장소 커밋
- `summary.json`: 자동 분석용 전체 지표
- `metrics.csv`: 표 계산 도구로 열기 쉬운 핵심 지표
- `trajectory.csv`: Gazebo `/odom` 기준 시각·위치·방향 표본
- `report.md`: 사람이 바로 읽을 수 있는 요약

경로 효율은 완료된 임무에서만 `계획 거리 / 실제 거리 × 100`으로 계산한다. 실패나
취소로 전체 계획 경로를 주행하지 않은 임무는 오해를 막기 위해 `N/A`로 기록한다.
