# HazardGuard Patrol Benchmark

Gazebo 순찰 임무의 시간, 실제 이동 거리, 웨이포인트 완료율, 열원 관측률과
지도 커버리지를 수집하는 실험용 ROS 2 패키지다. 제품 런타임과 WebUI에는
의존하지 않으며 기존 임무 상태를 읽기만 한다.

`Simulation_env`는 Robot 저장소와 나란한 별도 저장소로 유지한다. 경로는
`simulation_env_path` launch 인자 또는 `HAZARD_GUARD_SIMULATION_ENV` 환경변수로
전달한다. 실행 결과는 기본적으로
`HAZARD_GUARD_WORKSPACE/runtime/benchmarks`에 저장된다.

## 2026-08-24 개발 내역

이번 작업에서는 먼저 Docker 실행 환경의 성능을 측정한 뒤, 선택한 환경에서 순찰
임무의 품질을 반복 측정할 수 있도록 계측 범위와 자동화 하네스를 확장했다.

### 1. Docker 실행 프로필 비교

- CPU/GPU, Ogre/Ogre2, GUI/headless 조합별 RTF, CPU, RSS, GPU 사용률을 수집한다.
- LiDAR, RGB, Depth, PointCloud, Thermal 토픽 발행률을 함께 검사한다.
- 단순히 가장 빠른 조합이 아니라 필수 센서 발행률을 만족한 조합만 정상으로 판정한다.
- 현재 PC의 자동 반복 시험 권장값은 `GPU + Ogre1 + headless`이고, 화면 확인
  권장값은 `CPU + Ogre2 + GUI`다.
- 선택 결과는 `config/docker_profile_recommendation.yaml`에 기록했다.

관련 구현:

- `profile.py`: 프로필 측정 데이터 모델과 집계
- `profile_probe.py`: 실행 중 RTF·프로세스·GPU·토픽 발행률 수집
- `profile_select.py`: 여러 실행 결과의 정상 여부 판정과 권장 프로필 선택
- `scripts/run_docker_profile_matrix.ps1`: Docker 프로필 매트릭스 반복 실행
- `scripts/docker/`: 기존 Compose를 수정하지 않고 시험 조건을 덧붙이는 overlay

### 2. 순찰 임무 지표 확장

- 시간별 고유 관측 면적과 전체 공간 커버리지를 기록한다.
- 25%, 50%, 75%, 90% 커버리지 최초 도달 시간과 최장 정체 시간을 계산한다.
- 오도메트리 프레임 수가 아니라 지도 중심 셀 전환을 기준으로 중복 방문률을 계산한다.
- 웨이포인트별 이동 시간, dwell 시간, 실제 거리, 직선거리, 경로 효율과 도착
  위치·방향 오차를 저장한다.
- 열원 ID 기준 Recall·Precision, 중복 이벤트, 최초 탐지 시간과 탐지 거리를 계산한다.
- LaserScan 기반 최소거리와 근접 위험 진입 사건을 기록한다.
- `/plan` 발행 횟수는 `global_plan_update_count`로 기록하며 실제 Nav2 복구 또는
  재계획 횟수와 구분한다.

관련 구현:

- `coverage.py`: 지도 셀 기반 커버리지 누적과 시계열
- `metrics.py`: 경로·구간·안전·탐지·위치추정 지표
- `node.py`: ROS 2 토픽 구독과 임무별 측정 세션 연결
- `report.py`: JSON, CSV, Markdown 결과 생성
- `aggregate.py`: 반복 실행 평균·중앙값·P95·최소·최대·성공률 집계

### 3. 반복 실행 하네스 안정화

- Docker 컨테이너 생성부터 ROS 2 빌드, Gazebo·AMCL·Nav2 실행, 순찰 Action,
  결과 집계와 환경 종료까지 한 명령으로 수행한다.
- Action 서버와 토픽의 존재뿐 아니라 Nav2 managed node 활성 로그를 확인하고
  안정화 시간을 둔 뒤 임무를 시작한다.
- `ros2 action send_goal`의 셸 종료코드가 0이어도 Action 결과가 실패할 수 있으므로
  결과 본문의 `success: true`까지 확인한다.
- 원본 `Simulation_env` SDF는 수정하지 않고 렌더러 전환용 사본을 컨테이너
  `/tmp`에 만든다.
- GUI가 꺼진 센서 렌더링에도 필요한 WSL X11 프록시를 자동 실행하고 종료한다.

관련 구현:

- `runner.py`: 전체 순찰 반복 실행 및 Action 결과 검증
- `scripts/run_docker_patrol_benchmark.ps1`: 종단 간 실행 하네스
- `scripts/x11_proxy.py`: WSLg와 Docker 사이의 로컬 X11 연결 보조

### 4. 검증 결과

| 검증 항목 | 결과 |
|---|---|
| Python 단위 테스트 | 28개 통과 |
| Python `compileall` | 통과 |
| PowerShell Parser 검사 | 두 실행 스크립트 통과 |
| ROS 2 증분 빌드 | 11개 패키지 완료 |
| Docker 프로필 시험 | headless 각 2회, GUI 각 1회 완료 |
| 4m Nav2 스모크 Action | `SUCCEEDED` |
| 자동 결과 파일 생성 | JSON, CSV, Markdown 생성 확인 |

최종 스모크 결과는 시뮬레이션 121.924초, 현실 131.754초, RTF 0.9254였다.
4m 계획에 대해 실제 5.056m를 이동해 경로 효율은 79.108%였고, Gazebo Ground
Truth 기준 도착 오차는 0.306m/1.971°였다. 이 실행은 전체 공장 성능값이 아니라
빌드부터 Action 실행과 결과 저장까지의 통합 계약을 확인한 결과다.

### 5. 확인된 제한사항

- 전체 `real_factory` 18개 웨이포인트 10회 기준선은 아직 수행하지 않았다.
- 현재 시뮬레이션 LaserScan은 15cm 자체 반사 필터를 통과한 유효 거리 표본이
  없어 안전거리와 근접 위험이 `null`이다. 이는 위험이 없다는 뜻이 아니라 센서
  모델 교정이 필요하다는 뜻이다.
- 충돌과 Nav2 recovery 전용 토픽이 설정되지 않으면 해당 지표도 `null`이다.
- 합성 열원은 파이프라인 검증용이며 실제 열화상 카메라의 정확도를 대표하지 않는다.
- HTML 기술 보고서는 생성물로 관리하며 Robot 저장소에는 포함하지 않는다.

## 무엇을 검증하는가

이 패키지는 “로봇이 움직였다”는 사실이 아니라 다음 질문에 수치로 답하기 위해 만든다.

1. 정해진 면적을 관측하는 데 시뮬레이션 시간과 현실 시간이 얼마나 걸리는가?
2. 계획한 웨이포인트를 빠짐없이 방문했고, 각 지점에 얼마나 정확히 도착했는가?
3. 계획 경로에 비해 실제로 얼마나 우회했으며 어느 구간이 병목인가?
4. 열원을 얼마나 빨리, 얼마나 멀리서, 누락이나 오탐 없이 발견했는가?
5. 충돌, 근접 위험, Nav2 복구 또는 경로 갱신이 얼마나 발생했는가?
6. 결과가 다른 PC나 다른 Git 커밋에서도 재현 가능한가?

Gazebo `/odom`은 시뮬레이션의 실제 위치인 Ground Truth로 사용하고, `/amcl_pose`는
로봇이 스스로 추정한 위치로 취급한다. 두 값을 비교하면 “로봇이 실제로 어디에
있었는가”와 “로봇이 자신을 어디에 있다고 믿었는가”의 차이를 측정할 수 있다.

## 실험 설계 원칙

- **한 번에 한 조건만 바꾼다.** 월드, spawn, 지도, 순찰 경로, 센서 설정과 dwell
  시간은 고정하고 비교하려는 Docker 또는 알고리즘 설정만 변경한다.
- **시뮬레이션 시간과 현실 시간을 분리한다.** RTF가 0.5라면 시뮬레이션 1초가
  현실에서는 약 2초 걸린다. 주행 알고리즘은 시뮬레이션 시간으로 비교하고,
  작업자 대기시간과 PC 처리 성능은 현실 시간으로 비교한다.
- **GUI는 반복 시험에서 끈다.** 화면 렌더링이 센서와 물리 계산 자원을 사용하므로
  사람이 확인할 때만 켠다.
- **최초 1회가 아니라 반복 분포를 본다.** 정식 비교는 동일 조건 10회를 권장하며
  평균, 중앙값, P95, 최소, 최대와 성공률을 함께 본다.
- **실패도 결과다.** 실패 실행을 삭제하지 않고 상태와 로그를 보존한다. 완료되지
  않은 임무의 전체 경로 효율은 잘못된 해석을 막기 위해 `N/A`로 기록한다.
- **합성 열원은 실제 탐지 성능이 아니다.** `real_factory` 온도는 placeholder이며
  시뮬레이션 파이프라인과 순찰 설계를 검증하는 용도다.

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

## Docker 실행 프로필

Windows 11, Docker Desktop, WSL2, RTX 3070에서 `real_factory`와 전체 센서 및 Nav2를
동일하게 켜고 비교했다. 10초 준비 후 30초를 측정했으며 headless는 두 번, GUI는 한
번 실행했다. 선택값은 `config/docker_profile_recommendation.yaml`에 기록되어 있다.

| 용도 | 고정 프로필 | RTF 중앙값 | CPU 중앙값 | RSS 중앙값 | 판정 |
|---|---|---:|---:|---:|---|
| 반복 자동 시험 | GPU + Ogre1 + headless | 0.630 | 400% | 1,482 MiB | 권장 |
| 비교용 CPU 반복 시험 | CPU + Ogre2 + headless | 0.458 | 500% | 1,665 MiB | 정상 |
| 화면 확인 | CPU + Ogre2 + GUI | 0.257 | 636% | 2,572 MiB | 권장 |
| GPU 화면 확인 | GPU + Ogre1 + GUI | 0.255 | 437% | 2,516 MiB | 깊이 스트림 4.68 Hz로 탈락 |

GPU headless는 CPU headless보다 RTF가 약 37% 높고 CPU 사용량은 약 20% 낮았다.
WSLg D3D12에서 Ogre2는 텍스처 복사 예외로 종료되어 GPU 프로필만 Ogre1을 사용한다.
GUI에서는 GPU/Ogre1의 RTF 이득이 없고 깊이 스트림이 정상 기준에 미달하므로
CPU/Ogre2를 사용한다. 이는 이 PC에서의 선택이며 Jetson이나 다른 PC의 기본값을
변경하는 정책은 아니다.

PowerShell에서 동일 조건을 다시 측정하려면 다음을 실행한다.

```powershell
Set-Location <hazard-guard-workspace>\Robot
.\src\hazard_guard_patrol_benchmark\scripts\run_docker_profile_matrix.ps1 `
  -Mode all -HeadlessRuns 2 -GuiRuns 1 -WarmupSeconds 10 -DurationSeconds 30
```

원본 `Simulation_env` 월드는 수정하지 않는다. GPU 시험 중 Ogre1이 필요한 경우
컨테이너의 `/tmp/profile_world.sdf`만 생성한다. 결과 원본은
`runtime/benchmarks/docker_profiles/<실행시각>`에, 현재 권장값은 같은 디렉터리의
`recommended-profile.json`에 저장된다. 판정 실패도 RTF와 자원 측정값을 보존하며
`failed_runs`에 센서 스트림 등 구체적인 탈락 사유를 기록한다.

## 처음부터 끝까지 한 번에 실행

Docker Desktop이 실행 중이고 `Robot`, `Simulation_env`, `slam-jetson-env` 저장소가
같은 상위 디렉터리에 있을 때 PowerShell에서 다음을 실행한다.

```powershell
Set-Location <hazard-guard-workspace>\Robot
.\src\hazard_guard_patrol_benchmark\scripts\run_docker_patrol_benchmark.ps1 `
  -Runs 1 -DwellSeconds 3
```

스크립트는 권장 자동 시험 프로필인 GPU/Ogre1/headless로 컨테이너를 만들고 다음을
순서대로 수행한다.

1. 필요한 ROS 패키지를 증분 빌드한다.
2. 원본 월드를 수정하지 않고 `/tmp`에 Ogre1 월드 사본을 만든다.
3. Gazebo, 저장 지도 기반 localization, AMCL, Nav2와 임무 관리자를 실행한다.
4. 벤치마크 노드를 먼저 실행한 뒤 전체 순찰 Action을 전송한다.
5. 각 실행 결과와 전체 집계 JSON, CSV, Markdown을 생성한다.
6. 기본적으로 컨테이너를 종료한다. 조사 목적으로 남기려면 `-KeepEnvironment`를 쓴다.

설치·토픽·결과 파일 계약만 빠르게 확인할 때는 약 4m 직선 구간을 사용하는
`-SmokeTest`를 붙인다. 이 결과는 전체 면적이나 정식 순찰 성능 근거로 사용하지 않는다.

```powershell
.\src\hazard_guard_patrol_benchmark\scripts\run_docker_patrol_benchmark.ps1 `
  -Runs 1 -DwellSeconds 1 -SmokeTest
```

2026-08-24 통합 검증에서는 이 스모크 임무가 정상 완료됐다. 4m 목표에 대해 실제
이동거리는 5.056m, 경로 효율은 79.1%, 시뮬레이션 시간은 121.9초, 현실 시간은
131.8초, RTF는 0.925였다. Nav2 임무 메시지는 최종 오차 9.8cm/1.24°를 보고했고,
Gazebo Ground Truth로 별도 계산한 도착 오차는 30.6cm/1.97°였다. 서로 다른 기준의
값이므로 둘 중 하나를 임의로 선택하지 말고 함께 비교한다. 이 실행은 계측 계약과
실행 자동화가 연결되었음을 확인한 사례일 뿐, 전체 공장 성능의 대표값은 아니다.

정식 비교 예시는 다음과 같다.

```powershell
.\src\hazard_guard_patrol_benchmark\scripts\run_docker_patrol_benchmark.ps1 `
  -Runs 10 -DwellSeconds 3 -ContinueOnFailure
```

전체 `real_factory` 경로는 18개 점검점과 시작점 복귀를 포함하므로 PC의 RTF와
로봇 속도에 따라 오래 걸릴 수 있다. 먼저 1회로 계약과 경로를 확인한 뒤 10회를
실행한다.

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
- `coverage_timeseries.csv`: 시간에 따른 관측 면적과 커버리지
- `segments.csv`: 웨이포인트 구간별 시간·거리·도착 오차
- `detections.csv`: 열원 탐지 시각·거리·온도·신뢰도와 정답 여부
- `report.md`: 사람이 바로 읽을 수 있는 요약

경로 효율은 완료된 임무에서만 `계획 거리 / 실제 거리 × 100`으로 계산한다. 실패나
취소로 전체 계획 경로를 주행하지 않은 임무는 오해를 막기 위해 `N/A`로 기록한다.

## 지표 정의와 읽는 법

| 지표 | 정의 | 좋은 방향 | 주의점 |
|---|---|---|---|
| 공간 커버리지 | 순찰 가능한 free 셀 중 관측 반경 안에 들어온 고유 셀 비율 | 높음 | 카메라 실제 FOV가 아니라 설정된 점검 반경 기반 |
| 90% 도달 시간 | 공간 커버리지가 최초로 90% 이상이 된 시뮬레이션 시간 | 낮음 | 최종 90% 미달이면 `null` |
| 커버리지 속도 | 최종 고유 관측 면적 ÷ 시뮬레이션 시간 | 높음 | 빠르게 중복 주행하면 개선되지 않음 |
| 중복 방문 비율 | 경로 보간 중심 셀 방문 중 이미 방문한 셀의 비율 | 낮음 | 안전 우회와 복귀 구간은 필요한 중복일 수 있음 |
| 경로 효율 | 완료 임무에서 계획 거리 ÷ 실제 거리 × 100 | 100%에 가까움 | 실패 임무에는 계산하지 않음 |
| 구간 경로 효율 | 직선거리 ÷ 해당 구간 실제 이동거리 × 100 | 100%에 가까움 | 장애물을 피한 정상 우회도 효율을 낮춤 |
| 도착 위치/방향 오차 | 웨이포인트 목표와 도착 시 Ground Truth의 차이 | 낮음 | AMCL 오차와는 별개 |
| 최소 장애물 거리 | 각 `/scan`의 유효 거리 중 전체 최솟값 | 높음 | LiDAR 사각지대는 포함하지 못함 |
| 근접 위험 | 최소 거리가 설정 임계값 아래로 진입한 연속 사건 수 | 낮음 | 프레임 수가 아니라 진입 사건 단위 |
| 전역 경로 갱신 수 | `/plan` 메시지를 받은 횟수 | 상황에 따라 해석 | Nav2의 실제 재계획·복구 횟수와 동일하지 않음 |
| 열원 Recall | 기준 열원 중 한 번 이상 탐지한 열원 비율 | 높음 | 합성 기준 데이터에 대한 결과 |
| 열원 Precision | 고유 탐지 ID 중 기준 열원 ID의 비율 | 높음 | ID 계약이 맞지 않으면 오탐처럼 보일 수 있음 |
| 최초 탐지 시간 | 순찰 시작 후 첫 기준 열원을 발견한 시간 | 낮음 | 경로 순서의 영향을 받음 |
| RTF | 시뮬레이션 경과 시간 ÷ 현실 경과 시간 | 1에 가까움 | 1보다 작으면 실제보다 느리게 실행됨 |

`summary.json`은 한 실행의 최종 결과이고 `coverage_timeseries.csv`는 진행 과정을
보여준다. 평균만 보면 간헐적인 긴 지연이 가려질 수 있으므로 반복 집계에서는
중앙값과 P95를 우선 확인한다. P95는 실행 100회 중 느린 쪽 약 5회 수준을 나타내는
보수적 지표다.

안전거리 계산은 기본적으로 15cm 미만 반환값을 로봇 본체의 자체 반사로 간주해
제외한다. 결과의 `scan_message_count`는 받은 LaserScan 메시지 수,
`usable_scan_sample_count`는 유효 최솟값을 계산할 수 있었던 메시지 수,
`filtered_range_count`는 거리 범위 조건으로 제외한 유한값 수다. 메시지는 받았지만
사용 가능한 표본이 0이면 최소거리와 근접 위험을 `0`으로 채우지 않고 `null`로
남긴다. 이 경우는 “안전했다”가 아니라 “현 센서 모델로 측정하지 못했다”는 뜻이며,
LiDAR 장착 위치·collision mesh·self-filter를 먼저 교정해야 한다.

## 유효성 확인 순서

결과 비교 전에 다음을 확인한다.

1. `status`가 `completed`인지 확인한다.
2. RTF와 센서 스트림 상태가 실험 기준을 만족하는지 확인한다.
3. `dropped_jump_count`가 증가하지 않았는지 확인한다.
4. 웨이포인트 실패 ID와 구간별 `status`를 확인한다.
5. Git 커밋, 월드, 지도, spawn이 비교군과 같은지 확인한다.
6. 열원 온도가 실제 측정값인지 placeholder인지 구분한다.

Docker 프로필 자체의 센서 발행률과 자원 사용량은
`run_docker_profile_matrix.ps1`이 담당하고, 순찰 임무의 커버리지·정확도·안전성은
`run_docker_patrol_benchmark.ps1`이 담당한다. 두 결과를 섞지 않고 함께 보아야
“시뮬레이터가 느려서 실패한 것”과 “경로 또는 Nav2 정책이 나빠서 실패한 것”을
구분할 수 있다.
