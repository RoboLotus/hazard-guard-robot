# HazardGuard Patrol Benchmark

Gazebo 순찰 임무의 시간, 실제 이동 거리, 웨이포인트 완료율, 열원 관측률과
지도 커버리지를 수집하는 실험용 ROS 2 패키지다. 제품 런타임과 WebUI에는
의존하지 않으며 기존 임무 상태를 읽기만 한다.

`Simulation_env`는 Robot 저장소와 나란한 별도 저장소로 유지한다. 경로는
`simulation_env_path` launch 인자 또는 `HAZARD_GUARD_SIMULATION_ENV` 환경변수로
전달한다. 실행 결과는 기본적으로
`HAZARD_GUARD_WORKSPACE/runtime/benchmarks`에 저장된다.

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
