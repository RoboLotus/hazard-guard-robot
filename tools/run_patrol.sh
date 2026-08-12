#!/usr/bin/env bash
# worlds/demo_facility_scaled.sdf 의 외곽 통로를 한 바퀴 자동 순찰한다.
# 설비가 홀 가운데 모여 있고 둘레가 통로라 경로는 닫힌 순환로다. 남 -> 남동 ->
# 동 -> 북동 -> 북 -> 북서 -> 서 -> 남서 순으로 반시계 방향으로 돈다.
#
# yaw 는 설비 쪽이 아니라 통로와 나란한 방향이다. 통로가 0.585 ~ 0.610 m 인데
# 로봇이 0.600 x 0.310 m 라서, 통로를 가로질러 서면 (설비를 바라보면) 길이
# 0.600 m 가 통로 폭을 그대로 다 먹는다. 남/서(0.609, 0.610 m)는 여유가 1 cm
# 미만이고 북/동(0.585, 0.590 m)은 아예 들어가지 않는다. 그래서 어느 지점에서도
# 제자리에서 설비를 마주 볼 수 없다. 통로와 나란히 서면 폭 0.310 m 만 쓰므로
# 양쪽에 0.14 m 씩 남는다.
#
# 사전 조건: navigation.launch.py 또는 localization.launch.py 가 떠 있어야 한다.
# 둘 다 Nav2 와 hazard_guard_mission_manager 를 함께 올린다. simulation.launch.py
# 나 slam.launch.py 만으로는 Nav2 가 없어 액션 서버가 뜨지 않는다.
#
# 좌표는 통로 중심선 위의 점이고, 각 자세와 구간 연결성을 로봇 풋프린트 기준
# 배위공간에서 검증했다. 방 크기나 배치를 바꿨다면 좌표를 다시 뽑을 것.
set -euo pipefail

MISSION_ID="${1:-demo-$(date +%s 2>/dev/null || echo manual)}"
DWELL="${PATROL_DWELL_SECONDS:-2.0}"

exec ros2 action send_goal --feedback /hazard_guard/run_patrol \
  hazard_guard_interfaces/action/RunPatrol \
"{mission_id: '${MISSION_ID}', name: '외곽 통로 순환 순찰', frame_id: 'map',
  return_to_start: true,
  waypoints: [
    {id: 'P1', name: '남동',  x:  2.645, y: -1.417, yaw:  0.00000, dwell_seconds: ${DWELL}},
    {id: 'P2', name: '동측',  x:  2.645, y:  0.000, yaw:  1.57080, dwell_seconds: ${DWELL}},
    {id: 'P3', name: '북동',  x:  2.645, y:  1.405, yaw:  1.57080, dwell_seconds: ${DWELL}},
    {id: 'P4', name: '북측',  x:  0.000, y:  1.405, yaw:  3.14159, dwell_seconds: ${DWELL}},
    {id: 'P5', name: '북서',  x: -2.655, y:  1.405, yaw:  3.14159, dwell_seconds: ${DWELL}},
    {id: 'P6', name: '서측',  x: -2.655, y:  0.000, yaw: -1.57080, dwell_seconds: ${DWELL}},
    {id: 'P7', name: '남서',  x: -2.655, y: -1.417, yaw: -1.57080, dwell_seconds: ${DWELL}},
    {id: 'P8', name: '남측',  x:  0.098, y: -1.417, yaw:  0.00000, dwell_seconds: ${DWELL}}
  ]}"
