#!/usr/bin/env bash
# 데모 월드의 남측 통로를 자동 순찰한다. 수동 주행이 필요 없다.
#
# 사전 조건: navigation.launch.py 또는 localization.launch.py 가 떠 있어야 한다.
# 둘 다 Nav2 와 hazard_guard_mission_manager 를 함께 올린다. simulation.launch.py
# 나 slam.launch.py 만으로는 Nav2 가 없어 액션 서버가 뜨지 않는다.
#
# 좌표는 tools/gen_demo_world.py 가 주행 가능 영역으로 검증한 웨이포인트다.
# 방 크기나 배치를 바꿨다면 그 스크립트를 다시 돌려 좌표를 갱신할 것.
set -euo pipefail

MISSION_ID="${1:-demo-$(date +%s 2>/dev/null || echo manual)}"
DWELL="${PATROL_DWELL_SECONDS:-2.0}"

exec ros2 action send_goal --feedback /hazard_guard/run_patrol \
  hazard_guard_interfaces/action/RunPatrol \
"{mission_id: '${MISSION_ID}', name: '남측 통로 순찰', frame_id: 'map',
  return_to_start: true,
  waypoints: [
    {id: 'P1', name: '벙커 앞',      x: -1.29, y: -1.07, yaw: 0.0, dwell_seconds: ${DWELL}},
    {id: 'P2', name: '파쇄기 앞',    x:  0.13, y: -0.99, yaw: 0.0, dwell_seconds: ${DWELL}},
    {id: 'P3', name: '2차 처리기',   x:  1.51, y: -0.99, yaw: 0.0, dwell_seconds: ${DWELL}},
    {id: 'P4', name: '베일 창고',    x:  2.93, y: -0.99, yaw: 0.0, dwell_seconds: ${DWELL}}
  ]}"
