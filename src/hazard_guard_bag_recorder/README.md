# HazardGuard ROS Bag Recorder

실주행 ROS 2 토픽을 목적별로 기록하고, 세션 단위의 `session.json`과 간단한 주행 측정값을 남기는 패키지다.

## 범위

- 명시한 토픽만 `ros2 bag record`로 기록
- 실행 전 토픽 존재 여부와 디스크 여유 공간 점검
- 시작·중지 서비스, 최대 시간·용량 제한, 세션 manifest
- `/odom` 기반 이동 거리·속도·정지 시간 요약

YOLO 토픽과 `--all` 기록은 의도적으로 사용하지 않는다. WebUI 연결과 센서 드라이버 설치도 이 패키지 범위 밖이다.

## 프로파일

| 프로파일 | 용도 | 상태 |
| --- | --- | --- |
| `navigation-core` | 2D SLAM, AMCL, Nav2 주행 | 즉시 사용 가능 |
| `rgbd-mapping` | RGB-D / RTAB-Map 짧은 맵 작성 | 카메라 토픽 확인 후 사용 |
| `patrol-core` | 웨이포인트 순찰과 열화상 분석 결과 | 즉시 사용 가능 |
| `thermal-calibration` | 열화상-깊이 보정 캡처 | 실험적, 실제 토픽 확인 필요 |
| `patrol-thermal` | 원본 열화상·깊이 증거를 포함한 순찰 | 실험적, 실제 토픽 확인 필요 |

실험적 프로파일은 `allow_experimental:=true` 없이는 시작하지 않는다. 각 프로파일의 후보 토픽 별칭은 `config/profiles.json`에 있으므로, 실제 Jetson에서 `ros2 topic list -t` 결과에 맞춰 추가·검토한다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/hazard-guard-robot/install/setup.bash

ros2 launch hazard_guard_bag_recorder bag_record.launch.py \
  profile:=navigation-core session_name:=2d-map-baseline auto_start:=true
```

기본 저장 위치는 `~/.local/share/hazard_guard/bags/`이다. 팀 공유나 대용량 SSD를 쓸 때만 `storage_root:=/mnt/ssd/hazard_guard-bags`처럼 변경한다.

기본값은 한 세션 **30분**, bag 데이터 **10 GiB**, 최소 여유 공간 **2 GiB**다. 제한 없이 쓰려면 운영자가 각 값을 명시적으로 `0`으로 바꿔야 한다. 이 값은 세션 시작 시 고정되므로, 기록 중 파라미터 변경으로 상한을 완화할 수 없다.

시작·중지 ROS 서비스는 기본적으로 열지 않는다. 같은 ROS 도메인 안의 다른 노드가 임의로 기록을 시작·중지하는 것을 막기 위한 안전 기본값이다. 신뢰한 폐쇄망에서만 `enable_control_services:=true`로 명시적으로 열고 다음을 사용한다.

```bash
ros2 launch hazard_guard_bag_recorder bag_record.launch.py \
  profile:=navigation-core enable_control_services:=true
ros2 service call /hazard_guard/bag/start std_srvs/srv/Trigger '{}'
ros2 service call /hazard_guard/bag/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /hazard_guard/bag/status
```

열화상 보정 예시는 다음과 같다. 시작 전에 실제 카메라의 메시지 타입·QoS·토픽명을 반드시 확인한다.

```bash
ros2 topic list -t
ros2 launch hazard_guard_bag_recorder bag_record.launch.py \
  profile:=thermal-calibration allow_experimental:=true \
  session_name:=thermal-calibration-01 auto_start:=true max_duration_seconds:=180
```

## 산출물과 확인

각 세션은 시간·이름·난수로 구분한 하위 폴더에 저장된다.

```text
~/.local/share/hazard_guard/bags/
  20260820T120000-2d-map-baseline-a1b2c3d4/
    bag/             # rosbag2 데이터
    rosbag.log       # ros2 bag 프로세스 출력
    session.json     # 선택/누락 토픽, 종료 사유, 용량, 주행 요약
```

`sqlite3` 저장 형식에서는 종료 시 토픽별 메시지 수·평균 주기를 `session.json`에 기록한다. MCAP 토픽 요약은 v1에서 제공하지 않지만, 원본 MCAP은 정상 저장된다.

## 제한과 운영 원칙

- 실제 열화상 카메라 토픽·타입·QoS는 아직 확정되지 않았으므로, 열화상 프로파일을 기본 운용에 쓰지 않는다.
- 카메라 원본·점군은 CPU, RAM보다 주로 디스크 I/O와 저장 용량을 크게 사용한다. `rgbd-mapping`, `patrol-thermal`은 짧게 기록한다.
- 기록 실패 또는 토픽 누락은 `session.json`에 남기며, 필수 토픽 누락 시 기록을 시작하지 않는다.
- 신뢰할 수 없는 장치가 같은 ROS/DDS 도메인에 참여할 수 있다면 서비스 opt-in만으로는 충분하지 않다. SROS2 또는 네트워크 분리도 적용한다.
- `runtime/bags/`, `*.db3`, `*.mcap`은 Git에서 제외한다. 실제 데이터·개인 경로·비밀값을 커밋하지 않는다.
