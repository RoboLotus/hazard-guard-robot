# HazardGuard 디스펜서 개발 노트

이 문서는 Jetson의 기존 독립 작업공간인
`/home/jetson/dispenser_ws`에서 가져온 디스펜서 코드를 계속 개발하기 위한
현황, 의존성, 알려진 문제와 작업 목록을 기록한다.

## 현재 상태

- 기준 브랜치: `dev` (`751916c`)
- 최초 반입일: 2026-08-19
- 원본 패키지: `/home/jetson/dispenser_ws/src/hazard_guard_dispenser`
- 하드웨어 연결: Jetson USB `/dev/myserial` -> Rosmaster 확장보드 S1 -> MG946R
- 코드 안전 기본값: ID 1, 대기 0도, 배출 30도
- 실물 프로필: `config/dispenser_physical.yaml`, 대기 0도, 배출 60도
  (2026-08-21 팀원 실측 반영, 실제 디스펜서 조립 완료 후 재검증 필요)
- BLE 프로토콜: 비콘 큐브에 ARM/CANCEL을 전송하고 낙하 보고를 수신

현재 코드는 `physical_patrol.launch.py`와 mission manager에 선택적으로 연결된다.
`use_dispenser`, `enable_hazard_approval`, `enable_physical_drop`은 모두 기본값이
`false`다. 운영 승인 흐름을 명시적으로 켜기 전까지
`enable_physical_drop=false`가 유지되며,
명시적으로 활성화해도 Rosmaster 하드웨어와 BLE ARM 성공 큐브가 최소 1개
확인되지 않으면 서보를 움직이지 않는다. 또한 `/odom`의 선속도와 각속도가
각각 기본 `0.02 m/s`, `0.05 rad/s` 이하로 0.5초간 유지되어야 한다. odom이
0.75초 이상 끊겨도 정지로 간주하지 않는다.

노드 시작·종료 시 자동 home 동작도 기본 비활성화되어 있다. 정비 시험에서
필요할 때만 `home_on_startup` 또는 `home_on_shutdown`을 명시적으로 켠다.

## ROS 인터페이스

운영 배출은 타입이 있는 ROS Action을 사용한다. `request_id`, `detection_id`,
HMAC 승인값이 Action goal에 포함되고 진행 상태와 최종 결과가 같은 goal에
귀속된다. 문자열 토픽은 상태 호환성과 제한된 정비 명령에만 남겨 둔다.

| 방향 | 이름 | 형식 | 값 |
| --- | --- | --- | --- |
| Action | `/hazard_guard/dispenser/dispense` | `DispenseBeacon` | 서명된 배출 요청, 진행 피드백, 취소 및 최종 결과 |
| 구독 | `/hazard_guard/dispenser/command` | `std_msgs/String` | 운영 배출에는 사용하지 않음. 허용된 정비 명령만 처리 |
| 발행 | `/hazard_guard/dispenser/status` | `std_msgs/String` | `ready`, `busy`, `dropped`, `jam_suspected`, `error:...` |
| 발행 | `/hazard_guard/dispenser/result` | `std_msgs/String` | request ID별 진행·최종 결과 JSON |
| 발행 | `/hazard_guard/dispenser/battery` | `std_msgs/String` | BLE 비콘별 전압·상태 JSON |
| 서비스 | `/hazard_guard/dispenser/request_status` | `DispenserRequestStatus` | SQLite 원장 결과와 detection ID 일치 확인 |
| 서비스 | `/hazard_guard/incidents/decision` | `HazardDecision` | 서명된 관리자 결정 처리 |

실행 예:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hazard_guard_dispenser dispenser_node --ros-args -p use_cube_ble:=false
```

실물 60도 프로필을 사용하되 실제 배출은 차단한 상태로 시작하려면 다음 launch를
사용한다.

```bash
ros2 launch hazard_guard_dispenser dispenser.launch.py
```

`enable_physical_drop:=true`는 정지·BLE·전원·기구 체크리스트를 통과한 실물
시험에서만 명시한다. 일반 `ros2 run`은 코드 안전 기본값인 30도를 사용하므로
실물 디스펜서 운용 명령으로 사용하지 않는다.

실물 통합 시험에서는 세 프로세스가 같은 승인 비밀값을 사용한다. 이 값은
저장소나 shell history에 기록하지 말고 Jetson의 권한 제한 환경 파일로
주입한다.

먼저 권한이 제한된 파일을 만들고 편집기에서 값을 입력한다. 실제 비밀값을
명령행 인자에 직접 쓰지 않는다.

```bash
sudo install -d -m 755 /etc/hazard-guard
sudo install -o "$USER" -g "$(id -gn)" -m 600 /dev/null \
  /etc/hazard-guard/dispenser.env
sudoedit /etc/hazard-guard/dispenser.env
# 편집기 안에서 다음 한 줄을 작성:
# HAZARD_GUARD_DISPENSER_APPROVAL_SECRET=<외부에서 생성한 32바이트 이상 비밀값>
set -a
source /etc/hazard-guard/dispenser.env
set +a

ros2 launch hazard_guard_simulation physical_patrol.launch.py \
  map:=/absolute/path/facility.yaml \
  use_person_safety:=true \
  enable_thermal_pipeline:=true \
  thermal_roi_config:=/absolute/path/facility_rois.json \
  use_dispenser:=true \
  enable_hazard_approval:=true \
  enable_physical_drop:=false
```

마지막 인자를 `true`로 바꾸는 것은 정지·사람 안전·BLE·전원·기구 검증을
모두 통과한 실물 시험에서만 허용한다. 일반 사용자가 `ros2 topic pub`으로
배출하는 흐름은 제공하지 않는다.
RGB-depth 픽셀 정합을 실물로 확인한 경우에만
`person_depth_registration_verified:=true`를 launch 인자에 추가한다.

## 위험 승인 상태 흐름

1. 열화상 분석기는 mission/cycle별 correlation ID가 일치하는 결과만 전달한다.
2. `warning` 또는 `critical`이면 Nav2 goal을 취소하고 `approval_required`로
   정지한다.
3. FastAPI가 관리자 확인을 기록하고 HMAC이 포함된 결정을 보낸다.
4. `resume`, `drop_then_resume`, `drop_then_monitor` 중 하나를 수행한다.
5. Action 결과와 디스펜서 SQLite 원장을 함께 확인하며, 결과 토픽만 믿지 않는다.
6. `jam_suspected`, 원장 불일치, 확인 시간 초과는 재배출하지 않고 현장 확인
   상태로 유지한다.
7. 감시 중 정상화돼도 자동 재개하지 않으며 관리자가 별도로 감시 완료를
   확인해야 한다.
8. `drop_then_monitor` 성공 후에는 로봇을 정지 상태로 유지한 채 열화상 방문을
   주기적으로 새로 수집한다. 정상 관측이 확인되어야
   `admin_release_required`로 전환되고, 그 뒤에만 관리자가 순찰을 재개할 수 있다.
9. 배출 성공 시 `map -> base_link` 위치와 후면 출구 오프셋으로 비콘 설치 위치를
   기록한다. TF를 얻지 못하면 열원 위치로 대체하지 않고 위치 미확인으로 남긴다.
   실물 출구 위치가 확정되면 `dispenser_rear_offset_m` launch 인자를
   `base_link` X축 기준 실측값으로 조정한다(후면은 음수).

## 런타임 의존성

### ROS 2 패키지

- ROS 2 Humble
- `rclpy`
- `hazard_guard_interfaces`
- `nav_msgs`
- `std_msgs`
- `ament_python`
- `launch`, `launch_ros`

### Python 및 시스템 의존성

- `pyserial`: Rosmaster USB 직렬 통신
- `Rosmaster_Lib` 3.3.9: PWM 서보 명령 생성
- `bleak`: 비콘 큐브 BLE 연결
- BlueZ 및 D-Bus: Linux BLE 백엔드

현재 Jetson에서는 다음 위치에서 발견된다.

```text
Rosmaster_Lib:
  /usr/local/lib/python3.10/dist-packages/Rosmaster_Lib-3.3.9-py3.10.egg
bleak:
  /home/jetson/.local/lib/python3.10/site-packages/bleak
serial:
  /home/jetson/.local/lib/python3.10/site-packages/serial
```

의존성 재현 시 주의 사항:

- `python3-serial`은 Ubuntu Jammy의 apt/rosdep으로 설치할 수 있다.
- `python3-bleak` rosdep 키는 Ubuntu Jammy에서 설치 항목이 제공되지 않는다.
- `Rosmaster_Lib`에는 현재 rosdep 규칙이 없다.
- 따라서 저장소 clone과 `rosdep install`만으로 하드웨어 환경이 완성되지
  않는다. 설치 방법과 검증된 버전을 별도 스크립트 또는 컨테이너/이미지로
  고정해야 한다.

## 알려진 문제와 위험

### 1. `/dev/myserial` 다중 소유

물리 M1 드라이버의 `Mcnamu_driver_M1`과 디스펜서 노드가 각각
`Rosmaster()`를 생성해 같은 `/dev/myserial`을 연다. 디스펜서 코드는 수신
스레드를 만들지 않지만, 두 프로세스의 송신이 조정되지 않는다. 주행 중
`cmd_vel` 전송과 서보 전송이 겹치는 부하 시험이 필요하다.

장기적으로는 다음 중 하나를 선택한다.

1. 하나의 하드웨어 드라이버가 Rosmaster 직렬 포트를 독점하고 주행과
   디스펜서 명령을 모두 처리한다.
2. Yahboom M1 드라이버에 PWM ID 1 디스펜서 인터페이스를 추가한다.
3. 별도 프로세스 접근을 유지한다면 직렬 패킷 무결성과 장애 복구를 실기로
   검증하고 그 제약을 명시한다.

### 2. 하드웨어 전송 실패 감지 불가

`Rosmaster_Lib.set_pwm_servo()`는 내부 예외를 출력한 뒤 삼킨다. 호출자는
시리얼 전송 실패를 알 수 없어 실제로 서보가 움직이지 않아도 성공 상태를
발행할 수 있다. 예외와 결과를 전달하는 하드웨어 어댑터가 필요하다.

### 3. BLE ARM 실패 시 배출 차단

연결된 큐브가 없거나 ARM 전송이 모두 실패하면
`rejected_no_confirmation`으로 종료하고 서보 배출을 시작하지 않는다. BLE 없는
서보 단독 시험은 운영 노드의 안전 경로를 우회하지 않고 별도 하드웨어 정비
도구에서 수행한다.

### 4. 운영 Action과 정비 문자열 토픽 분리

운영 배출은 `DispenseBeacon` Action으로 전환했다. 기존 문자열 토픽의 JSON
배출 요청은 HMAC 검증을 유지하지만 WebUI 직접 배출 API에서는 호출하지 않는다.
`angle:NN`, `home`, 설치 비콘 초기화는 정비 모드를 명시적으로 허용한 경우에만
사용할 수 있다.

### 5. 상태 메시지가 곧바로 덮임

`dropped` 또는 `jam_suspected`를 발행한 직후 `finally`에서 `ready`가
발행된다. 느린 구독자는 최종 결과를 놓칠 수 있다. 명시적인 상태 메시지,
요청 ID, 완료 결과 및 transient-local QoS 적용을 검토한다.

### 6. 실제 각도 피드백 없음

`current_angle`은 마지막으로 명령한 값일 뿐 실제 서보 위치가 아니다. 걸림,
전원 부족, 기구 충돌을 직접 감지하지 못한다. BLE 낙하 보고도 서보 복귀 성공을
보장하지 않는다.

### 7. 비콘 큐브 펌웨어 미포함

`cube_ble.py`가 참조하는 `BeaconCube.ino`는 Jetson 파일시스템과 이 저장소에
없다. 펌웨어 소스를 추가하거나 별도 저장소 URL, 버전, UUID 및 프로토콜을
문서화해야 한다.

### 8. 자동 시작 및 순찰 연동

`physical_patrol.launch.py`에서 선택 실행할 수 있고 mission manager가 위험
승인 및 결과 상태를 관리한다. 의도하지 않은 물리 동작을 피하기 위해 세 launch
옵션은 계속 기본 비활성화로 유지한다.

## 개발 체크리스트

### A. 반입 및 재현성

- [x] 기존 `dispenser_ws` 소스를 `hazard_guard_dispenser` 독립 패키지로 반입
- [x] 현재 의존성과 하드웨어 연결 기록
- [x] `package.xml`에 ROS 런타임 의존성 표현
- [ ] Jammy용 `bleak` 설치 방식을 고정하고 설치 검증 절차 추가
- [ ] `Rosmaster_Lib` 설치 출처, 라이선스, 버전과 체크섬 기록
- [ ] 저장소만으로 재현 가능한 하드웨어 설치 스크립트 또는 이미지 절차 작성
- [ ] 비콘 큐브 펌웨어 또는 외부 버전 링크 추가

### B. 코드 구조와 오류 처리

- [ ] ROS 노드, 배출 상태 머신, Rosmaster 하드웨어, BLE 계층 분리
- [ ] Rosmaster/직렬 실패가 호출자까지 전달되도록 어댑터 구현
- [ ] 노드 시작 시 장치 존재, 포트 열기, 서보 설정 검증
- [x] BLE 0/3 및 하드웨어 부재 fail-closed 정책 추가
- [x] `/odom` 선속도·각속도 기반 연속 정지 확인 가드 추가
- [x] 시작·종료 시 무조건 home으로 움직이는 동작 기본 비활성화
- [x] 배출 시작 후 예외 시 home 복귀 시도 및 결과 기록
- [ ] 취소 시 안전한 home 복귀 및 타임아웃 추가
- [ ] 파라미터 범위 검증 (`step_deg > 0`, 각도와 시간 제한)
- [ ] BLE 연결/재연결/종료 시 thread와 event loop 정리 검증
- [x] 상태 전이와 요청 ID를 포함하는 명시적 모델 추가

### C. ROS API

- [x] `hazard_guard_interfaces`에 `DispenseBeacon` Action 설계
- [x] 운영 문자열 `drop` 명령을 타입이 있는 Action으로 교체
- [x] 진행 상태: accepted, arming, dispensing, waiting, homing
- [x] ARM 전송 단계를 별도 `arming` 진행 상태로 발행
- [x] 결과 상태: succeeded, jam_suspected, hardware_error, canceled 등
- [x] 수동 각도 명령을 개발 모드에서만 허용
- [ ] 상태 QoS와 마지막 결과 보존 정책 정의

### D. 물리 로봇 통합

- [ ] Rosmaster 직렬 포트 단일 소유 구조 결정
- [ ] 주행 중 `cmd_vel`과 서보 패킷 동시 부하 시험
- [x] `physical_patrol.launch.py`에 `use_dispenser:=false` 선택 옵션 추가
- [x] 디스펜서 파라미터 YAML과 launch 파일 추가
- [ ] 시작 시 무조건 home으로 움직이는 현재 동작의 안전성 검토
- [x] 로봇 정지 확인 후에만 배출하도록 연동
- [x] 사람 안전 상태가 최신 CLEAR일 때만 승인 배출 허용
- [ ] 비상 정지·순찰 취소·프로세스 종료 시 동작 정의

### E. 임무 및 열화상 연동

- [x] warning/critical에서 관리자 승인 대기 정책 정의
- [x] correlation이 확인된 열화상 추세와 배출 결정 계층 연결
- [x] 같은 `detection_id`에 대한 중복 배출 방지
- [x] 낙하 확인된 큐브를 설치 상태로 영속화하고 재배출 후보에서 제외
- [x] mission manager 상태와 배출 결과 연결
- [x] 위험 탐지 자동 배출을 금지하고 관리자 승인만 허용

### F. 테스트

- [x] 하드웨어와 BLE 없이 검증 가능한 순수 정책 테스트 기반 마련
- [ ] 정상 배출 및 home 복귀 테스트
- [x] 동시·중복 `drop` 요청 멱등 처리 테스트
- [x] BLE ARM 실패 시 fail-closed 순수 정책 테스트
- [ ] 낙하 보고 타임아웃 및 걸림 상태 테스트
- [ ] 하드웨어 전송 예외 테스트
- [ ] 종료 및 취소 중 home 복귀 테스트
- [ ] 잘못된 각도·시간·step 파라미터 테스트
- [x] `use_dispenser` true/false launch 계약 테스트
- [ ] 실제 M1 정지 상태 단독 배출 시험
- [ ] 실제 M1 주행 명령과 디스펜서 동시 스트레스 시험
- [ ] 전원 재인가, USB 재연결, BLE 재연결 복구 시험

## 단계별 완료 기준

### 단계 1: 코드 보존

- 패키지가 저장소에서 추적된다.
- 기존 독립 작업공간과 동일한 소스가 빌드된다.
- 알려진 외부 의존성과 위험이 문서화된다.

### 단계 2: 수동 운용 가능

- 직렬 접근 방식이 결정되고 전송 실패가 결과에 반영된다.
- 타입이 있는 ROS API로 한 번의 배출 결과를 확정할 수 있다.
- BLE ARM과 서보 동작의 실패 정책이 테스트된다.
- 물리 launch에서 기본 비활성 상태로 선택 실행할 수 있다.

### 단계 3: 자동 임무 연동 가능

- 로봇 정지, 사람 안전, 위험 등급과 중복 방지 조건이 모두 적용된다.
- mission manager가 배출 성공·실패를 기록하고 실패 시 안전하게 임무를
  중단하거나 계속할 수 있다.
- 실제 주행 중 동시 부하와 장애 복구 시험을 통과한다.

## 변경 기록 원칙

체크리스트 항목을 완료할 때는 관련 커밋 또는 시험 결과를 항목 아래에 남긴다.
실기 파라미터를 변경할 경우 날짜, 하드웨어 구성, 성공/실패 결과를 함께
기록한다. 실패한 시험도 삭제하지 말고 재현 조건과 후속 조치를 남긴다.
