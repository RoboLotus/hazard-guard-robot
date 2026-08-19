# HazardGuard 디스펜서 개발 노트

이 문서는 Jetson의 기존 독립 작업공간인
`/home/jetson/dispenser_ws`에서 가져온 디스펜서 코드를 계속 개발하기 위한
현황, 의존성, 알려진 문제와 작업 목록을 기록한다.

## 현재 상태

- 기준 브랜치: `dev` (`751916c`)
- 최초 반입일: 2026-08-19
- 원본 패키지: `/home/jetson/dispenser_ws/src/hazard_guard_dispenser`
- 하드웨어 연결: Jetson USB `/dev/myserial` -> Rosmaster 확장보드 S1 -> MG946R
- 기본 서보 설정: ID 1, 대기 0도, 배출 30도
- BLE 프로토콜: 비콘 큐브에 ARM/CANCEL을 전송하고 낙하 보고를 수신

현재 코드는 기존 실험 코드를 보존한 초기 반입본이다. 패키지 단독 빌드와
수동 배출 이력은 있지만, 물리 순찰 launch 및 mission manager에는 아직
연결하지 않았다. 운영 기능으로 활성화하기 전에 아래 필수 작업을 완료해야
한다.

## ROS 인터페이스

현재 구현은 다음 문자열 토픽을 사용한다.

| 방향 | 이름 | 형식 | 값 |
| --- | --- | --- | --- |
| 구독 | `/hazard_guard/dispenser/command` | `std_msgs/String` | `drop`, `home`, `status`, `angle:NN` |
| 발행 | `/hazard_guard/dispenser/status` | `std_msgs/String` | `ready`, `busy`, `dropped`, `jam_suspected`, `error:...` |

실행 예:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hazard_guard_dispenser dispenser_node --ros-args -p use_cube_ble:=false
```

수동 배출 예:

```bash
ros2 topic pub --once /hazard_guard/dispenser/command \
  std_msgs/msg/String "{data: 'drop'}"
```

## 런타임 의존성

### ROS 2 패키지

- ROS 2 Humble
- `rclpy`
- `std_msgs`
- `ament_python`

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

### 3. BLE ARM 실패 후에도 배출 진행

현재는 연결된 큐브가 없거나 ARM 전송이 모두 실패해도 서보 배출을 계속한다.
운영 모드에서는 `require_cube_arm=true`를 기본으로 두고 ARM 실패 시 배출을
중단하는 fail-closed 정책이 필요하다. BLE 없이 기구만 시험할 때만 명시적으로
우회해야 한다.

### 4. 문자열 명령과 결과 상관관계 부재

자유 형식 문자열 토픽은 요청별 응답, 중복 요청 식별, 취소와 진행 피드백을
제공하지 않는다. 수 초 걸리는 배출 동작에는 전용 Action이 적합하며, 최소
구현으로는 `Trigger` 서비스를 사용할 수 있다. 수동 `angle:NN`은 운영
환경에서 기본 차단해야 한다.

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

### 8. 자동 시작 및 순찰 연동 없음

현재 디스펜서 노드는 `physical_patrol.launch.py`, mission manager 또는
systemd에 연결되지 않았다. 의도하지 않은 물리 동작을 피하기 위해 검증 전까지
launch 기본값은 비활성화로 유지한다.

## 개발 체크리스트

### A. 반입 및 재현성

- [x] 기존 `dispenser_ws` 소스를 `hazard_guard_dispenser` 독립 패키지로 반입
- [x] 현재 의존성과 하드웨어 연결 기록
- [ ] `package.xml`에 실제 런타임 의존성 표현
- [ ] Jammy용 `bleak` 설치 방식을 고정하고 설치 검증 절차 추가
- [ ] `Rosmaster_Lib` 설치 출처, 라이선스, 버전과 체크섬 기록
- [ ] 저장소만으로 재현 가능한 하드웨어 설치 스크립트 또는 이미지 절차 작성
- [ ] 비콘 큐브 펌웨어 또는 외부 버전 링크 추가

### B. 코드 구조와 오류 처리

- [ ] ROS 노드, 배출 상태 머신, Rosmaster 하드웨어, BLE 계층 분리
- [ ] Rosmaster/직렬 실패가 호출자까지 전달되도록 어댑터 구현
- [ ] 노드 시작 시 장치 존재, 포트 열기, 서보 설정 검증
- [ ] `require_cube_arm` fail-closed 정책 추가
- [ ] 종료·예외·취소 시 안전한 home 복귀 및 타임아웃 추가
- [ ] 파라미터 범위 검증 (`step_deg > 0`, 각도와 시간 제한)
- [ ] BLE 연결/재연결/종료 시 thread와 event loop 정리 검증
- [ ] 상태 전이와 요청 ID를 포함하는 명시적 모델 추가

### C. ROS API

- [ ] `hazard_guard_interfaces`에 배출 Action 또는 Service 설계
- [ ] 문자열 `drop` 명령을 타입이 있는 API로 교체
- [ ] 진행 상태: arming, dispensing, waiting, homing
- [ ] 결과 상태: succeeded, arm_failed, jam_suspected, hardware_error, canceled
- [ ] 수동 각도 명령을 개발 모드에서만 허용
- [ ] 상태 QoS와 마지막 결과 보존 정책 정의

### D. 물리 로봇 통합

- [ ] Rosmaster 직렬 포트 단일 소유 구조 결정
- [ ] 주행 중 `cmd_vel`과 서보 패킷 동시 부하 시험
- [ ] `physical_patrol.launch.py`에 `use_dispenser:=false` 선택 옵션 추가
- [ ] 디스펜서 파라미터 YAML과 launch 파일 추가
- [ ] 시작 시 무조건 home으로 움직이는 현재 동작의 안전성 검토
- [ ] 로봇 정지 확인 후에만 배출하도록 연동
- [ ] 사람 안전 상태가 CLEAR일 때만 자동 배출 허용
- [ ] 비상 정지·순찰 취소·프로세스 종료 시 동작 정의

### E. 임무 및 열화상 연동

- [ ] 어떤 위험 등급에서 큐브를 배출할지 정책 정의
- [ ] `/hazard_guard/thermal_detections`와 배출 결정 계층 연결
- [ ] 같은 `detection_id`에 대한 중복 배출 방지
- [ ] 큐브 재고와 누적 배출 수 관리
- [ ] mission manager 상태와 배출 결과 연결
- [ ] 자동 배출은 수동 API의 물리 검증 완료 후 별도 변경으로 구현

### F. 테스트

- [ ] 하드웨어와 BLE 가짜 구현을 사용한 단위 테스트 기반 마련
- [ ] 정상 배출 및 home 복귀 테스트
- [ ] 동시 `drop` 요청 거부 테스트
- [ ] BLE ARM 실패 시 fail-closed 테스트
- [ ] 낙하 보고 타임아웃 및 걸림 상태 테스트
- [ ] 하드웨어 전송 예외 테스트
- [ ] 종료 및 취소 중 home 복귀 테스트
- [ ] 잘못된 각도·시간·step 파라미터 테스트
- [ ] `use_dispenser` true/false launch 테스트
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
