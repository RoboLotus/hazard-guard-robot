# YOLO11n 사람 탐지 환경 설정 및 기록

이 문서는 Hazard Guard의 사람 탐지 환경을 개발 PC와 Jetson에서 동일하게 재현하기 위한 기준이다. 대상은 **Ultralytics YOLO11n detection**, **COCO pretrained `yolo11n.pt`**, **`person` 클래스 ID 0**이다. 초기 단계에서는 별도 학습 없이 사람 탐지와 거리 추정의 정확성을 확인한다.

## 1. 채택 기준

| 항목 | 기준 |
|---|---|
| 모델 | YOLO11n detection |
| 초기 가중치 | COCO pretrained `yolo11n.pt` |
| 사용 클래스 | `person` (`class_id=0`) |
| 기본 입력 크기 | 640×640, 성능 측정 후 조정 |
| 정확성 검증 백엔드 | PyTorch (`.pt`) |
| Jetson 운영 목표 | Jetson에서 직접 생성한 TensorRT FP16 (`.engine`) |
| ROS | ROS 2 Humble, 시스템 설치 `rclpy`/`cv_bridge` 재사용 |

Ultralytics 버전은 팀원이 실제로 탐지에 성공한 **정확한 버전**을 최우선으로 고정한다. 해당 정보가 확인되지 않은 신규 환경에서는 `ultralytics==8.4.118`을 검증 후보로 사용할 수 있지만, 후보 버전을 검증 없이 운영 기준으로 확정하지 않는다.

개발 순서는 다음과 같다.

1. PyTorch `.pt` 모델로 이미지 전처리, `person` 필터, bounding box 및 confidence가 올바른지 확인한다.
2. ROS 2 카메라 입력과 Depth 거리 추정, Nav2 연동을 검증한다.
3. 동일 Jetson에서 TensorRT FP16 engine을 생성하고 PyTorch 결과와 비교한다.
4. 속도·지연시간·GPU 메모리와 탐지 결과 차이를 기록한 뒤 운영 백엔드를 선택한다.

TensorRT engine은 CUDA, TensorRT, JetPack과 대상 GPU에 종속될 수 있다. x86 PC나 다른 Jetson에서 생성한 engine을 배포하지 않는다.

## 2. 지원 환경 매트릭스

| 용도 | x86_64 개발/시뮬레이션 | Jetson 실물 |
|---|---|---|
| 운영체제 | 프로젝트 개발 환경 | JetPack에 포함된 Ubuntu |
| ROS | ROS 2 Humble | ROS 2 Humble |
| 모델 | `yolo11n.pt` | 먼저 `yolo11n.pt`, 이후 FP16 `.engine` |
| PyTorch | 호스트 CUDA 또는 CPU 호환 빌드 | JetPack과 호환되는 NVIDIA Jetson용 aarch64 빌드 |
| TensorRT | 필수 아님 | JetPack 제공 버전 사용 |
| 목적 | 기능 테스트, Gazebo/rosbag 재생 | RGB-D 입력, 성능·안전 통합 검증 |
| 성능 수치 사용 | 참고값 | 운영 판단의 기준값 |

x86에서 성공한 것은 ROS 인터페이스와 탐지 로직의 검증 결과다. Jetson 실시간 성능 또는 바이너리 호환성을 보장하지 않는다.

## 3. 설치 전 환경 기록

아래 스크립트는 Python 표준 라이브러리만 사용하며 **조회만 수행**한다. 패키지를 설치하거나 시스템 설정 및 파일을 변경하지 않는다. 명령이나 Python 모듈이 없어도 나머지 항목을 계속 수집한다.

```bash
cd ~/hazard-guard-robot
python3 tools/check_yolo_environment.py
python3 tools/check_yolo_environment.py --json > yolo-environment.json
```

모델 또는 engine의 SHA-256도 함께 확인할 수 있다. JSON 리다이렉션 파일은 필요 시 작업 기록 위치에 보관하고 비밀정보 포함 여부를 확인한 뒤 공유한다.

```bash
python3 tools/check_yolo_environment.py \
  --model runtime/models/yolo11n.pt \
  --model runtime/models/yolo11n_fp16.engine
```

필수 기록 항목은 다음과 같다.

- 하드웨어 아키텍처(`aarch64`)와 Jetson 모델
- Ubuntu, kernel, JetPack, L4T
- Python 실행 파일 및 버전, 활성 virtual environment
- ROS 배포판(Humble) 및 `ros2 doctor --report`
- CUDA, cuDNN, TensorRT
- `torch`, `torchvision`, `ultralytics`, OpenCV, NumPy의 버전과 로드 경로
- `torch.version.cuda`, `torch.cuda.is_available()`, GPU 이름
- 모델 파일명, 크기, SHA-256

환경 변경 전과 변경 후에 각각 결과를 남기면 의존성 회귀를 찾기 쉽다.

## 4. Python virtual environment

ROS 2 Humble은 apt로 설치된 Python 패키지와 연동된다. 시스템 Python에 YOLO 의존성을 직접 덮어쓰지 않고, ROS 패키지를 볼 수 있는 전용 venv를 사용한다.

```bash
cd ~/hazard-guard-robot
python3 -m venv --system-site-packages .venv-yolo
source .venv-yolo/bin/activate
python -m pip install --upgrade pip
```

`--system-site-packages`는 시스템의 `rclpy`, `cv_bridge`, ROS 메시지를 venv에서도 사용하기 위한 선택이다. 활성화 후 다음을 먼저 확인한다.

```bash
python -c "import rclpy, cv_bridge; print('ROS Python imports: OK')"
python tools/check_yolo_environment.py
```

venv를 사용해도 pip가 시스템 패키지와 ABI가 다른 NumPy/OpenCV를 선택하면 `cv_bridge`가 실패할 수 있다. `numpy`, `opencv-python`을 임의로 업그레이드하지 말고, 설치 전후 버전과 로드 경로를 비교한다.

## 5. Jetson PyTorch 설치 원칙

> **Jetson에서 일반 PyPI의 `torch`로 기존 PyTorch를 덮어쓰지 않는다.**

Jetson의 PyTorch/torchvision은 설치된 JetPack(L4T/CUDA)에 맞는 NVIDIA 제공 aarch64 패키지 또는 해당 JetPack이 공식 지원하는 방식으로 설치한다. 다음 절차를 따른다.

1. `tools/check_yolo_environment.py --json` 결과로 JetPack/L4T와 기존 torch를 기록한다.
2. NVIDIA의 해당 JetPack용 PyTorch 호환 표에서 정확한 wheel과 torchvision 조합을 확인한다.
3. 기존 Jetson 이미지에 제조사 제공 torch가 있으면 먼저 CUDA 동작을 검증한다.
4. 변경이 필요하면 복구 가능한 OS 이미지/환경에서 수행하고, 설치 명령과 wheel URL 또는 파일 SHA-256을 기록한다.
5. 설치 직후 `torch.cuda.is_available()`과 간단한 GPU tensor 연산을 검증한다.

```bash
python - <<'PY'
import torch
print("torch:", torch.__version__)
print("torch CUDA build:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit("GPU-enabled Jetson PyTorch verification failed")
x = torch.ones((1024, 1024), device="cuda")
print("device:", x.device, "sum:", x.sum().item())
PY
```

PyTorch CUDA 검증이 끝나기 전에는 Ultralytics나 TensorRT 문제로 판단하지 않는다. 기반 torch가 CPU 전용이거나 JetPack과 맞지 않으면 상위 계층의 결과도 신뢰할 수 없다.

## 6. Ultralytics 설치와 버전 고정

먼저 팀원에게 다음 정보를 받아 기록한다.

- `ultralytics`, `torch`, `torchvision`, Python 버전
- 사용한 정확한 모델 파일과 SHA-256
- 입력 크기, confidence, device, 실행 명령
- JetPack/L4T 또는 개발 PC CUDA 버전

설치는 pip가 Jetson용 torch를 교체하지 않도록 실행 계획을 먼저 검토해야 한다. 특히 일반적인 `pip install ultralytics`가 의존성을 해석하면서 `torch`/`torchvision`, NumPy, OpenCV를 바꿀 수 있다. 운영 환경에서는 호환 버전을 명시한 constraints 또는 검증된 lock 파일을 사용한다.

팀원 버전 미확인 시의 탐색 후보:

```text
ultralytics==8.4.118
model=yolo11n.pt
task=detect
classes=[0]
```

후보를 설치하기 전 `python -m pip install --dry-run ...`으로 변경될 패키지를 확인하고, Jetson용 `torch`가 교체 대상으로 표시되면 중단한다. 실제 프로젝트 설치 명령은 팀원이 사용한 버전 및 JetPack 호환 조합을 확인한 후 별도 requirements/constraints 파일로 확정한다.

설치 후 최소 검증:

```bash
python tools/check_yolo_environment.py
python - <<'PY'
import torch
from ultralytics import YOLO

assert torch.cuda.is_available(), "Jetson CUDA is unavailable"
model = YOLO("runtime/models/yolo11n.pt")
print("model task:", model.task)
PY
```

## 7. 모델 파일 및 manifest

가중치와 생성 산출물은 소스 저장소에 커밋하지 않는다. 저장 위치는 다음 구조를 사용한다.

```text
runtime/models/
├── yolo11n.pt
├── yolo11n_fp16.engine
└── manifest.json
```

저장소 `.gitignore`에는 최종 통합 시 다음 패턴을 반영한다. 이 문서 작성 작업에서는 공용 `.gitignore`를 변경하지 않는다.

```gitignore
runtime/models/*.pt
runtime/models/*.onnx
runtime/models/*.engine
```

`manifest.json`에는 바이너리를 재현하고 검증하는 데 필요한 메타데이터만 둔다. manifest에 로컬 절대 경로나 자격 증명을 넣지 않는다.

```json
{
  "model": "yolo11n.pt",
  "sha256": "<64 hex characters>",
  "source": "Ultralytics YOLO11n COCO pretrained",
  "task": "detect",
  "classes": {"0": "person"},
  "imgsz": 640,
  "ultralytics": "<exact version>",
  "torch": "<exact version>",
  "jetpack": "<exact version>",
  "l4t": "<exact version>",
  "tensorrt": "<exact version or null>",
  "precision": "pt-fp32 or engine-fp16",
  "built_on": "<Jetson model or null>"
}
```

모델 교체 시 파일명만 같게 덮어쓰지 않는다. SHA-256과 manifest를 함께 갱신하고 PyTorch/TensorRT 비교 결과를 다시 남긴다.

## 8. TensorRT FP16 전환

기능 정확성이 확인된 후 실제 목표 Jetson에서 engine을 생성한다. export 명령과 옵션은 고정된 Ultralytics 버전에서 확인해 기록하며, 일반적인 목표 설정은 다음과 같다.

```text
source: yolo11n.pt
format: engine
precision: FP16
imgsz: 640 (또는 벤치마크로 결정한 값)
batch: 1
device: 0
```

engine 생성 후 동일한 대표 이미지/rosbag에서 다음을 비교한다.

- 탐지된 사람 수와 bbox 좌표
- confidence 차이
- 누락 및 오검출 사례
- warm-up 제외 latency와 처리율
- GPU 메모리, 시스템 메모리, 온도 및 throttling

TensorRT 결과가 빠르더라도 안전 임계값 근처에서 PyTorch와 의미 있는 탐지 차이가 있으면 즉시 운영에 사용하지 않는다.

## 9. 벤치마크 기록 양식

모든 수치는 Nav2, 카메라 드라이버 등 실제 동시 실행 조건을 함께 표시한다.

| 필드 | 기록값 |
|---|---|
| 일시 / 담당자 / Git commit | |
| Jetson 모델 / 전원 모드 / `nvpmodel` | |
| JetPack / L4T / Ubuntu | |
| Python / ROS 2 | |
| CUDA / cuDNN / TensorRT | |
| torch / torchvision / ultralytics | |
| OpenCV / NumPy | |
| 모델 파일 / SHA-256 / backend / precision | |
| 입력 토픽 / 해상도 / FPS | |
| YOLO `imgsz` / confidence / class filter | |
| warm-up 프레임 / 측정 프레임 | |
| 평균, p50, p95, p99 추론 지연시간(ms) | |
| end-to-end p95 지연시간(ms) | |
| 처리 FPS / 입력 프레임 drop 수 | |
| CPU / GPU 사용률 | |
| RAM / GPU 메모리 최대값 | |
| 온도 / thermal throttling 여부 | |
| Nav2+SLAM 동시 실행 여부 | |
| 사람 거리/조도/가림 조건 | |
| 누락·오검출 및 비고 | |

초기 성능 목표는 사람 탐지 8~10 FPS 이상, 추론 p95 200 ms 이하를 출발점으로 사용하되 안전 요구사항과 실제 하드웨어 측정 결과로 확정한다. 640 입력이 부족하면 FP16 engine, 추론 주기 제한, 512/416 입력 순으로 비교한다.

## 10. 라이선스 및 배포 기록

Ultralytics 코드와 모델 사용에는 배포 방식에 따른 라이선스 조건이 적용된다. 현재 사용되는 Ultralytics 라이선스(예: AGPL-3.0 또는 별도 Enterprise 조건), 모델 출처, 버전, 수정 및 배포 형태를 릴리스 전에 담당자가 검토해야 한다. 이 문서는 법률 자문을 대체하지 않는다.

또한 PyTorch, torchvision, TensorRT, CUDA, cuDNN 및 모델에 포함된 제3자 구성 요소의 라이선스와 고지 의무를 `THIRD_PARTY_NOTICES.md` 정책에 맞춰 확인한다. 외부 배포 전에는 다음을 완료한다.

- 실제 사용 버전과 다운로드 출처 기록
- 모델 및 wheel/engine SHA-256 기록
- 소스 제공 또는 고지 의무 검토
- 상용/외부 배포 방식에 맞는 Ultralytics 라이선스 확인
- 컨테이너 또는 Jetson 이미지에 포함되는 NVIDIA 구성 요소의 재배포 조건 확인

## 11. 장애 확인 순서

1. `tools/check_yolo_environment.py`로 Python 실행 파일과 모듈 로드 경로를 확인한다.
2. JetPack/L4T와 torch wheel 호환성을 확인한다.
3. `torch.cuda.is_available()` 및 GPU tensor 연산을 확인한다.
4. ROS를 source한 같은 shell/venv에서 `rclpy`, `cv_bridge`, OpenCV, NumPy import를 확인한다.
5. 고정된 `yolo11n.pt` SHA-256으로 단일 이미지 추론을 확인한다.
6. 그 다음 ROS 토픽, RGB-D 정합, TensorRT engine 순으로 범위를 넓힌다.

환경 문제를 해결하기 위해 검증 없이 전체 패키지를 최신 버전으로 올리지 않는다. 한 번에 한 계층만 변경하고, 매 변경 전후 환경 JSON과 벤치마크를 남긴다.

## 12. Jetson Codex 인수인계 절차

이 절은 실물 Jetson에서 Codex가 저장소를 처음 열었을 때 따라야 하는 작업 지시서다.
목표는 **기존 네이티브 ROS/JetPack 환경을 보존하면서 YOLO11n TensorRT engine을
준비하고, 사람 탐지 노드가 이를 읽을 수 있는 상태까지만 만드는 것**이다. 실제 바퀴를
움직이는 검증은 사용자가 안전요원과 함께 별도로 승인한 뒤 수행한다.

### 12.1 작업 경계

- Docker 이미지를 새로 만들지 않는다. Jetson의 기존 네이티브 환경을 사용한다.
- JetPack, CUDA, cuDNN, TensorRT를 업그레이드하거나 교체하지 않는다.
- 일반 PyPI `torch`/`torchvision`으로 NVIDIA aarch64 빌드를 덮어쓰지 않는다.
- 제조사 공장 이미지에 설치된 환경과 `/home/jetson/ultralytics` 예제를 먼저 조사한다.
- 모델과 engine은 Git에 추가하지 않는다. `runtime/models/`는 의도적으로 무시된다.
- 절대 경로를 소스 코드나 YAML에 커밋하지 않는다. launch 인자로 전달한다.
- 실행하지 않은 검증을 성공으로 기록하지 않는다.
- 환경 변경, commit, push는 사용자의 명시적인 승인 없이 수행하지 않는다.

### 12.2 저장소 및 환경 확인

저장소 루트에서 다음을 실행한다. 실제 저장소 위치는 고정하지 말고 `pwd` 결과를
사용한다.

```bash
cd <hazard-guard-robot 저장소>
export HAZARD_GUARD_ROBOT_ROOT="$PWD"
mkdir -p runtime/debug runtime/models

git branch --show-current
git status --short
uname -m
cat /etc/nv_tegra_release 2>/dev/null || true
python3 tools/check_yolo_environment.py --json \
  > runtime/debug/yolo-jetson-before.json
```

환경 조사 결과에서 최소한 다음을 확인하고 사용자에게 보고한다.

- `aarch64`, Jetson 모델, JetPack/L4T 버전
- Python, ROS 2, CUDA, TensorRT 버전
- `torch`, `torchvision`, `ultralytics`, NumPy, OpenCV 버전과 로드 경로
- `torch.cuda.is_available()` 결과
- 기존 `yolo11n.pt`, `.onnx`, `.engine` 위치와 SHA-256
- 제조사 HP60C launch 및 RGB/Depth 토픽 존재 여부

기존 모델은 먼저 다음과 같이 찾는다. 전체 루트 파일시스템을 무차별 탐색하지 않는다.

```bash
find "$HOME/ultralytics" "$HOME/Rosmaster" "$HAZARD_GUARD_ROBOT_ROOT" \
  -maxdepth 5 -type f \
  \( -name 'yolo11n.pt' -o -name 'yolo11n.onnx' -o -name '*.engine' \) \
  2>/dev/null
```

### 12.3 PyTorch 기준 모델 준비

기존 공식 `yolo11n.pt`가 있다면 새로 내려받지 말고 SHA-256을 기록한 뒤 로컬 런타임
디렉터리에 복사한다. 같은 이름의 파일이 이미 있으면 덮어쓰지 말고 두 파일의
SHA-256을 비교한다.

```bash
mkdir -p "$HAZARD_GUARD_ROBOT_ROOT/runtime/models"
sha256sum <발견한-yolo11n.pt>
cp -n <발견한-yolo11n.pt> \
  "$HAZARD_GUARD_ROBOT_ROOT/runtime/models/yolo11n.pt"
```

기존 모델이 없고 네트워크 다운로드가 필요하면 출처와 버전을 사용자에게 보고한 뒤
진행한다. 모델을 찾기 위해 패키지를 무작정 재설치하지 않는다.

### 12.4 TensorRT FP16 engine 생성

먼저 PyTorch 모델이 GPU에서 단일 추론되는지 확인한다. 그 다음 **실제 운용할 같은
Jetson에서** engine을 생성한다. Ultralytics와 JetPack 조합이 확인된 동일 Python을
사용한다.

```bash
cd "$HAZARD_GUARD_ROBOT_ROOT"
export YOLO_AUTOINSTALL=false
python3 - <<'PY'
from pathlib import Path
from ultralytics import YOLO

root = Path.cwd()
source = root / "runtime/models/yolo11n.pt"
if not source.is_file():
    raise SystemExit(f"missing source model: {source}")

model = YOLO(str(source))
exported = Path(model.export(
    format="engine",
    half=True,
    imgsz=640,
    batch=1,
    device=0,
))
target = root / "runtime/models/yolo11n_fp16.engine"
if target.exists() and target.resolve() != exported.resolve():
    raise SystemExit(f"refusing to overwrite existing engine: {target}")
if target.resolve() != exported.resolve():
    exported.replace(target)
print(target)
PY
```

Ultralytics가 변환 중 ONNX를 자동 생성할 수 있지만 운영 launch는 최종 `.engine`을
직접 읽는다. `YOLO_AUTOINSTALL=false`는 변환 과정이 환경을 몰래 변경하지 못하게 한다.
누락된 export 의존성이 표시되면 설치하지 말고 정확한 패키지와 요구 버전을 사용자에게
먼저 보고한다. export가 실패해도 의존성 전체를 업그레이드하지 않는다.

생성 직후 engine 로드와 단일 GPU 추론을 확인한다.

```bash
python3 - <<'PY'
from pathlib import Path
import numpy as np
from ultralytics import YOLO

engine = Path("runtime/models/yolo11n_fp16.engine").resolve()
if not engine.is_file():
    raise SystemExit(f"missing engine: {engine}")
result = YOLO(str(engine))(
    np.zeros((480, 640, 3), dtype=np.uint8),
    imgsz=640,
    device=0,
    verbose=False,
)
print(f"engine smoke test: OK, results={len(result)}")
PY

python3 tools/check_yolo_environment.py \
  --model runtime/models/yolo11n.pt \
  --model runtime/models/yolo11n_fp16.engine
```

### 12.5 ROS 빌드와 탐지 단독 확인

```bash
source /opt/ros/humble/setup.bash
cd "$HAZARD_GUARD_ROBOT_ROOT"
rosdep check --from-paths src --ignore-src
colcon build --symlink-install --packages-up-to \
  hazard_guard_person_detection \
  hazard_guard_safety_supervisor \
  hazard_guard_mission_manager \
  hazard_guard_simulation
source install/setup.bash
```

`rosdep check`가 누락 의존성을 보고하면 목록을 사용자에게 먼저 제시한다. 설치 승인을
받은 경우에만 `rosdep install --from-paths src --ignore-src -r -y`를 실행한다. 기존
제조사 패키지를 교체하려 한다면 중단하고 보고한다. HP60C 드라이버를 한 터미널에서
실행한 뒤 토픽을 확인한다.

```bash
ros2 launch ascamera hp60c.launch.py

# 다른 터미널
source /opt/ros/humble/setup.bash
source "$HAZARD_GUARD_ROBOT_ROOT/install/setup.bash"
ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image
ros2 topic hz /ascamera_hp60c/camera_publisher/depth0/image_raw
```

모터 및 Nav2를 시작하지 않고 탐지 노드만 실행할 수 있다. RGB-Depth 픽셀 정합을
아직 확인하지 않았으므로 처음에는 반드시 `false`를 유지한다.

```bash
ros2 launch hazard_guard_person_detection person_detection.launch.py \
  rgb_topic:=/ascamera_hp60c/camera_publisher/rgb0/image \
  depth_topic:=/ascamera_hp60c/camera_publisher/depth0/image_raw \
  model_path:="$HAZARD_GUARD_ROBOT_ROOT/runtime/models/yolo11n_fp16.engine" \
  device:=0 \
  image_size:=640 \
  inference_rate_hz:=10.0 \
  confidence:=0.4 \
  depth_registration_verified:=false
```

확인 토픽:

```bash
ros2 topic hz /hazard_guard/person/observations
ros2 topic echo /hazard_guard/person/observations --once
ros2 run rqt_image_view rqt_image_view \
  /hazard_guard/person/annotated_image
```

이 단계에서 바운딩박스와 inference 시간이 보이면 engine 이식은 완료된 것이다. 거리는
의도적으로 무효이며 안전 상태를 활성화해서는 안 된다.

### 12.6 RGB-Depth 정합 승인 후 실물 launch

`person_depth_registration_verified:=true`는 다음을 실물로 확인한 뒤에만 사용한다.

1. RGB와 Depth 해상도가 동일하다.
2. 두 영상의 같은 픽셀이 같은 물체를 나타낸다.
3. 움직이는 물체에서도 timestamp 차이가 허용 범위 안이다.
4. 사람 bbox 중앙 영역의 Depth가 실제 줄자 거리와 일치한다.

정합되지 않으면 `true`로 우회하지 않는다. HP60C aligned-depth 토픽을 사용하거나
`CameraInfo` 기반 좌표 투영 개발이 필요하다고 보고한다.

승인 후 전체 순찰 launch는 다음과 같이 실행한다.

```bash
ros2 launch hazard_guard_simulation physical_patrol.launch.py \
  map:="$HAZARD_GUARD_ROBOT_ROOT/runtime/maps/<map-name>.yaml" \
  use_person_safety:=true \
  start_person_camera:=true \
  person_model_path:="$HAZARD_GUARD_ROBOT_ROOT/runtime/models/yolo11n_fp16.engine" \
  person_device:=0 \
  person_image_size:=640 \
  person_inference_rate_hz:=10.0 \
  person_confidence:=0.4 \
  person_depth_registration_verified:=true
```

HP60C 드라이버가 이미 실행 중이면 `start_person_camera:=false`로 중복 실행을 막는다.

### 12.7 Codex 완료 보고 형식

Codex는 작업 종료 시 다음 형식으로 사용자에게 보고한다.

```text
## 환경
- Git branch/commit:
- Jetson/JetPack/L4T:
- Python/ROS:
- CUDA/TensorRT:
- torch/torchvision/ultralytics:

## 모델
- PT 경로/SHA-256:
- engine 경로/SHA-256:
- export 옵션:
- engine 단일 추론 결과:

## ROS 확인
- HP60C RGB/Depth 토픽과 Hz:
- annotated image:
- observations topic:
- RGB-Depth 정합 승인 여부:

## 실행한 검증
- 실제로 성공한 명령:
- 실패한 명령과 원문 오류:

## 미해결 및 실물 검증 필요사항
- 안전 기능 활성화를 막는 조건:
- 다음 담당자가 수행할 항목:
```
