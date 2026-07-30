# Third-Party Notices

이 저장소의 ROS 2 소스 코드, launch, 설정, 프로젝트 작성 URDF/SDF에는 루트
`LICENSE`의 Apache License 2.0이 적용됩니다. 아래 외부 자산에는 원저작자의
별도 권리가 적용되며, 프로젝트의 Apache-2.0 라이선스로 재라이선스되지 않습니다.

## Yahboom ROSMASTER-M1

- 원제작자: Yahboom Technology
- 참고 저장소: <https://github.com/YahboomTechnology/ROSMASTER-M1>
- 제품 자료: <https://category.yahboom.net/products/rosmaster-m1>
- 사용 범위: ROSMASTER-M1 치수·구조 참고 및 차체·카메라·LiDAR·휠 visual mesh

다음 mesh는 제조사 공개 자료를 기반으로 내부 개발·검증에 사용합니다.

```text
src/hazard_guard_simulation/meshes/yahboom_m1/base_link.stl
src/hazard_guard_simulation/meshes/yahboom_m1/camera_link.stl
src/hazard_guard_simulation/meshes/yahboom_m1/laser_link.stl
src/hazard_guard_simulation/meshes/yahboom_m1/le_be_link.stl
src/hazard_guard_simulation/meshes/yahboom_m1/le_fr_link.stl
src/hazard_guard_simulation/meshes/yahboom_m1/ri_be_link.stl
src/hazard_guard_simulation/meshes/yahboom_m1/ri_fr_link.stl
```

외부 공개 또는 상업 배포 전에는 Yahboom 원자료의 최신 이용 조건과 재배포
허용 범위를 다시 확인해야 합니다.

## Provisional rear dispenser

- 원제작자: RoboLotus 팀원
- 원본: <https://sketchfab.com/3d-models/dispenser-e528afd6dffc4302811993ddef7cc919>
- 사용 범위: 후면 디스펜서의 시뮬레이션 visual mesh
- 상태: 팀 내부 사용 승인을 받은 임시 모델이며, 실제 설계 확정 전 변경 가능

팀 내부 저장소에는 변환된 `dispenser_*.stl` 파일을 포함할 수 있습니다. 원본
USDZ, glTF, BIN과 텍스처는 제조사·팀원 원본 보관 디렉터리에만 유지합니다.
외부 공개 시에는 원제작자 표기와 공개 범위를 팀원과 다시 확인해야 합니다.

## Project-authored simulation data

충돌 형상, 관성 근사, 센서 장착 위치, 열화상 센서 설정, Fortress plugin 구성,
SLAM/Nav2 설정과 시설 월드는 HazardGuard 시뮬레이션을 위해 작성·조정했습니다.
이 항목들은 외부 visual mesh의 권리까지 포함하거나 대체하지 않습니다.
