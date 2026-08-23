# 순찰 로봇 열 이상탐지 로직 및 기준값 근거

## 1. 판정 개요

순찰 1회마다 설비 ROI 내부를 Voxel로 나누고, 각 Voxel의 `P95 온도`를 대표값으로 저장함. P95는 단일 Max보다 불량 픽셀과 순간 반사의 영향을 줄이면서 국소 발열을 보존하기 위한 값임.

판정은 다음 세 신호를 조합함.

- `Critical`: 현재 절대온도가 설비의 위험온도 이상인지 확인함.
- `Trend`: 최근 순찰 3회 동안 온도차가 지속적으로 상승하는지 확인함.
- `Adaptive`: 현재 온도차가 정상 기준선보다 10℃ 이상 높은지 확인함.

`Critical`은 즉시 위험으로 판정함. 그 미만에서는 Trend와 Adaptive가 모두 발생하면 이상, 하나만 발생하면 이상 의심 및 재순찰, 둘 다 없으면 정상으로 판정함.

## 2. 사용하는 값

| 기호 | 의미 |
| --- | --- |
| `Tn` | 현재 Voxel P95 절대온도 |
| `Tenv` | 같은 순찰에서 환경 기준 ROI로 계산한 중앙값 |
| `ΔTn` | `Tn - Tenv` |
| `Tbase` | 정상 순찰에서 얻은 Voxel P95 절대온도 중앙값 |
| `ΔTbase` | 정상 순찰에서 얻은 `P95 - 환경 기준값`의 중앙값 |
| `Tcritical` | 설비 종류와 제조사 자료로 정한 절대 위험온도 |

환경 기준 ROI는 직접적인 열원의 영향을 적게 받는 바닥·벽 등의 고정 영역으로 지정함. 중앙값을 사용하여 일시적인 고온점과 가림의 영향을 줄임.

## 3. 판정 흐름

```text
현재 Voxel P95 측정
        │
        ├─ Tn ≥ Tcritical
        │       └─ Critical: 즉시 위험
        │
        └─ Tn < Tcritical
                ├─ Trend와 Adaptive 모두 참 → Warning: 이상
                ├─ 하나만 참                 → Watch: 이상 의심·재순찰
                └─ 둘 다 거짓                → Normal: 정상
```

Trend와 Adaptive에는 환경 기준 데이터가 유효할 때 `ΔT`를 사용함. 환경 기준 데이터가 부족하면 해당 순찰은 기존 `Tn`과 `Tbase` 비교 방식으로 자동 대체함. Critical은 항상 원래의 절대온도로 판정하여 보정 때문에 즉시 위험이 가려지지 않도록 함.

## 4. 공통 기준값과 근거

| 설정값 | 적용값 | 근거 |
| --- | ---: | --- |
| 대표온도 | Voxel P95 | 단일 Max의 노이즈 영향을 낮추면서 국소 고온을 유지하기 위한 선택임. AI-Hub 열화상 데이터도 촬영 조건과 보정 변수를 함께 관리함. [1][2] |
| Trend 관측 수 | 최근 3회 | 2회보다 일시 변화와 지속 상승을 구분하기 유리함. UCI 안정 상태 구간 검증에서 결합 규칙의 오검출이 없었음. [3] |
| 회당 최소 상승 | 0.5℃ | UCI 안정 상태 인접 사이클의 P95 변화 95백분위가 약 0.09~0.15℃였음. 단독 판정하지 않고 3회 조건과 결합함. [3] |
| 최소 총 상승 | 2.0℃ | `3회·회당 0.5℃·총 2℃` 조건을 UCI 안정 상태 구간에 적용했을 때 Trend 검출이 0건이었음. [3] |
| Adaptive 편차 | `ΔTbase + 10℃` | NETA의 유사 부품 간 온도차 기준과 폐기물 허가문서의 인접부 대비 10℃ 상승 조사 기준을 반영함. [5][6] |
| 환경 기준 최소 Point | 40개 | 현재 P95 유효성에 사용하는 최소 ROI Point 수와 동일하게 설정한 품질 하한임. 통계적 보편값이 아니라 프로젝트 초기 운영값임. |
| 기준선 표본 수 | 정상 순찰 최소 10회 | 동일 조건의 단일 측정보다 반복 순찰 중앙값이 안정적이라는 판단에 따른 초기 타협값임. 현장 데이터 축적 후 확대가 필요함. [3] |
| 메모리 이력 | 최근 8회 | Trend 3회를 포함하고 재시작 전후 상태를 비교하기 위한 공학적 보관값임. 장기 이력은 JSONL에 별도 저장함. |

UCI 유압 시스템 데이터는 2,205개의 60초 운전 사이클을 포함함. 본 프로젝트에서는 동일 설비 상태의 안정 구간을 이용해 Trend 초기값을 검증했음. 실제 시설과 동일한 설비 데이터는 아니므로 현장 운용 후 오경보율을 재검증해야 함.

## 5. 설비별 절대 위험온도

| 설비 | `Tcritical` | 근거와 적용 조건 |
| --- | ---: | --- |
| 폐기물 적치부 | 49℃ | 목재칩 시험 임계 59℃에서 10℃ 안전여유를 적용한 조치 기준임. 폐기물 성상과 함수율이 다르면 별도 시험값으로 교체해야 함. [6] |
| 파쇄기 모터 | 110℃ | WEG 모터 베어링 alarm 기준을 적용한 동종 장비 초기값임. 설치 모터 제조사 기준을 우선해야 함. [7] |
| 공정 펌프 | 105℃ | Flowserve 그리스 윤활 베어링 trip 기준을 적용함. 설치 펌프의 베어링·윤활방식이 다르면 해당 매뉴얼을 우선해야 함. [8] |
| 유압탱크 표면 | 82℃ | Parker의 유온 기준을 참고한 초기 상한임. 표면온도와 내부 유온의 관계를 현장에서 교정하기 전에는 자동정지 기준으로 사용하지 않음. [9] |

폐기물 49℃는 비교적 직접적인 자료 기반 값임. 나머지 세 값은 설치 모델이 확정되지 않아 적용한 동종 장비 기반 임시값임.

## 6. 설정 범위

| 설정 범위 | 설정 항목 |
| --- | --- |
| 프로젝트 공통 | Trend 3회, 회당 0.5℃, 총 2.0℃, Adaptive 10℃, 최소 Point 40개, 메모리 8회 |
| 시설별 | 환경 기준 ROI 위치, 촬영 경로, 고정 촬영 자세 |
| 설비별 | 설비 ROI, 정상 부하 조건, `Tbase`, `ΔTbase`, `Tcritical` |

실제 시설 적용 시 동일 위치·거리·각도와 정상 부하에서 최소 10회 순찰하여 `Tbase`와 `ΔTbase`를 산출함. Mendeley 모터 열화상 데이터도 정상·고장 상태를 부하별로 구분하므로 기준선은 동일 부하 조건에서 수집하는 것이 적절함. [4]

환경 기준 ROI는 열원·직사 반사·이동 물체의 영향을 적게 받으며 순찰마다 반복 관측되는 고정 표면을 선택함. 현재 구현은 한 순찰의 유효 환경 ROI Point를 합쳐 중앙값 하나를 사용함. 시설 내부 구역별 열 조건 차이가 크면 향후 구역별 기준값으로 분리하는 것이 적절함.

시뮬레이션과 실제 로봇은 동일한 기준선 수집기를 사용함. 활성 기준선이 없으면 Critical은 첫 순찰부터, Trend는 유효 순찰 3회부터 동작하며 Adaptive는 비활성 상태로 유지됨. 모든 설비에서 유효한 정상 순찰 10회가 모이면 기준선 파일을 자동 생성하고 분석 노드에 즉시 적용함. 기준선이 완성된 순찰은 기존 조건으로 판정하고 다음 순찰부터 Adaptive 판정에 새 기준선을 사용함.

다음 조건의 순찰만 기준선 표본으로 인정함.

- 설비 P95 품질조건을 만족함.
- 환경 기준 Point가 40개 이상임.
- 해당 순찰에서 Critical 또는 Trend가 발생하지 않음.
- 해당 설비와 Voxel이 실제로 관측됨.

Critical이 발생하면 현재 표본을 제외하고 수집을 일시중지하되 이전 정상 표본은 보존함. Trend가 발생하면 Trend 판단에 사용된 최근 3회 구간만 제외하고 그 이전 정상 표본은 보존함. 이후 정상 순찰이 3회 연속 확인되면 자동으로 수집을 재개하며, 재확인 중인 표본은 기준선 계산에 포함하지 않음. 제외된 표본과 사유는 전용 JSON 파일에 최대 100건까지 남겨 재시작 후에도 확인할 수 있음.

전체 수집 표본 삭제는 자동으로 수행하지 않으며 `/hazard_guard/thermal/reset_baseline_collection` 서비스를 명시적으로 호출한 경우에만 수행함. 진행 상태는 다음 ROS 서비스로 확인할 수 있음.

```bash
ros2 service call /hazard_guard/thermal/baseline_status std_srvs/srv/Trigger '{}'
ros2 service call /hazard_guard/thermal/reset_baseline_collection std_srvs/srv/Trigger '{}'
```

시뮬레이션과 실제 로봇은 절차와 판정 규칙은 같지만 이력·후보·승인 파일을 서로 다른 경로에 저장함. 따라서 한쪽에서 수집한 기준선이 다른 쪽에 적용되지 않음.

## 7. 제한사항 및 운영 원칙

- 기준 표면이 복사열이나 이동 물체의 영향을 받으면 보정값도 흔들릴 수 있으므로 ROI 선정과 중앙값 품질조건이 중요함.
- 환경 기준 Point가 40개 미만이면 보정값을 사용하지 않고 절대온도 기준선 방식으로 대체함.
- TMC160B의 절대 정확도는 `±5℃ 또는 ±5%`이므로 0.5℃는 동일 카메라·동일 위치의 반복 변화 감지용 값으로만 사용해야 함. [10]
- 실제 카메라 설치 후 기준 온도원과 현장 순찰 데이터로 `0.5℃·2℃·10℃`의 오경보율을 재검증해야 함.
- Critical 경보는 제조사 보호장치와 작업자 확인을 보완하는 기능이며 단독 자동정지 조건이 아님.

이번 변경은 기존 환경 ROI 저장값을 판정에 연결한 것임. 열 전이와 설비·주변 영역 온도 수집·저장 구조는 변경하지 않았음.

## 8. 출처

1. [AI-Hub, 산업시설 열화상 CCTV 데이터](https://www.aihub.or.kr/aihubdata/data/view.do?aihubDataSe=realm&dataSetSn=514)
2. [AI-Hub, 산업단지 열화상 데이터](https://www.aihub.or.kr/aihubdata/data/view.do?currMenu=115&dataSetSn=235&topMenu=100)
3. [UCI, Condition Monitoring of Hydraulic Systems](https://archive.ics.uci.edu/dataset/447/condition+monitoring+of+hydraulic+systems)
4. [Mendeley Data, Induction Motor-Thermal Images](https://data.mendeley.com/datasets/pymtdhfzbj/1)
5. [ANSI/NETA MTS-2007, Table 100.18](https://www.vendorportal.ecms.va.gov/FBODocumentServer/DocumentServer.aspx?DocumentId=3076056&FileName=VA246-17-Q-0018-004.pdf)
6. [UK Environment Agency, EPR/KP3031YD/A001 결정문](https://assets.publishing.service.gov.uk/government/uploads/system/uploads/attachment_data/file/713552/EPR_KP3031YD_A001_Decision_Document.pdf)
7. [WEG, Motor Installation, Operation and Maintenance Manual](https://static.weg.net/medias/downloadcenter/h48/h95/WEG-WMO-smoke-extraction-motors-installation-operation-and-maintenance-manual-50026367-manual-english-web.pdf)
8. [Flowserve, Pump Installation, Operation and Maintenance Manual](https://www.flowserve.com/sites/default/files/dam/documents/71569294_EN_AQ.pdf)
9. [Parker, PGP/PGM 600 Series Fluid Recommendations](https://www.parker.com/content/dam/Parker-com/Literature/Pump---Motor-Division/Catalogs/HY09-600_US_OOS.pdf)
10. [ThermoEye, TMC160B 사양](https://thermoeye.co.kr/solutions/thermalcamera)
