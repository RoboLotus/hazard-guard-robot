# 순찰 로봇 열 이상탐지 정책

> 적용 상태: Schema 3 프로토타입 · 2026-08-25 기준

## 공유용 요약

| 항목 | 현재 정책 |
| --- | --- |
| 검사 시점 | 로봇이 설비 웨이포인트에 정지한 검사 구간 |
| 공간 단위 | 설비 ROI 내부를 8 cm Voxel로 분할 |
| 대표 온도 | 각 Voxel에 들어온 유효 온도 Point의 P95 |
| 최소 품질 | Voxel당 5 Point, 설비 ROI 전체 40 Point 이상 |
| 권장 품질 | 설비 ROI 전체 100 Point 이상 |
| 즉시 위험 | 유효한 Voxel 하나라도 `P95 >= 설비별 Tcritical` |
| 국소 발열 | 한 개 또는 10개 미만의 작은 Voxel 영역도 판정 가능 |
| 사용하지 않는 조건 | 고온 픽셀 군집 수, 인접 고온 Voxel 수를 판정 차단 조건으로 사용하지 않음 |
| 시간 안정화 | 정지 검사 구간에서 같은 Voxel의 여러 관측값을 시간 중앙값으로 집계 |
| 완만한 이상 | 최근 3회 Trend와 정상 기준선 대비 Adaptive 편차가 함께 성립하면 Warning |
| 운영 결과 | Critical은 위험 이벤트로 전달하고 관리자 승인 흐름으로 연결 |

고온 픽셀 군집과 인접 Voxel 개수를 요구하지 않는 이유는 작은 열원과 국소 발열을 놓치지 않기 위해서임. 대신 단일 불량 픽셀에 과민해지는 문제는 `Voxel당 최소 5 Point`, `ROI 전체 최소 40 Point`, 정지 구간의 시간 중앙값으로 완화함. 이 값들은 실증 전 프로토타입 초기값이며 실제 거리·각도·열원 크기별 시험으로 조정해야 함.

## 1. 판정 개요

순찰 1회마다 설비 ROI 내부를 Voxel로 나누고, 각 Voxel의 `P95 온도`를 대표값으로 저장함. P95는 단일 Max보다 불량 픽셀과 순간 반사의 영향을 줄이면서 국소 발열을 보존하기 위한 값임. 설비 ROI 전체에서 P95 하나를 계산하는 방식이 아니라 Voxel별로 계산하므로 작은 열원이 전체 설비 평균에 묻히는 것을 줄임.

판정은 다음 세 신호를 조합함.

- `Critical`: 현재 절대온도가 설비의 위험온도 이상인지 확인함.
- `Trend`: 최근 순찰 3회 동안 온도차가 지속적으로 상승하는지 확인함.
- `Adaptive`: 현재 온도차가 정상 기준선보다 10℃ 이상 높은지 확인함.

`Critical`은 즉시 위험으로 판정함. 그 미만에서는 Trend와 Adaptive가 모두 발생하면 이상, 하나만 발생하면 이상 의심 및 재순찰, 둘 다 없으면 정상으로 판정함.

Schema 3에서는 `고온 픽셀 군집 9개`와 `인접 고온 Voxel 2개`를 판정에 사용하지 않음. 이 값은 과거 Schema 2의 단일 Max 확인 규칙에만 필요했으며, 현재 설정과 판정 결과에서는 노출하지 않음.

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
| 최소 총 상승 | 1.0℃ | 최근 3회에서 두 구간이 각각 0.5℃ 이상 상승해야 하므로 최소 총 상승과 일치시킨 프로토타입 초기값임. |
| Adaptive 편차 | `ΔTbase + 10℃` | NETA의 유사 부품 간 온도차 기준과 폐기물 허가문서의 인접부 대비 10℃ 상승 조사 기준을 반영함. [5][6] |
| 환경 기준 최소 Point | 40개 | 현재 P95 유효성에 사용하는 최소 ROI Point 수와 동일하게 설정한 품질 하한임. 통계적 보편값이 아니라 프로젝트 초기 운영값임. |
| 기준선 표본 수 | 정상 순찰 최소 8회 | 동일 조건의 단일 측정보다 반복 순찰 중앙값이 안정적이라는 판단에 따른 초기 타협값임. 현장 데이터 축적 후 확대가 필요함. [3] |
| 메모리 이력 | 최근 8회 | Trend 3회를 포함하고 재시작 전후 상태를 비교하기 위한 공학적 보관값임. 장기 이력은 JSONL에 별도 저장함. |

UCI 유압 시스템 데이터는 2,205개의 60초 운전 사이클을 포함함. 본 프로젝트에서는 동일 설비 상태의 안정 구간을 이용해 Trend 초기값을 검증했음. 실제 시설과 동일한 설비 데이터는 아니므로 현장 운용 후 오경보율을 재검증해야 함.

## 5. 설비별 절대 위험온도

| 설비 | `Tcritical` | 근거와 적용 조건 |
| --- | ---: | --- |
| 폐기물 적치부 | 60℃ | 폐기물 시설의 60~65℃ 조치 사례를 반영한 초기값임. 폐기물 성상과 함수율이 다르면 별도 시험값으로 교체해야 함. [6] |
| 파쇄기 모터 | 110℃ | WEG 모터 베어링 alarm 기준을 적용한 동종 장비 초기값임. 설치 모터 제조사 기준을 우선해야 함. [7] |
| 공정 펌프 | 105℃ | Flowserve 그리스 윤활 베어링 trip 기준을 적용함. 설치 펌프의 베어링·윤활방식이 다르면 해당 매뉴얼을 우선해야 함. [8] |
| 유압탱크 표면 | 82℃ | Parker의 유온 기준을 참고한 초기 상한임. 표면온도와 내부 유온의 관계를 현장에서 교정하기 전에는 자동정지 기준으로 사용하지 않음. [9] |

폐기물 49℃는 비교적 직접적인 자료 기반 값임. 나머지 세 값은 설치 모델이 확정되지 않아 적용한 동종 장비 기반 임시값임.

## 6. 설정 범위

| 설정 범위 | 설정 항목 |
| --- | --- |
| 프로젝트 공통 | Trend 3회, 회당 0.5℃, 총 1.0℃, Adaptive 10℃, Voxel 최소 5 Point, ROI 최소 40 Point, 메모리 8회 |
| 시설별 | 환경 기준 ROI 위치, 촬영 경로, 고정 촬영 자세 |
| 설비별 | 설비 ROI, 정상 부하 조건, `Tbase`, `ΔTbase`, `Tcritical` |

실제 시설 적용 시 동일 위치·거리·각도와 정상 부하에서 최소 8회 순찰하여 `Tbase`와 `ΔTbase`를 산출함. Mendeley 모터 열화상 데이터도 정상·고장 상태를 부하별로 구분하므로 기준선은 동일 부하 조건에서 수집하는 것이 적절함. [4]

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

- 작은 국소 열원은 한 개의 유효 Voxel만으로도 Critical이 될 수 있음. 이는 의도된 민감도이며, 반사·불량 픽셀에 의한 오경보는 실증에서 반드시 측정해야 함.
- 고온 픽셀 군집 수와 인접 고온 Voxel 수는 현재 판정 조건이 아님. 화면 표시나 사후 분석 지표로 계산할 수는 있으나 경보를 막지 않음.
- 기준 표면이 복사열이나 이동 물체의 영향을 받으면 보정값도 흔들릴 수 있으므로 ROI 선정과 중앙값 품질조건이 중요함.
- 환경 기준 Point가 40개 미만이면 보정값을 사용하지 않고 절대온도 기준선 방식으로 대체함.
- TMC160B의 절대 정확도는 `±5℃ 또는 ±5%`이므로 0.5℃는 동일 카메라·동일 위치의 반복 변화 감지용 값으로만 사용해야 함. [10]
- 실제 카메라 설치 후 기준 온도원과 현장 순찰 데이터로 `Voxel 5 Point·ROI 40 Point·0.5℃·1℃·10℃`의 미탐지율과 오경보율을 재검증해야 함.
- Critical 경보는 제조사 보호장치와 작업자 확인을 보완하는 기능이며 단독 자동정지 조건이 아님.

### 프로토타입 실증 권장 항목

| 시험 | 확인할 내용 |
| --- | --- |
| 작은 열원 | 1~9개 Voxel만 고온일 때 Critical 검출 여부 |
| 정상 설비 | 정상 부하에서 반복 측정했을 때 오경보 횟수 |
| 반사 표면 | 금속·유광 표면에서 순간 고온점이 지속되는지 |
| 거리·각도 | 예상 순찰 거리와 정면·사선 촬영에서 Point 수가 품질 하한을 넘는지 |
| 정지 시간 | 웨이포인트 검사 시간별 유효 프레임 수와 판정 재현성 |
| 경계 열원 | `Tcritical` 부근에서 반복 측정한 판정 흔들림 |

프로토타입 단계에서는 현재 정책으로 진행 가능함. 다만 실제 운영 기준으로 확정하기 전에는 작은 실제 열원에 대한 Recall과 정상 설비의 오경보율을 함께 측정해야 하며, 민감도를 낮춰야 할 경우 군집 조건을 다시 넣기보다 먼저 최소 Point, 정지 시간, 측정 거리와 설비별 임계값을 조정하는 것을 우선함.

본 정책은 열 전이 자체를 물리적으로 해석하는 모델이 아니라, 지도에 등록된 설비 ROI의 반복 열화상 관측으로 이상 후보를 선별하는 프로토타입 정책임.

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
