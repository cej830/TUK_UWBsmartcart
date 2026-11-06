# UWB 거리 측정 실험 데이터

[프로젝트 소개로 돌아가기](../README.md)

앵커별 정적 거리 측정, 이동 중 필터 응답, 두 앵커를 이용한 중심 거리 추정을 기록한 CSV 파일이다. 모든 파일은 첫 행에 컬럼명이 있으며, 아래 행 수는 헤더를 제외한 값이다.

## 데이터 목록

| 파일 | 구분 | 행 수 |
| --- | --- | ---: |
| [uwb_logA_cal.csv](uwb_logA_cal.csv) | 앵커 A 정적 거리 측정 통계 | 51 |
| [uwb_logB_cal.csv](uwb_logB_cal.csv) | 앵커 B 정적 거리 측정 통계 | 51 |
| [uwb_dynamic_A_fast.csv](uwb_dynamic_A_fast.csv) | 앵커 A 동적 측정 (`fast`) | 300 |
| [uwb_dynamic_A_run2.csv](uwb_dynamic_A_run2.csv) | 앵커 A 동적 측정 (`run2`) | 200 |
| [uwb_dynamic_B_fast.csv](uwb_dynamic_B_fast.csv) | 앵커 B 동적 측정 (`fast`) | 200 |
| [uwb_dynamic_B_slow.csv](uwb_dynamic_B_slow.csv) | 앵커 B 동적 측정 (`slow`) | 300 |
| [uwb_trilateration_r_test.csv](uwb_trilateration_r_test.csv) | 중심 거리 추정 평가 | 8 |

## 정적 거리 데이터

대상: `uwb_logA_cal.csv`, `uwb_logB_cal.csv`.

| 컬럼 | 설명 |
| --- | --- |
| `timestamp` | 측정 기록 시각 |
| `tag_position` | 기준 태그 위치·거리 라벨 |
| `raw_avg`, `raw_std` | 원시 거리의 평균 및 표준편차 |
| `ma_avg`, `ma_std` | 이동평균 처리 거리의 평균 및 표준편차 |
| `spike_avg`, `spike_std` | 스파이크 대응 처리 거리의 평균 및 표준편차 |
| `spike_count` | 스파이크 감지 횟수 |

거리 값의 단위는 m이다.

## 동적 거리 데이터

대상: `uwb_dynamic_*.csv`.

| 컬럼 | 설명 |
| --- | --- |
| `time_s` | 시간(s) |
| `raw` | 원시 거리 |
| `ma_only` | 이동평균 처리 거리 |
| `ma_spike` | 스파이크 대응을 포함한 이동평균 처리 거리 |
| `spike_flag` | 스파이크 감지 여부 플래그 |
| `dA_raw`, `dA_ma_only`, `dA_ma_spike` | 각 처리 경로에 대응하는 보정 거리 값으로 해석되는 컬럼 |

B 파일에도 `dA_*` 컬럼명을 사용한다. 앵커는 파일명의 A·B로 구분하고, `fast`, `slow`, `run2`는 동적 실험 구분으로 사용한다.

## 중심 거리 추정 데이터

대상: `uwb_trilateration_r_test.csv`.

| 컬럼 | 설명 |
| --- | --- |
| `timestamp`, `label` | 측정 시각과 실험 조건 라벨 |
| `r_true` | 기준 중심 거리(m) |
| `dA_mean`, `dB_mean` | 앵커 A·B 거리 평균(m) |
| `r_est_mean` | 추정 중심 거리 평균(m) |
| `r_error` | 추정 오차: `r_est_mean - r_true`(m) |
| `dA_std`, `dB_std`, `r_std` | 앵커별 거리 및 중심 거리의 표준편차(m) |
| `samples` | 해당 조건의 측정 샘플 수 |

## 활용 안내

- 정적 데이터로 거리 구간별 편차와 필터 적용 후 변동성을 비교할 수 있다.
- 동적 데이터로 이동평균의 응답 지연 및 급격한 거리 변화에 대한 필터 동작을 살펴볼 수 있다.
- 중심 거리 데이터로 기준 거리와 추정값의 차이를 확인할 수 있다.

보정 계수와 필터 조건은 [중앙 제어 코드](../controller/UWB_smartcart_main.py)에 정리되어 있다. 실험 배치와 결과 해석은 [졸업논문](../docs/README.md)에서 다룬다.
