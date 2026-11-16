# 내장 IMU (On-board IMU) — XIAO nRF54L15 Sense

2026-11-16. 순서 변경: 외부 센서(PPG)보다 내장 센서를 먼저 동작시킴
(Order change: on-board sensors before the external PPG).
PPG 동잡음(motion artifact) 판정용 기준 신호로 사용 예정.

## 하드웨어 (Hardware)
- i2c30: SDA P0.03, SCL P0.04, 400 kHz, 주소 0x6A, INT1 P0.02 (GPIOTE30)
- 전원: PDM 마이크와 공유 (P0.01 `pdm_imu_pwr`, regulator-boot-on)
- **칩 = LSM6DS3TR-C (WHO_AM_I 0x6A)**. NCS v3.2.1 보드 DTS는 `st,lsm6dso`(0x6C)로
  잘못 적혀 있어 Zephyr `lsm6dso` 드라이버를 쓰면 칩 ID 검사에서 실패함.
  Board DTS declares LSM6DSO, but the chip is LSM6DS3TR-C → the Zephyr lsm6dso driver would fail.
- 보드 DTS의 `accel-pm = XL_ULP_MODE`도 쓰지 않음 (소프트 리셋으로 초기화).

## 펌웨어 (Firmware) — `apps/acq_fw/src/imu.c`
- Zephyr 센서 드라이버 대신 I2C 레지스터 직접 접근 (CONFIG_I2C만, CONFIG_SENSOR 안 씀)
- 설정: SW reset → CTRL3_C = BDU|IF_INC, CTRL1_XL = 104 Hz ±4 g, CTRL2_G = 104 Hz ±500 dps,
  INT1_CTRL = DRDY_XL
- INT1 상승 에지 → 0x1E부터 16 B 한 번에 읽기 (STATUS, TEMP, G, XL).
  에지를 놓쳐도 30 ms 시간 초과 경로가 STATUS를 읽어 복구.
- 프레임
  - `IMU` (type 6): 8샘플 × [ax, ay, az, gx, gy, gz] int16 = 96 B. t_dev_us = 마지막 샘플 시각,
    first_idx = 첫 샘플 번호
  - `IMU_DIAG` (0x7C, 40 B, 약 1초마다): `<BBbBHHHhIIIIIII>` = version, WHO_AM_I, init_err,
    reserved, ODR, FS_g, FS_dps, temp_raw, samples, gaps, i2c_errors, timeouts, tx_drop,
    dt_min_us, dt_max_us
- 감도: 0.122 mg/LSB (±4 g), 17.5 mdps/LSB (±500 dps), 온도 25 + raw/256 °C
- 빌드 옵션 `-DIMU=0` 으로 끔. 빌드: 경고 없음, FLASH 55.8 kB, RAM 32.6 %

## 측정 1 — 정지 (At rest), `imu_20261116_103533_still` (6 s, 책상 위)
| 항목 | 값 | 판정 |
|---|---|---|
| 칩 | LSM6DS3TR-C (0x6A) | ✅ |
| 실제 ODR (기기 시계) | **100.18 Hz** (설정 104 Hz 대비 −3.7 %) | ⚠️ 아래 참고 |
| 샘플 간격 | 9 952–10 016 µs, 1.5 주기 초과 0회 | ✅ 지터 ±0.3 % |
| 가속도 평균 x/y/z | −0.013 / −0.015 / +1.013 g | ✅ z축이 위 |
| \|a\| | 1.013 g, 표준편차 2.5 mg | ✅ 오프셋 1.3 % (영점 오차 범위) |
| 자이로 영점 x/y/z | +0.69 / −1.50 / +0.39 dps | ✅ 데이터시트 범위 내 |
| 자이로 잡음 (std) | 0.15–0.19 dps | ✅ |
| 온도 | 24.7 °C | ✅ |
| 손실·CRC·I2C·시간 초과·송신 버림 | 모두 0 | ✅ |

**ODR −3.7 %**: IMU는 자체 내부 발진기로 샘플링하므로 공칭 104 Hz가 아님.
기기 시계(GRTC)는 정확하므로, **IMU 샘플 시각은 공칭 ODR이 아니라 프레임의 t_dev_us로
계산해야 함** (분석·통합 시 필수). 개체·온도에 따라 달라질 수 있어 매 세션 측정값을 사용.
The IMU runs on its own oscillator (100.18 Hz measured); timestamp IMU samples from t_dev_us.

## 측정 2 — 움직임 (Motion), `imu_20261116_104012_motion` (26 s)
화면 안내(브라우저 큐 페이지)에 따라 기울이기 → 기울이기 → 흔들기 → 내려놓기.
움직임 구간 9.8–20.7 s, 앞뒤 정지 구간은 측정 1과 같은 값으로 복귀.

| 동작 | 반응 | 판정 |
|---|---|---|
| 기울이기 1 (x축 회전) | ay 0 → +0.63 g, gx 최대 +38 dps, 약 40° | ✅ |
| 기울이기 2 (y축 회전) | ax 0 → +0.68 g, gy 최대 −65 dps, 약 −33° | ✅ |
| 흔들기 | ax ±0.4 g 진동(약 2.5 Hz), \|a\| 최대 1.4 g | ✅ |
| 내려놓기 | \|a\| 순간 1.82 g 충격 후 정지값 복귀 | ✅ |

**가속도 기울기 vs 자이로 적분 교차 확인 (cross-check)**:
중력으로 구한 각도와 자이로(영점 보정)를 적분한 각도가 움직이는 동안 **약 3° 이내로 일치**
(예: 13.7 s roll 39.5° vs 39.7°, 16.8 s pitch −32.2° vs −34.6°).
→ 두 센서의 감도(0.122 mg/LSB, 17.5 mdps/LSB)·축 방향·부호가 서로 맞음.
Accel-derived tilt and integrated gyro agree within ~3° → sensitivities and axis signs are consistent.
끝난 뒤 자이로 적분이 2–3° 남는 것은 영점 보정 잔차의 누적으로 정상.

손실·CRC·I2C·INT 시간 초과·송신 버림 모두 0. **내장 IMU 검증 완료.**
