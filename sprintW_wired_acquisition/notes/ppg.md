# MAX30102 PPG — Day 2 (d2-1 ~ d2-7)

2026-11-16. 순서: Day 3(마이크) → 내장 IMU → Day 2(PPG) (Order: mic → on-board IMU → PPG).

## 하드웨어 (Hardware)
- i2c22 (`xiao_i2c`): SDA D4(P1.10), SCL D5(P1.11), 400 kHz — 보드 기본값, 모듈 풀업 사용
- INT D3(P1.07): `GPIO_PULL_UP | GPIO_ACTIVE_LOW`, 하강 에지 인터럽트
- overlay 노드 `max30102@57`, 앱 전용 바인딩 `dts/bindings/sprintw,max30102.yaml`
  (Zephyr `max30101` 드라이버는 쓰지 않음 / not using the Zephyr driver)

## 펌웨어 (Firmware) — `apps/acq_fw/src/max30102.c`
- 시작 시 i2c22 스캔(0x08–0x77, 1바이트 읽기) → PPG_DIAG에 결과 (d2-2)
- PART_ID 확인 → 리셋 → **INT Status 1 읽어 PWR_RDY 해제** (d2-5) → 설정
- 설정: SpO2 모드(Red + IR), 100 Hz, 411 µs(18 bit), ADC 4096 nA, 평균 없음, 롤오버 꺼짐,
  LED Red/IR 0x24(7.2 mA), INT = A_FULL(미읽음 17개)
- INT 에지 → 스레드: INT Status 1 읽기(INT 해제, PWR_RDY면 재초기화) → 포인터 → FIFO 일괄 읽기
  (계획의 워크큐 대신 IMU와 같은 전용 스레드 + 세마포어 / dedicated thread like the IMU)
- 300 ms 동안 INT가 없으면 시간 초과 경로가 같은 읽기로 복구
- 시각: 에지 = 미읽음 17개가 된 순간 → t_dev_us = 그 샘플 시각 (+ 더 쌓인 만큼 × 10 ms)
- FIFO 넘침(OVF_COUNTER)은 샘플 번호를 건너뛰어 PC가 손실로 봄 (rollover off → newest lost)
- 프레임
  - `PPG` (type 1): N × (red int32, ir int32), 보통 N = 17 (136 B, 약 170 ms) — 계획 1.3 형식
  - `PPG_DIAG` (0x7B, 40 B, 1초마다): `<BBBbBBBBB3sIIIIIIHH>` = version, part, rev, init_err,
    spo2_cfg, fifo_cfg, led_red, led_ir, scan_n, scan[3], samples, ovf_samples, i2c_errors,
    timeouts, tx_drop, reinits, batch_min, batch_max
- 빌드 옵션: `-DPPG=0` 끔, `-DPPG_LED_RED=0x.. -DPPG_LED_IR=0x..` (0.2 mA/LSB)
- 빌드: 경고 없음, FLASH 57.3 kB, RAM 33.8 % (마이크 2개 + IMU + PPG 동시)

## PC — `host/ppg_test.py`
- 실시간 1초 요약 → 끝나면 Part ID, 스캔, 실제 레이트, 포화, 손가락 구간 자동 선택
  (IR ≥ 10 000 인 가장 긴 구간 ± 1.5 s 제외), IR 대역 통과 0.7–3.5 Hz, 심박수(피크 간격·FFT)
- 저장: `data/raw/ppg_<시각>_<note>/` (session.bin, ppg_raw.csv), `figures/ppg_...png`
- `--replay <session.bin>` 으로 다시 분석 (d2-6의 `analysis/quick_ppg.py` 역할)

## 결과 (Results)
### d2-2·d2-3: 스캔·Part ID
- i2c22 스캔: **0x57 하나만** 응답, PART_ID **0x15**, REV_ID 3 ✅ (첫 시도 통과)

### 손가락 없음 (No finger), `ppg_20261116_105012_nofinger` (6 s)
- Red ≈ 784, IR ≈ 735 (만점의 0.3 %) — 주변광·반사 기준값
- 실제 레이트 **100.22 Hz**, 한 번에 17개, INT 시간 초과 0 ✅

### 손가락 (Finger), `ppg_20261116_105120_finger` (41 s, 화면·음성 안내)
| 항목 | 값 | 판정 |
|---|---|---|
| 실제 레이트 (기기 시계) | **100.216 Hz** (+0.22 %) | ✅ 자체 발진기 → t_dev_us 사용 |
| 손가락 구간 | 5.6–36.6 s (31 s) | |
| DC | Red 123 078, IR 139 592 (만점의 53 %) | ✅ 포화 0 %, 여유 충분 |
| IR 맥동 p-p | 약 590 counts, AC/DC 0.43 % | ✅ |
| 심박수 | 피크 간격 **74.2 bpm** (32 피크), FFT **75.0 bpm** | ✅ 두 방법 일치 |
| 손실·FIFO 넘침·CRC·I2C·INT 시간 초과·재초기화·송신 버림 | 모두 0 | ✅ |

- 파형: 박동마다 급히 내려갔다 천천히 오르는 전형적 PPG 모양(흡수 증가 = 값 감소).
- 처음 약 5초(5.6–10 s)는 맥동이 작음 → 손가락 압력·혈류가 안정되는 시간. 이후 점점 커짐.
  → 실험 프로토콜: **손가락을 올린 뒤 5초 안정화 후 기록 구간 시작** 권장.
- 그림: `figures/ppg_20261116_105120_finger.png`

### d2-7: LED 전류
7.2 mA에서 DC 53 %, 포화 없음, 맥동 뚜렷 → **기본값 0x24 유지**.
손가락 압력·피부색·온도에 따라 DC가 오르면 포화될 수 있으므로 80 % 넘으면 낮출 것.
`profiles/human.json` 초안 작성.

**Day 2 완료 기준 충족**: 손가락 맥파가 그림에 보임 (30 s 이상, 100.2 Hz), Part ID·설정값 기록.
