# Day 3 — SPH0645 마이크 기록 (Microphone log)

## 클럭 (d3-3) — 2026-11-10 ~ 11-13 ✅
- 32 kHz는 nRF54L15 I2S 정수 분주(MCK = 32 MHz/N, PS 8.11.5·Table 42)로 만들 수 없음 → 요청 33 333 Hz
- 레지스터: MCKFREQ=0x10841000, RATIO=64 → MCK 2.1333 MHz, LRCK 33 333.33 Hz, SCK 2.1333 MHz
- 2C53T 실측: BCLK 2.13 MHz, ≈3.76 Vpp (프로브 ×10 설정 일치 후), LRCL 33.33 kHz ✅
- decimation ÷4 → 8 333.33 Hz (`analysis/design_decim.py`, 63 taps, 0–2 kHz alias < −80 dB)
- PS Table 77: I2S에서 clock pin 필요 = SCK·MCK만 (LRCK 불필요). I2S20 = P1 전용

## 문제: 마이크 출력 없음 (d3-4) ❌
| 확인 | 결과 |
|------|------|
| 모듈 3V–모듈 GND | 3.328 V ✅ |
| 모듈 GND ↔ XIAO GND 도통 | ✅ |
| DOUT 헤더 ↔ XIAO D2 도통 | ✅ |
| DOUT 파형 | 논리 신호 없음, BCLK 크로스토크만 (≈0.6 Vpp) |
| SEL=GND / SEL=3V | 왼쪽·오른쪽 모두 0 |
| 헤더 6개 리플로 후 (11-13) | 여전히 0 |
| I2S 수신 경로 | 프레임 손실 0, CRC 0, I2S 오류 0 (데이터만 0) |

→ 판단: **모듈 불량 가능성 높음** (정전기·칩 불량). 교체품으로 재시험.
- 주의: 2C53T 프로브 스위치와 채널 배율(×1/×10)을 반드시 일치시킬 것 (불일치 시 1/10로 표시됨)

## 대안: XIAO 내장 PDM 마이크 (On-board PDM mic) — 2026-11-13 ✅
목적 (Purpose): SPH0645 교체품을 기다리는 동안 마이크 → UART → PC → WAV 전체 경로를 실제 소리로 검증,
이후 두 마이크 동시 녹음 비교 준비 (validate the full path; prepare a simultaneous comparison).

| 항목 | 값 |
|---|---|
| 핀 (pins) | PDM20 CLK P1.12, DIN P1.13, 전원 P0.01 (보드 DTS `pdm_imu_pwr`, regulator-boot-on) |
| 클럭 (clock) | PRESCALER 24, RATIO 80 → PDM CLK 1.3333 MHz, 캡처 16 666.67 Hz (레지스터로 확인) |
| 출력 (output) | FIR 31탭 ÷2 → **8 333.33 Hz** (I2S 경로와 같음 / same as I2S path), alias < −82 dB |
| 프레임 (frames) | MIC_PDM type 5 (int16 128개 / 15.36 ms), PDM_DIAG 0x7D (L/R 통계·레지스터) |
| 펌웨어 (firmware) | `apps/acq_fw/src/mic_pdm.c`, 기본은 I2S·PDM 동시 동작 (`-DMIC_I2S=0`, `-DMIC_PDM=0`으로 끔) |
| PC | `host/mic_test.py --source pdm` (기본값 / default) |

첫 측정 `mic_pdm_20261113_004112_quiet` (6 s, 실내 주변음 / room sound):
- 손실 0, CRC 오류 0, 수신 오류 0, 장치 송신 버림 0
- 범위 −610 ~ 779, DC ≈ 0, RMS 136.7 (−47.6 dBFS) — 300–850 Hz 고조파·시간에 따른 포락선 변화 → **실제 소리 수신 확인**
- 3.3 kHz 위 감쇠는 decimation 필터 차단 주파수 그대로 (roll-off matches the FIR cutoff)
- L·R 값이 거의 같음: 마이크가 반대 edge에서 DIN을 놓아도 선 전압이 유지되기 때문 → 채널 판정 불가, 출력은 L 사용
  (L ≈ R because the DIN line holds its level on the unused edge)

손뼉 시험 `mic_pdm_20261113_004243_clap` (10 s) ✅
- 손뼉 13번이 약 0.72초 간격의 뾰족한 펄스로 모두 보임 (13 claps, all visible)
- 최대 −14 497 (약 −7 dBFS) → 클리핑 없음, 손뼉 사이 바닥은 거의 0 (no clipping)
- 스펙트럼: 500–3 300 Hz 넓게 퍼짐(손뼉 특성), 3.3 kHz 위는 필터로 감쇠
- 첫 손뼉 뒤 기준선이 약 0.5초 동안 천천히 돌아옴 → 큰 충격 후 마이크·PDM 고역통과 회복으로 보임 (참고만)
- 손실·CRC·수신 오류·송신 버림 모두 0
