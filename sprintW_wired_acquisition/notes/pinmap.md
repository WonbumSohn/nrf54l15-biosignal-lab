# XIAO nRF54L15 핀맵 · 배선 표 (Pin map & wiring table)

> d1-1 결과 (Result of step d1-1) — 2026-11-10 작성, 보드 파일 기준 / based on NCS v3.2.1 board files
>
> 근거 (Sources):
> - `/opt/nordic/ncs/v3.2.1/zephyr/boards/seeed/xiao_nrf54l15/seeed_xiao_connector.dtsi` (D0–D10 → Px.yy)
> - 같은 폴더 `xiao_nrf54l15-pinctrl.dtsi`, `xiao_nrf54l15_common.dtsi`, `xiao_nrf54l15_nrf54l15_cpuapp.dts`
> - `zephyr/dts/vendor/nordic/nrf54l_05_10_15.dtsi` (포트별 GPIOTE 연결 / GPIOTE per port)
> - nRF54L15 Product Specification, Pin assignments (QFN48 Table 4, CSP47 Table 6) — clock pin 열 / "Clock pin" column

---

## 1. 헤더 핀 → nRF 핀 (Header pin → nRF pin)

| XIAO | nRF | 포트 GPIOTE (인터럽트) | Clock pin | 보드 기본 용도 (Board default) |
|------|-----|-------------------------|:---------:|-------------------------------|
| D0 | P1.04 | GPIOTE20 ✅ | ✅ | — |
| D1 | P1.05 | GPIOTE20 ✅ | — | — |
| D2 | P1.06 | GPIOTE20 ✅ | — | — |
| D3 | P1.07 | GPIOTE20 ✅ | — | — |
| D4 | P1.10 | GPIOTE20 ✅ | — | `xiao_i2c` = i2c22 **SDA** |
| D5 | P1.11 | GPIOTE20 ✅ | ✅ | `xiao_i2c` = i2c22 **SCL** |
| D6 | P2.08 | ❌ 없음 | — | `xiao_serial` = uart21 TX |
| D7 | P2.07 | ❌ 없음 | — | uart21 RX |
| D8 | P2.01 | ❌ 없음 | ✅ (SPIM SCK) | `xiao_spi` = spi00 SCK |
| D9 | P2.04 | ❌ 없음 | — | spi00 MISO |
| D10 | P2.02 | ❌ 없음 | — | spi00 MOSI |

## 2. 헤더에 없는 · 이미 쓰이는 핀 (Not on header / already used)

| nRF | 용도 (Use) | 메모 |
|-----|-----------|------|
| P1.09 / P1.08 | uart20 TX / RX → **SAMD11 → USB** (`zephyr,console`) | **Gate 0에서 측정할 경로**. P1.08은 clock pin이지만 이미 사용 중 |
| P1.12 / P1.13 | 내장 PDM 마이크 CLK / DIN (pdm20) | 내장 마이크 — 이번에는 안 씀 |
| P0.03 / P0.04 | i2c30 SCL / SDA → 내장 IMU LSM6DSO (0x6A) | MAX30102(0x57)와 다른 버스 |
| P0.01 | IMU·PDM 전원 (gpio-hog HIGH) | |
| P0.02 | IMU INT | |
| P0.00 | **USR 버튼** (내부 풀업, active-low) | GPIOTE30 ✅ — 인터럽트 가능 |
| P2.00 | LED0 | |
| P2.03 / P2.05 | RF 스위치 전원 / 제어 | |
| P1.15 | VBAT 측정 전원 (S3 배터리 주의 — `XIAO_Flash_Guide.md` 6절) | |

## 3. 제약 (Constraints) — 핀 결정의 근거

1. **P2에는 GPIOTE가 없음** → P2 핀(D6–D10)은 **인터럽트 불가** (폴링만). `nrf54l_05_10_15.dtsi`에서 gpio0→gpiote30, gpio1→gpiote20, gpio2는 `gpiote-instance` 없음.
   P2 has no GPIOTE, so D6–D10 can't raise interrupts (polling only).
2. **TWIM20–22와 I2S20은 P1 핀** (TWIM30은 P0). P2에는 TWIM 없음 (Nordic DevZone 답변).
   TWIM20–22 and I2S20 use P1 pins; there is no TWIM on P2.
3. **Clock pin 필요**: TWIM **SCL**, I2S **SCK**·MCK, SPIM SCK, PDM CLK. 데이터 선은 clock pin **가까이**.
   헤더에서 쓸 수 있는 P1 clock pin은 **D0(P1.04)와 D5(P1.11) 두 개뿐**.
   SCL, I2S SCK (and MCK) need clock pins; only D0 and D5 are free P1 clock pins on the header.
4. I2S **LRCK**가 clock pin이어야 하는지는 표 추출이 애매함 → **Day 3 위험 항목** (아래 5절).

## 4. 결정 (Decision)

| 기능 (Function) | XIAO | nRF | 비고 (Notes) |
|-----------------|------|-----|-------------|
| I2C SDA (MAX30102) | D4 | P1.10 | 보드 기본 `xiao_i2c` 그대로 — overlay 필요 없음. 모듈 풀업 사용 (외부 저항 없음) |
| I2C SCL (MAX30102) | D5 | P1.11 | ✅ clock pin |
| MAX30102 INT | D3 | P1.07 | 인터럽트 가능(GPIOTE20). `GPIO_PULL_UP \| GPIO_ACTIVE_LOW` (d2-5) |
| I2S SCK = BCLK (SPH0645) | D0 | P1.04 | ✅ clock pin |
| I2S LRCK = LRCL/WS (SPH0645) | D1 | P1.05 | SCK 옆 핀 |
| I2S SDIN ← DOUT (SPH0645) | D2 | P1.06 | DOUT–GND 100 kΩ (d0-5) |
| I2S MCK | — | — | SPH0645는 MCK 불필요 → 핀 지정 안 함 |
| 마커 버튼 (Marker) | **내장 USR** | P0.00 | 배선 없음, 인터럽트 가능, DT에 `sw0`로 이미 정의 |
| (예비) 외부 마커 버튼 | D9 | P2.04 | P2 → **1 ms 폴링** + 디바운스. 손에 들고 누를 버튼이 필요할 때 |
| (Phase 2) 동기 LED/펄스 출력 | D10 | P2.02 | 출력은 P2도 문제없음 |
| 남겨 둠 (Reserved) | D6/D7 | P2.08/P2.07 | uart21 — 혹시 두 번째 UART가 필요할 때 |
| 남겨 둠 | D8 | P2.01 | |

→ P1 헤더 핀 6개(D0–D5)를 **모두 사용**. 버튼은 P1에 자리가 없어서 P0 내장 버튼으로.
All six P1 header pins (D0–D5) are used; the marker button moves to the on-board P0 button.

## 5. 남은 위험 · 확인할 것 (Open risks)

- [ ] **LRCK가 clock pin 필요?** — Day 3에서 I2S가 안 되면 첫 의심. 대안: D1/D5 교환은 SCL이 clock pin을 잃으므로 불가 → 그때는 PS의 I2S 장 원문 확인, 낮은 SCK(약 1 MHz)로 시험, 최후엔 DK에서 마이크 수집.
- [x] Seeed Wiki 핀아웃 그림과 D0–D10 표 일치 확인 (2026-11-10, 사용자 확인). Matches the Seeed pinout figure.
- USR 버튼 = 그림의 **USER_BUTTON (P0.00)** = USB-C 반대쪽 아래, **"B" 표시 옆** 택트 스위치. **"R" 옆은 RESET** — 생김새가 같으니 기록 중 잘못 누르지 않게 주의 (리셋되면 기기 시계·seq가 처음부터 다시 시작).
  USER button = the tactile switch next to "B"; the identical one next to "R" is RESET.
- [ ] 내장 USR 버튼을 마커로 쓰면 누를 때 보드·배선이 흔들릴 수 있음 → 본인 측정(Day 5) 전에 시험. 불편하면 D9 외부 버튼으로.
- [ ] 이 표는 Day 2(I2C·INT), Day 3(I2S pinctrl) overlay의 기준.

## 6. 실제 배선 기록 (As-wired, d1-2) — 2026-11-10

사진 (Photos): `wiring_1.jpeg`–`wiring_4.jpeg`. 선 색은 모듈 쪽 사진(3, 4)에서 읽음 / wire colours read at the module end.

| 모듈 핀 | 선 색 (Colour) | → XIAO |
|---------|----------------|--------|
| MAX30102 VIN | 초록 green | 3V3 |
| MAX30102 GND | 노랑 yellow | GND |
| MAX30102 SCL | 연회색 light grey | D5 (P1.11) |
| MAX30102 SDA | 자주 magenta | D4 (P1.10) |
| MAX30102 INT | 주황 orange | D3 (P1.07) |
| SPH0645 SEL | 갈색 brown | GND |
| SPH0645 LRCL | 검정 black | D1 (P1.05) |
| SPH0645 DOUT | 흰색 white | D2 (P1.06) + 100 kΩ → GND |
| SPH0645 BCLK | 회색 grey | D0 (P1.04) |
| SPH0645 GND | 보라 purple | GND |
| SPH0645 3V | 파랑 blue | 3V3 (**VBUS 아님 / not VBUS**) |

- 브레드보드: XIAO가 가운데 홈을 걸쳐 핀마다 다른 열에 꽂힘. 위·아래 전원 레일은 파란 점퍼로 연결.
- 전원 전 확인 (Pre-power check): 3V3–GND 단락 없음 ✅ (사용자 측정)
