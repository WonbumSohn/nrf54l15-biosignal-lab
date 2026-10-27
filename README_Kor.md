# nrf54l15-biosignal-lab

[English](README.md) | **한국어**

**Nordic nRF54L15** SoC를 **Zephyr RTOS**(nRF Connect SDK)로 깊이 익히고, 이를 웨어러블 생체신호 처리에 적용하는 실습 학습 기록입니다.

첫 번째 대상은 **PPG**(광용적맥파)이며, 앞으로 **ECG**, **EEG** 등 다른 생체신호로 넓혀 갈 예정입니다.

## 목표

- Cortex-M33 코어와 RRAM부터 페리페럴, BLE 무선, 저전력 모드까지 nRF54L15를 깊이 이해합니다.
- DeviceTree, Kconfig, 스레드, 드라이버를 활용해 Zephyr 방식으로 펌웨어를 만듭니다.
- nRF54L15 DK에서 SWD/RTT로 디버깅하며 개발하고, DeviceTree overlay만 바꿔 같은 코드를 XIAO nRF54L15로 이식합니다.
- 센서 → 필터링 → 특징 추출 → BLE로 이어지는 배터리 구동 웨어러블 파이프라인을 만듭니다.
- 전력 프로파일링, 배터리 수명 모델링, 정확도 지표 등 논문에서 쓰는 방식으로 성능을 평가합니다.

## 하드웨어

| 장비 | 역할 |
|------|------|
| Seeed Studio XIAO nRF54L15 | 최종 웨어러블 보드 |
| Nordic nRF54L15 DK | 개발 및 SWD/RTT 디버깅 |
| Power Profiler Kit II (PPK2) | 전류·에너지 측정 |
| 오실로스코프 / 멀티미터 | 신호·전원 확인 |

## 로드맵

| Sprint | 주제 | 상태 |
|--------|------|------|
| 1 | 개발 환경 + Zephyr 기초 | ⏳ 예정 |
| 2 | GPIO + DeviceTree + SWD/RTT 디버깅 | ⏳ 예정 |
| 3 | 배터리 우선 동작 + BLE 디버그 채널 | ⏳ 예정 |
| 4 | UART + 로깅/Shell | ⏳ 예정 |
| 5 | I2C + 첫 생체신호 센서 (PPG) | ⏳ 예정 |
| 6 | 타이머, 인터럽트, 워크큐, PWM | ⏳ 예정 |
| 7 | 저전력 심화 + 논문식 전력 분석 | ⏳ 예정 |
| 7.5 | 온디바이스 DSP + 피크 검출 + 심박수 | ⏳ 예정 |
| 8 | BLE GATT: 처리량과 신뢰성 | ⏳ 예정 |
| 9 | 시스템 통합 + 논문 형식 평가 리포트 | ⏳ 예정 |
| 10+ | 생체신호 확장 (ECG, EEG, SpO2 등) | 🔭 향후 |

각 Sprint 폴더에는 코드, 노트, 회고(어려웠던 점, 흥미로웠던 점, 더 알고 싶은 점)가 함께 담깁니다.

## 레포 구조

Sprint가 진행되면서 폴더가 추가됩니다. 예시는 다음과 같습니다.

```
nrf54l15-biosignal-lab/
├── sprint1_morse_blinky/   # Zephyr 앱 (src/, prj.conf, CMakeLists.txt, boards/)
│   └── README.md           # 만든 내용 + 회고
├── sprint2_.../
└── ...
```

## 개발 도구

- nRF Connect SDK (Zephyr RTOS)
- VS Code + nRF Connect for VS Code 확장
- `west` 빌드/플래시
- 신호 분석용 Python + Jupyter

## 작성자

손원범 Wonbum Sohn ([@WonbumSohn](https://github.com/WonbumSohn))
