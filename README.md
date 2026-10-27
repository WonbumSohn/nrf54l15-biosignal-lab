# nrf54l15-biosignal-lab

**English** | [한국어](README_Kor.md)

A hands-on learning log for mastering the **Nordic nRF54L15** SoC with **Zephyr RTOS** (nRF Connect SDK), and applying it to wearable biosignal processing.

The first target is **PPG** (photoplethysmography). The lab will grow to cover other biosignals such as **ECG** and **EEG**.

## Goals

- Understand the nRF54L15 in depth, from the Cortex-M33 core and RRAM to its peripherals, BLE radio, and low-power modes.
- Build firmware the Zephyr way, using DeviceTree, Kconfig, threads, and drivers.
- Develop on the nRF54L15 DK with SWD/RTT debugging, then port the same code to the XIAO nRF54L15 by changing only the DeviceTree overlay.
- Build a battery-powered wearable pipeline: sensor → filtering → feature extraction → BLE.
- Evaluate it the way research papers do, with power profiling, battery-life modeling, and accuracy metrics.

## Hardware

| Item | Role |
|------|------|
| Seeed Studio XIAO nRF54L15 | Target wearable board |
| Nordic nRF54L15 DK | Development and SWD/RTT debugging |
| Power Profiler Kit II (PPK2) | Current and energy measurement |
| Oscilloscope / multimeter | Signal and power checks |

## Roadmap

| Sprint | Topic | Status |
|--------|-------|--------|
| 1 | Development environment + Zephyr basics | ⏳ Planned |
| 2 | GPIO + DeviceTree + SWD/RTT debugging | ⏳ Planned |
| 3 | Battery-first operation + BLE debug channel | ⏳ Planned |
| 4 | UART + logging/shell | ⏳ Planned |
| 5 | I2C + first biosignal sensor (PPG) | ⏳ Planned |
| 6 | Timers, interrupts, work queues, PWM | ⏳ Planned |
| 7 | Low-power deep dive + paper-style power analysis | ⏳ Planned |
| 7.5 | On-device DSP + peak detection + heart rate | ⏳ Planned |
| 8 | BLE GATT: throughput and reliability | ⏳ Planned |
| 9 | System integration + paper-style evaluation report | ⏳ Planned |
| 10+ | More biosignals (ECG, EEG, SpO2, ...) | 🔭 Future |

Each sprint folder will include the code, notes, and a retrospective covering what was hard, what was interesting, and what to learn next.

## Repository Structure

Folders are added as sprints progress, for example:

```
nrf54l15-biosignal-lab/
├── sprint1_morse_blinky/   # Zephyr app (src/, prj.conf, CMakeLists.txt, boards/)
│   └── README.md           # What was built + retrospective
├── sprint2_.../
└── ...
```

## Toolchain

- nRF Connect SDK (Zephyr RTOS)
- VS Code + nRF Connect for VS Code extension
- `west` build/flash
- Python + Jupyter for signal analysis

## Author

Wonbum Sohn ([@WonbumSohn](https://github.com/WonbumSohn))
