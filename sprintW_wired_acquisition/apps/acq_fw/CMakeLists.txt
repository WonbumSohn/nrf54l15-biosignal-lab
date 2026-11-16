# SPDX-License-Identifier: Apache-2.0
# Sprint W 수집 펌웨어 (Acquisition firmware) — Day 2: PPG, Day 3: 마이크·IMU, Day 4: 통합

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(acq_fw)

target_sources(app PRIVATE
	src/main.c
	src/frame_tx.c
	src/mic.c
	src/mic_pdm.c
	src/imu.c
	src/max30102.c
)

# MIC_RAW=1 이면 decimation 없이 33.3 kHz 원본을 보냄 (짧은 비교 시험용)
# MIC_RAW=1 sends the 33.3 kHz stream without decimation (short comparison test)
if(DEFINED MIC_RAW)
	target_compile_definitions(app PRIVATE MIC_RAW=${MIC_RAW})
endif()

# 센서 선택 (Sensor selection) — 기본은 모두 켬 (all on by default)
#   -DMIC_I2S=0 : SPH0645(I2S) 끔 / disable SPH0645
#   -DMIC_PDM=0 : 내장 PDM 마이크 끔 / disable on-board PDM mic
#   -DIMU=0     : 내장 IMU 끔 / disable on-board IMU
#   -DPPG=0     : MAX30102 끔 / disable MAX30102
#   -DPPG_LED_RED=0x24 -DPPG_LED_IR=0x24 : PPG LED 전류 (0.2 mA/LSB)
#   -DPDM_OUT_CH=1 : PDM 출력 채널 R / output the R channel
foreach(opt MIC_I2S MIC_PDM IMU PPG)
	if(NOT DEFINED ${opt})
		set(${opt} 1)
	endif()
	target_compile_definitions(app PRIVATE ${opt}=${${opt}})
endforeach()
if(DEFINED PDM_OUT_CH)
	target_compile_definitions(app PRIVATE PDM_OUT_CH=${PDM_OUT_CH})
endif()
foreach(opt PPG_LED_RED PPG_LED_IR)
	if(DEFINED ${opt})
		target_compile_definitions(app PRIVATE ${opt}=${${opt}})
	endif()
endforeach()
