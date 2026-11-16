/*
 * 프레임 송신 모듈 (Frame transmit module) — uart20 → SAMD11 → USB
 *
 * 형식은 계획서 1.3절 (format: plan section 1.3), little-endian:
 *   sync A5 5A | type | len(2) | seq(2) | t_dev_us(4) | first_idx(4) | payload | crc(2)
 * seq는 타입별로 따로 증가 (per-type sequence numbers).
 */
#ifndef FRAME_TX_H_
#define FRAME_TX_H_

#include <stdint.h>

#define FRAME_TYPE_PPG      1
#define FRAME_TYPE_MIC      2
#define FRAME_TYPE_EVENT    3
#define FRAME_TYPE_STATUS   4
#define FRAME_TYPE_MIC_PDM  5    /* 내장 PDM 마이크 (on-board PDM mic), int16 8 333.33 Hz */
#define FRAME_TYPE_IMU      6    /* 내장 IMU (on-board IMU), 6축 int16 104 Hz */
#define FRAME_TYPE_PPG_DIAG 0x7B /* MAX30102 진단 (PPG diagnostics) */
#define FRAME_TYPE_IMU_DIAG 0x7C /* 내장 IMU 진단 (on-board IMU diagnostics) */
#define FRAME_TYPE_PDM_DIAG 0x7D /* 내장 마이크 진단 (on-board mic diagnostics) */
#define FRAME_TYPE_MIC_DIAG 0x7E /* Day 3 진단용 (diagnostics) */
#define FRAME_TYPE_TEST     0x7F

#define FRAME_MAX_PAYLOAD   600

/** 초기화: UART 콜백·송신 스레드 시작 (init UART callback and TX thread). 0 = OK */
int frame_tx_init(void);

/**
 * 프레임 하나를 큐에 넣음 — 복사하므로 payload는 바로 재사용 가능.
 * Queue one frame (payload is copied).
 *
 * @return 0, -EMSGSIZE(payload 너무 큼), -ENOMEM(버퍼 풀 가득 → 버려짐, 카운트됨)
 */
int frame_tx_send(uint8_t type, uint32_t t_dev_us, uint32_t first_idx,
		  const void *payload, uint16_t len);

/** 버퍼가 없어 버린 프레임 수 (frames dropped because the pool was full) */
uint32_t frame_tx_dropped(void);

/** 기기 시계 [µs], uint32 (약 71.6분마다 0) / device clock in µs */
uint32_t dev_time_us(void);

#endif /* FRAME_TX_H_ */
