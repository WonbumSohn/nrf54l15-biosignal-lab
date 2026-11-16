/*
 * 내장 IMU 모듈 (On-board IMU, XIAO nRF54L15 Sense)
 *
 * i2c30: SDA P0.03, SCL P0.04, 주소 0x6A, INT1 P0.02 (보드 DTS / board DTS)
 * 전원은 PDM 마이크와 공유 (P0.01 pdm_imu_pwr, regulator-boot-on)
 *
 * 칩: 보드 DTS는 LSM6DSO(WHO_AM_I 0x6C), Seeed 자료는 LSM6DS3TR-C(0x6A).
 * 두 칩 모두 이 모듈이 쓰는 레지스터 주소가 같아 직접 레지스터로 다룸.
 * Chip: DTS says LSM6DSO, Seeed docs say LSM6DS3TR-C; both share the registers
 * used here, so this module talks to registers directly (no Zephyr sensor driver).
 *
 * 설정 (Config): 104 Hz, 가속도 ±4 g, 자이로 ±500 dps, DRDY_XL → INT1
 * 출력 (Output):
 *   IMU (type 6): 샘플 8개 × [ax, ay, az, gx, gy, gz] int16 (96 B, 약 77 ms)
 *   IMU_DIAG (0x7C): 약 1초마다 WHO_AM_I, 온도, 샘플 간격 최소/최대, 오류
 */
#ifndef IMU_H_
#define IMU_H_

/** IMU 스레드 시작 (start the IMU thread). 0 = OK. 칩 초기화는 스레드에서 재시도 */
int imu_start(void);

#endif /* IMU_H_ */
