/*
 * MAX30102 PPG 모듈 (MAX30102 PPG module) — Day 2
 *
 * i2c22 (xiao_i2c): SDA=D4(P1.10), SCL=D5(P1.11), 주소 0x57, INT=D3(P1.07, active low)
 *
 * 설정 (Config): SpO2 모드 (Red + IR), 100 Hz, 펄스폭 411 µs (18비트), ADC 4096 nA,
 *   평균 없음, FIFO A_FULL = 미읽음 17개 → INT
 *   LED 전류는 CMake -DPPG_LED_RED=0x24 -DPPG_LED_IR=0x24 (0.2 mA/LSB)
 * 출력 (Output):
 *   PPG (type 1): 샘플 N개 × (red int32, ir int32), 보통 N = 17 (약 170 ms)
 *   PPG_DIAG (0x7B): 약 1초마다 Part ID, 설정, I2C 스캔 결과, 오버플로·오류
 */
#ifndef MAX30102_H_
#define MAX30102_H_

/** PPG 스레드 시작 (start the PPG thread). 0 = OK. 칩 초기화는 스레드에서 재시도 */
int max30102_start(void);

#endif /* MAX30102_H_ */
