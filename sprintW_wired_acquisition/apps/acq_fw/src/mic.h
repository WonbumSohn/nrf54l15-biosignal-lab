/*
 * SPH0645 I2S 마이크 모듈 (SPH0645 I2S microphone module)
 *
 * I2S20 Master, 32비트 워드 × 2채널, LRCK 33 333.33 Hz (MCK 32 MHz/15, RATIO 64)
 * → 18비트 값 → FIR decimation ÷4 → 8 333.33 Hz int16 → MIC 프레임
 */
#ifndef MIC_H_
#define MIC_H_

/** I2S 설정 + 수신 스레드 시작 (configure I2S and start the RX thread). 0 = OK */
int mic_start(void);

#endif /* MIC_H_ */
