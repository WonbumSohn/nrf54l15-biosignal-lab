/*
 * 내장 PDM 마이크 모듈 (On-board PDM microphone, XIAO nRF54L15 Sense)
 *
 * PDM20: CLK P1.12, DIN P1.13 (보드 DTS / board DTS), 전원 P0.01 (regulator-boot-on)
 * PDM CLK = 32 MHz / 24 = 1.3333 MHz, RATIO 80 → 16 666.67 Hz 스테레오 캡처
 * → 선택 채널 FIR decimation ÷2 → 8 333.33 Hz int16 → MIC_PDM 프레임 (type 5)
 * (I2S SPH0645 경로와 같은 출력 레이트 / same output rate as the I2S path)
 */
#ifndef MIC_PDM_H_
#define MIC_PDM_H_

/** PDM 설정 + 수신 스레드 시작 (configure PDM and start the RX thread). 0 = OK */
int mic_pdm_start(void);

#endif /* MIC_PDM_H_ */
