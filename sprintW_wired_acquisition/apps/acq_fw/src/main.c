/*
 * Sprint W 수집 펌웨어 (Acquisition firmware) — XIAO nRF54L15
 *
 * Day 3: SPH0645 마이크(I2S) → MIC / MIC_DIAG 프레임
 *        내장 PDM 마이크     → MIC_PDM / PDM_DIAG 프레임 → uart20 (1 Mbaud) → PC
 *        내장 IMU           → IMU / IMU_DIAG 프레임
 * Day 2: MAX30102 (i2c22)   → PPG / PPG_DIAG 프레임
 *        (CMake -DMIC_I2S=0 / -DMIC_PDM=0 / -DIMU=0 / -DPPG=0 으로 각각 끔 / disable each)
 * (Day 4 통합은 이 위에 추가 / integration is added later)
 *
 * LED: 0.5초 깜빡임 = 정상, 빠른 깜빡임(0.1초) = 초기화 실패
 * LED: 0.5 s blink = running, fast 0.1 s blink = init failed
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "frame_tx.h"
#include "mic.h"
#include "mic_pdm.h"
#include "imu.h"
#include "max30102.h"

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	int blink_ms = 500;

	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	if (frame_tx_init() != 0) {
		blink_ms = 100; /* 초기화 실패 표시 (init failure) */
	}
#if MIC_I2S
	if (mic_start() != 0) {
		blink_ms = 100;
	}
#endif
#if MIC_PDM
	if (mic_pdm_start() != 0) {
		blink_ms = 100;
	}
#endif
#if IMU
	if (imu_start() != 0) {
		blink_ms = 100;
	}
#endif
#if PPG
	if (max30102_start() != 0) {
		blink_ms = 100;
	}
#endif

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(blink_ms);
	}
	return 0;
}
