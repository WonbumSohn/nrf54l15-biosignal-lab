/*
 * Gate 0 — XIAO nRF54L15 USB 브리지 처리량 시험 (USB-bridge throughput test)
 *
 * 동작 (What it does):
 *   uart20 (P1.09 TX → SAMD11 → USB CDC) 으로 type=0x7F TEST 프레임을 보낸다.
 *   목표 속도(kB/s)를 STEP_MS 마다 바꿔 가며 계속 반복하고, PC 스크립트
 *   (host/bridge_test.py)가 단계별 실수신 속도·seq 누락·CRC 오류를 센다.
 *   Sends type=0x7F TEST frames over uart20, stepping through target rates;
 *   the PC script measures received rate, seq gaps and CRC errors per step.
 *
 * 프레임 형식 (Frame format) — 계획서 1.3절, little-endian:
 *   sync(2)=A5 5A | type(1) | len(2) | seq(2) | t_dev_us(4) | first_idx(4)
 *   | payload(len) | crc(2) = CRC-16/XMODEM over sync..payload
 *
 * TEST payload (240 B):
 *   [0]      step_idx   uint8   — 현재 단계 번호 (current step index)
 *   [1..2]   target     uint16  — 목표 kB/s, 0 = 최대 속도 (0 = as fast as possible)
 *   [3..6]   baud       uint32  — 빌드된 baud (DT current-speed)
 *   [7..10]  tx_err     uint32  — uart_tx 실패·중단 누적 수 (tx errors/aborts so far)
 *   [11..]   pattern    (seq + i) & 0xFF
 *
 * first_idx = 부팅 후 보낸 프레임 총수 (total frames since boot, uint32)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/onoff.h>

/* ---- 프레임 상수 (Frame constants) ---- */
#define SYNC0         0xA5
#define SYNC1         0x5A
#define TYPE_TEST     0x7F
#define HDR_LEN       15
#define PAYLOAD_LEN   240
#define CRC_LEN       2
#define FRAME_LEN     (HDR_LEN + PAYLOAD_LEN + CRC_LEN) /* 257 B */

/* ---- 시험 단계 (Test steps) ---- */
#define STEP_MS       10000 /* 단계당 10초 (10 s per step) */
static const uint16_t step_target_kbps[] = {10, 20, 30, 40, 60, 80, 100, 0};
#define N_STEPS       ARRAY_SIZE(step_target_kbps)

#define UART_NODE     DT_NODELABEL(uart20)
#define UART_BAUD     DT_PROP(UART_NODE, current_speed)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* 이중 버퍼: 하나를 DMA로 보내는 동안 다른 하나를 채움
 * Double buffer: fill one while EasyDMA sends the other */
static uint8_t tx_buf[2][FRAME_LEN];

/* 1 = UART 비어 있음 (idle). 전송은 한 번에 하나만 (one transfer in flight) */
static K_SEM_DEFINE(tx_idle, 1, 1);
static atomic_t tx_err_count;

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_idle);
		break;
	case UART_TX_ABORTED:
		/* 중단도 오류로 세고 UART를 다시 비운 상태로 (count abort as error) */
		atomic_inc(&tx_err_count);
		k_sem_give(&tx_idle);
		break;
	default:
		break;
	}
}

/* 하트비트 LED: 0.5초마다 토글 → 펌웨어가 살아 있는지 눈으로 확인
 * Heartbeat LED toggles every 0.5 s */
static void led_timer_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	gpio_pin_toggle_dt(&led);
}
static K_TIMER_DEFINE(led_timer, led_timer_fn, NULL);

/* HFXO(정밀 32 MHz 수정 발진기) 켜기 — UART baud 오차를 줄임
 * Request HFXO so the UART baud is derived from the crystal */
static int hfxo_start(void)
{
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	struct onoff_client cli;
	int res = 0;
	int err;

	if (mgr == NULL) {
		return -ENODEV;
	}
	sys_notify_init_spinwait(&cli.notify);
	err = onoff_request(mgr, &cli);
	if (err < 0) {
		return err;
	}
	do {
		err = sys_notify_fetch_result(&cli.notify, &res);
	} while (err == -EAGAIN);

	return (err < 0) ? err : res;
}

static inline uint32_t now_us(void)
{
	/* 기기 시계 [µs], 약 71.6분마다 0으로 (wraps every ~71.6 min) */
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static void build_frame(uint8_t *f, uint16_t seq, uint32_t frame_idx,
			uint8_t step_idx, uint16_t target)
{
	uint8_t *p = &f[HDR_LEN];

	f[0] = SYNC0;
	f[1] = SYNC1;
	f[2] = TYPE_TEST;
	sys_put_le16(PAYLOAD_LEN, &f[3]);
	sys_put_le16(seq, &f[5]);
	sys_put_le32(now_us(), &f[7]);
	sys_put_le32(frame_idx, &f[11]);

	p[0] = step_idx;
	sys_put_le16(target, &p[1]);
	sys_put_le32(UART_BAUD, &p[3]);
	sys_put_le32((uint32_t)atomic_get(&tx_err_count), &p[7]);
	for (int i = 11; i < PAYLOAD_LEN; i++) {
		p[i] = (uint8_t)(seq + i);
	}

	uint16_t crc = crc16_itu_t(0, f, HDR_LEN + PAYLOAD_LEN);

	sys_put_le16(crc, &f[HDR_LEN + PAYLOAD_LEN]);
}

int main(void)
{
	uint16_t seq = 0;
	uint32_t frame_idx = 0;
	int cur = 0;
	int err;

	if (!device_is_ready(uart_dev)) {
		return -ENODEV; /* UART 준비 안 됨 → LED도 안 깜빡임 (no heartbeat) */
	}
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
		k_timer_start(&led_timer, K_MSEC(500), K_MSEC(500));
	}

	/* HFXO 실패해도 시험은 계속 (HFINT로 동작) / continue on HFINT if it fails */
	(void)hfxo_start();

	err = uart_callback_set(uart_dev, uart_cb, NULL);
	if (err) {
		return err;
	}

	while (1) {
		for (uint8_t step = 0; step < N_STEPS; step++) {
			const uint16_t target = step_target_kbps[step];
			const int64_t step_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
			const int64_t step_end_us = step_start_us + (int64_t)STEP_MS * 1000;
			/* 프레임 1개 간격 [µs] = FRAME_LEN / (target·1000 B/s) */
			const int64_t period_us =
				(target > 0) ? ((int64_t)FRAME_LEN * 1000 / target) : 0;
			uint32_t n = 0;

			while (1) {
				int64_t t = k_ticks_to_us_floor64(k_uptime_ticks());

				if (t >= step_end_us) {
					break;
				}
				if (period_us > 0) {
					int64_t due = step_start_us + (int64_t)n * period_us;

					if (due > t) {
						k_usleep((int32_t)(due - t));
					}
				}

				/* 지금 보내지 않는 버퍼를 채움 (fill the idle buffer) */
				build_frame(tx_buf[cur], seq, frame_idx, step, target);

				k_sem_take(&tx_idle, K_FOREVER);
				err = uart_tx(uart_dev, tx_buf[cur], FRAME_LEN, SYS_FOREVER_US);
				if (err) {
					atomic_inc(&tx_err_count);
					k_sem_give(&tx_idle);
					k_msleep(1);
					continue; /* 같은 seq로 다시 시도 (retry with the same seq) */
				}

				cur ^= 1;
				seq++;
				frame_idx++;
				n++;
			}
		}
	}

	return 0;
}
