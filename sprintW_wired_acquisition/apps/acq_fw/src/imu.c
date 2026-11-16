/*
 * 내장 IMU 수신 (On-board IMU capture) — LSM6DSO / LSM6DS3TR-C 공통 레지스터 사용
 *
 * 흐름 (Flow):
 *   INT1(P0.02) 상승 에지 = 새 가속도 샘플 (DRDY_XL) → 시각 기록 + 세마포어
 *   → 스레드가 0x1E부터 16 B 한 번에 읽음 (STATUS, 온도, 자이로, 가속도)
 *   DRDY는 데이터를 읽을 때까지 high로 유지되므로 에지를 놓치면 멈출 수 있음 →
 *   30 ms 시간 초과 때도 STATUS를 읽어 복구 (timeout path recovers a missed edge)
 *
 * 레지스터 (Registers, 두 칩 동일 / identical on both chips):
 *   0x0D INT1_CTRL, 0x0F WHO_AM_I, 0x10 CTRL1_XL, 0x11 CTRL2_G, 0x12 CTRL3_C,
 *   0x1E STATUS_REG, 0x20 OUT_TEMP, 0x22 OUTX_L_G … 0x2D OUTZ_H_XL
 */
#include "imu.h"
#include "frame_tx.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#define IMU_NODE DT_NODELABEL(lsm6dso)

/* 레지스터 (registers) */
#define REG_INT1_CTRL  0x0D
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_STATUS     0x1E

#define WHO_LSM6DS3TRC 0x6A
#define WHO_LSM6DSO    0x6C

/* 설정 값 (config values) */
#define ODR_104HZ      0x40       /* ODR[7:4] = 0100 */
#define FS_XL_4G       0x08       /* FS_XL[3:2] = 10 → ±4 g */
#define FS_G_500DPS    0x04       /* FS_G[3:2] = 01 → ±500 dps */
#define CTRL3_BDU      BIT(6)     /* 블록 단위 갱신 (block data update) */
#define CTRL3_IF_INC   BIT(2)     /* 주소 자동 증가 (auto-increment) */
#define CTRL3_SW_RESET BIT(0)
#define INT1_DRDY_XL   BIT(0)
#define STATUS_XLDA    BIT(0)

#define IMU_ODR_HZ     104
#define IMU_FS_G       4
#define IMU_FS_DPS     500
#define SAMPLES_PER_FRAME 8       /* 약 77 ms (≈ 77 ms) */
#define AXES           6
#define DIAG_EVERY     13         /* 13 프레임 = 104 샘플 ≈ 1 s */
#define WAIT_MS        30         /* 샘플 간격 9.6 ms의 약 3배 (≈ 3 periods) */

BUILD_ASSERT(SAMPLES_PER_FRAME * AXES * sizeof(int16_t) <= FRAME_MAX_PAYLOAD, "frame too large");

static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(IMU_NODE);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(IMU_NODE, irq_gpios);
static struct gpio_callback irq_cb;

static K_SEM_DEFINE(drdy_sem, 0, 1);
static volatile uint32_t drdy_t_us; /* 마지막 INT1 에지 시각 (last INT1 edge time) */

struct imu_stats {
	uint8_t who;          /* WHO_AM_I (0 = 아직 못 읽음 / not read yet) */
	int8_t init_err;      /* 마지막 초기화 결과 (last init result, -errno) */
	int16_t temp_raw;     /* 256 LSB/°C, 0 = 25 °C */
	uint32_t samples;     /* 누적 샘플 (total samples) */
	uint32_t gaps;        /* 간격 > 1.5 주기 횟수 (intervals > 1.5 periods) */
	uint32_t i2c_errors;
	uint32_t timeouts;    /* INT1 없이 30 ms 경과 (no INT1 for 30 ms) */
	uint32_t dt_min_us;   /* 창 내 샘플 간격 (sample interval in window) */
	uint32_t dt_max_us;
};

static struct imu_stats st;

static void drdy_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	drdy_t_us = dev_time_us();
	k_sem_give(&drdy_sem);
}

static void send_diag(void)
{
	uint8_t p[40]; /* struct "<BBbBHHHhIIIIIII" */
	uint8_t *q = p;

	*q++ = 1;                                  /* version */
	*q++ = st.who;
	*q++ = (uint8_t)st.init_err;
	*q++ = 0;                                  /* reserved */
	sys_put_le16(IMU_ODR_HZ, q); q += 2;
	sys_put_le16(IMU_FS_G, q); q += 2;
	sys_put_le16(IMU_FS_DPS, q); q += 2;
	sys_put_le16((uint16_t)st.temp_raw, q); q += 2;
	sys_put_le32(st.samples, q); q += 4;
	sys_put_le32(st.gaps, q); q += 4;
	sys_put_le32(st.i2c_errors, q); q += 4;
	sys_put_le32(st.timeouts, q); q += 4;
	sys_put_le32(frame_tx_dropped(), q); q += 4;
	sys_put_le32(st.dt_min_us == UINT32_MAX ? 0 : st.dt_min_us, q); q += 4;
	sys_put_le32(st.dt_max_us, q); q += 4;

	(void)frame_tx_send(FRAME_TYPE_IMU_DIAG, dev_time_us(), 0, p, (uint16_t)(q - p));
	st.dt_min_us = UINT32_MAX;
	st.dt_max_us = 0;
}

/* 칩 확인 + 설정 (identify and configure). 0 = OK */
static int chip_init(void)
{
	uint8_t who = 0;
	uint8_t v = 0;
	int err = i2c_reg_read_byte_dt(&bus, REG_WHO_AM_I, &who);

	if (err) {
		return err;
	}
	st.who = who;
	if (who != WHO_LSM6DS3TRC && who != WHO_LSM6DSO) {
		return -ENODEV;
	}

	/* 소프트 리셋 → 보드 DTS의 저전력 모드 등 이전 설정 제거 (clears ULP etc.) */
	err = i2c_reg_write_byte_dt(&bus, REG_CTRL3_C, CTRL3_SW_RESET);
	if (err) {
		return err;
	}
	for (int i = 0; i < 10; i++) {
		k_msleep(1);
		err = i2c_reg_read_byte_dt(&bus, REG_CTRL3_C, &v);
		if (!err && !(v & CTRL3_SW_RESET)) {
			break;
		}
	}
	if (err || (v & CTRL3_SW_RESET)) {
		return err ? err : -ETIMEDOUT;
	}

	const uint8_t cfg[][2] = {
		{REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC},
		{REG_CTRL1_XL, ODR_104HZ | FS_XL_4G},
		{REG_CTRL2_G, ODR_104HZ | FS_G_500DPS},
		{REG_INT1_CTRL, INT1_DRDY_XL},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cfg); i++) {
		err = i2c_reg_write_byte_dt(&bus, cfg[i][0], cfg[i][1]);
		if (err) {
			return err;
		}
	}
	return 0;
}

static void imu_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	int16_t buf[SAMPLES_PER_FRAME * AXES];
	uint32_t n = 0;          /* 버퍼 안 샘플 수 (samples in buffer) */
	uint32_t idx = 0;        /* 누적 샘플 번호 (sample index) */
	uint32_t frames = 0;
	uint32_t last_t = 0;
	bool have_last = false;
	const uint32_t period_us = 1000000U / IMU_ODR_HZ;

	st.dt_min_us = UINT32_MAX;
	k_msleep(20); /* 전원 안정화 (power-up, startup-delay 5 ms + 여유) */

	while ((st.init_err = (int8_t)chip_init()) != 0) {
		send_diag(); /* WHO_AM_I·오류를 PC에 알림 (report failure) */
		k_msleep(1000);
	}

	while (1) {
		uint8_t raw[16]; /* 0x1E STATUS, 0x1F, 0x20-21 TEMP, 0x22-27 G, 0x28-2D XL */
		uint32_t t_us;

		if (k_sem_take(&drdy_sem, K_MSEC(WAIT_MS)) != 0) {
			st.timeouts++;
			t_us = dev_time_us();
		} else {
			t_us = drdy_t_us;
		}

		if (i2c_burst_read_dt(&bus, REG_STATUS, raw, sizeof(raw)) != 0) {
			st.i2c_errors++;
			k_msleep(10);
			continue;
		}
		if (!(raw[0] & STATUS_XLDA)) {
			continue; /* 새 데이터 없음 (no new sample) */
		}

		if (have_last) {
			const uint32_t dt = t_us - last_t;

			st.dt_min_us = MIN(st.dt_min_us, dt);
			st.dt_max_us = MAX(st.dt_max_us, dt);
			if (dt > period_us + period_us / 2) {
				st.gaps++;
			}
		}
		last_t = t_us;
		have_last = true;

		st.temp_raw = (int16_t)sys_get_le16(&raw[2]);
		/* 출력 순서: 가속도 xyz, 자이로 xyz (output order: accel then gyro) */
		int16_t *s = &buf[n * AXES];

		for (int k = 0; k < 3; k++) {
			s[k] = (int16_t)sys_get_le16(&raw[10 + 2 * k]);    /* XL 0x28 */
			s[3 + k] = (int16_t)sys_get_le16(&raw[4 + 2 * k]); /* G 0x22 */
		}
		st.samples++;

		if (++n == SAMPLES_PER_FRAME) {
			/* t_dev_us = 마지막 샘플 시각, first_idx = 첫 샘플 번호 */
			(void)frame_tx_send(FRAME_TYPE_IMU, t_us, idx, buf, sizeof(buf));
			idx += n;
			n = 0;
			if (++frames % DIAG_EVERY == 0) {
				send_diag();
			}
		}
	}
}

K_THREAD_STACK_DEFINE(imu_stack, 1536);
static struct k_thread imu_thread_data;

int imu_start(void)
{
	int err;

	if (!i2c_is_ready_dt(&bus) || !gpio_is_ready_dt(&irq)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&irq, GPIO_INPUT);
	if (err) {
		return err;
	}
	gpio_init_callback(&irq_cb, drdy_isr, BIT(irq.pin));
	err = gpio_add_callback_dt(&irq, &irq_cb);
	if (err) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}
	k_thread_create(&imu_thread_data, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack),
			imu_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread_data, "imu");
	return 0;
}
