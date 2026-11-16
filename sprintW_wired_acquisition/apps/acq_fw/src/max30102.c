/*
 * MAX30102 PPG 수신 (MAX30102 PPG capture) — Day 2, 레지스터 직접 접근 (raw registers)
 *
 * 흐름 (Flow):
 *   FIFO에 미읽음 샘플 17개가 쌓이면 A_FULL → INT(D3) low → ISR: 시각 기록 + 세마포어
 *   → 스레드: INT Status 1 읽기(INT 해제) → FIFO 포인터 → FIFO 일괄 읽기 → PPG 프레임
 *   ISR에서는 I2C를 쓰지 않음 (no I2C in ISR). INT를 놓쳐도 300 ms 시간 초과 경로가
 *   같은 읽기를 해서 복구 (timeout path recovers a missed edge).
 *   (계획 d2-5는 워크큐였지만 IMU와 같은 전용 스레드 구조로 통일 / thread instead of workqueue)
 *
 * 시각 (Timestamp): 에지 = 미읽음이 정확히 17개가 된 순간 → 그 17번째 샘플의 시각.
 *   읽을 때 더 쌓였으면 (n > 17) 주기 × (n − 17) 만큼 더함. 시간 초과 경로는 읽은 시각.
 *
 * 레지스터 (Registers, MAX30102 datasheet rev 1):
 *   0x00 INT_STATUS1 (A_FULL b7, PPG_RDY b6, ALC_OVF b5, PWR_RDY b0), 0x02 INT_EN1,
 *   0x04 FIFO_WR_PTR, 0x05 OVF_COUNTER, 0x06 FIFO_RD_PTR, 0x07 FIFO_DATA,
 *   0x08 FIFO_CONFIG, 0x09 MODE_CONFIG, 0x0A SPO2_CONFIG, 0x0C LED1(Red) PA, 0x0D LED2(IR) PA,
 *   0xFE REV_ID, 0xFF PART_ID (= 0x15)
 */
#include "max30102.h"
#include "frame_tx.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#define PPG_NODE DT_NODELABEL(max30102)

#define REG_INT_STATUS1 0x00
#define REG_INT_EN1     0x02
#define REG_INT_EN2     0x03
#define REG_FIFO_WR     0x04
#define REG_FIFO_DATA   0x07
#define REG_FIFO_CFG    0x08
#define REG_MODE_CFG    0x09
#define REG_SPO2_CFG    0x0A
#define REG_LED1_PA     0x0C
#define REG_LED2_PA     0x0D
#define REG_REV_ID      0xFE
#define REG_PART_ID     0xFF

#define PART_ID_MAX30102 0x15

#define INT_A_FULL   BIT(7)
#define INT_PWR_RDY  BIT(0)
#define MODE_RESET   BIT(6)
#define MODE_SPO2    0x03 /* Red + IR */

/* FIFO_CONFIG: SMP_AVE[7:5]=000(평균 없음), ROLLOVER_EN=0, A_FULL[3:0]=15 → 미읽음 17개 */
#define FIFO_A_FULL  15
#define FIFO_CFG_VAL (FIFO_A_FULL)
#define FIFO_DEPTH   32
#define BATCH        (FIFO_DEPTH - FIFO_A_FULL) /* 17 */

/* SPO2_CONFIG: ADC_RGE[6:5]=01(4096 nA), SR[4:2]=001(100 Hz), LED_PW[1:0]=11(411 µs, 18 bit) */
#define SPO2_CFG_VAL ((1 << 5) | (1 << 2) | 3) /* 0x27 */
#define PPG_RATE_HZ  100

/* LED 전류 (LED current), 0.2 mA/LSB: 0x24 = 7.2 mA. CMake -DPPG_LED_RED / -DPPG_LED_IR */
#ifndef PPG_LED_RED
#define PPG_LED_RED 0x24
#endif
#ifndef PPG_LED_IR
#define PPG_LED_IR 0x24
#endif

#define WAIT_MS      300 /* 배치 간격 170 ms의 약 2배 (≈ 2 batches) */
#define DIAG_MS      1000
#define SAMPLE_MASK  0x3FFFF /* 18비트 (18-bit) */

static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(PPG_NODE);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(PPG_NODE, int_gpios);
static struct gpio_callback irq_cb;

static K_SEM_DEFINE(int_sem, 0, 1);
static volatile uint32_t int_t_us; /* 마지막 INT 에지 시각 (last INT edge time) */

struct ppg_stats {
	uint8_t part;         /* PART_ID (0 = 아직 못 읽음 / not read yet) */
	uint8_t rev;
	int8_t init_err;      /* 마지막 초기화 결과 (last init result, -errno) */
	uint8_t scan_n;       /* i2c22에서 ACK한 주소 개수 (addresses that ACKed) */
	uint8_t scan[3];      /* 처음 3개 (first three) */
	uint32_t samples;     /* 보낸 샘플 (samples sent) */
	uint32_t ovf_samples; /* FIFO가 넘쳐 잃은 샘플 (OVF_COUNTER 합) */
	uint32_t i2c_errors;
	uint32_t timeouts;    /* INT 없이 300 ms 경과 (no INT for 300 ms) */
	uint32_t reinits;     /* PWR_RDY 감지 → 재초기화 (chip reset detected) */
	uint16_t batch_min;   /* 창 내 한 번에 읽은 샘플 수 (samples per read in window) */
	uint16_t batch_max;
};

static struct ppg_stats st;

static void int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	int_t_us = dev_time_us();
	k_sem_give(&int_sem);
}

static void send_diag(void)
{
	uint8_t p[40]; /* struct "<BBBbBBBBB3sIIIIIIHH" */
	uint8_t *q = p;

	*q++ = 1;                 /* version */
	*q++ = st.part;
	*q++ = st.rev;
	*q++ = (uint8_t)st.init_err;
	*q++ = SPO2_CFG_VAL;
	*q++ = FIFO_CFG_VAL;
	*q++ = PPG_LED_RED;
	*q++ = PPG_LED_IR;
	*q++ = st.scan_n;
	memcpy(q, st.scan, 3); q += 3;
	sys_put_le32(st.samples, q); q += 4;
	sys_put_le32(st.ovf_samples, q); q += 4;
	sys_put_le32(st.i2c_errors, q); q += 4;
	sys_put_le32(st.timeouts, q); q += 4;
	sys_put_le32(frame_tx_dropped(), q); q += 4;
	sys_put_le32(st.reinits, q); q += 4;
	sys_put_le16(st.batch_min == UINT16_MAX ? 0 : st.batch_min, q); q += 2;
	sys_put_le16(st.batch_max, q); q += 2;

	(void)frame_tx_send(FRAME_TYPE_PPG_DIAG, dev_time_us(), 0, p, (uint16_t)(q - p));
	st.batch_min = UINT16_MAX;
	st.batch_max = 0;
}

/* i2c22 주소 스캔 (d2-2): 1바이트 읽기에 ACK한 주소 기록 (record addresses that ACK) */
static void bus_scan(void)
{
	st.scan_n = 0;
	for (uint16_t a = 0x08; a <= 0x77; a++) {
		uint8_t b;

		if (i2c_read(bus.bus, &b, 1, a) == 0) {
			if (st.scan_n < ARRAY_SIZE(st.scan)) {
				st.scan[st.scan_n] = (uint8_t)a;
			}
			st.scan_n++;
		}
	}
}

/* 칩 확인 + 설정 (identify and configure). 0 = OK */
static int chip_init(void)
{
	uint8_t v = 0;
	int err = i2c_reg_read_byte_dt(&bus, REG_PART_ID, &st.part);

	if (err) {
		return err;
	}
	(void)i2c_reg_read_byte_dt(&bus, REG_REV_ID, &st.rev);
	if (st.part != PART_ID_MAX30102) {
		return -ENODEV;
	}

	/* 리셋 → RESET 비트가 0이 될 때까지 (reset, wait for the bit to self-clear) */
	err = i2c_reg_write_byte_dt(&bus, REG_MODE_CFG, MODE_RESET);
	if (err) {
		return err;
	}
	for (int i = 0; i < 10; i++) {
		k_msleep(1);
		err = i2c_reg_read_byte_dt(&bus, REG_MODE_CFG, &v);
		if (!err && !(v & MODE_RESET)) {
			break;
		}
	}
	if (err || (v & MODE_RESET)) {
		return err ? err : -ETIMEDOUT;
	}

	/* 전원 켤 때 생긴 PWR_RDY 해제 → INT가 high로 복귀 (clear power-up PWR_RDY, d2-5) */
	err = i2c_reg_read_byte_dt(&bus, REG_INT_STATUS1, &v);
	if (err) {
		return err;
	}

	const uint8_t cfg[][2] = {
		{REG_FIFO_CFG, FIFO_CFG_VAL},
		{REG_SPO2_CFG, SPO2_CFG_VAL},
		{REG_LED1_PA, PPG_LED_RED},
		{REG_LED2_PA, PPG_LED_IR},
		{REG_INT_EN1, INT_A_FULL},
		{REG_INT_EN2, 0},
		{REG_FIFO_WR, 0},     /* FIFO_WR_PTR */
		{REG_FIFO_WR + 1, 0}, /* OVF_COUNTER */
		{REG_FIFO_WR + 2, 0}, /* FIFO_RD_PTR */
		{REG_MODE_CFG, MODE_SPO2},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cfg); i++) {
		err = i2c_reg_write_byte_dt(&bus, cfg[i][0], cfg[i][1]);
		if (err) {
			return err;
		}
	}
	return 0;
}

static void init_until_ok(void)
{
	while ((st.init_err = (int8_t)chip_init()) != 0) {
		bus_scan();
		send_diag(); /* Part ID·스캔·오류를 PC에 알림 (report failure) */
		k_msleep(1000);
	}
}

static void ppg_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	static uint8_t raw[FIFO_DEPTH * 6];      /* 샘플당 Red 3 B + IR 3 B */
	static uint8_t out[FIFO_DEPTH * 8];      /* 샘플당 red int32 + ir int32 */
	const uint32_t period_us = 1000000U / PPG_RATE_HZ;
	uint32_t idx = 0;                        /* 누적 샘플 번호 (sample index) */
	int64_t next_diag;

	BUILD_ASSERT(sizeof(out) <= FRAME_MAX_PAYLOAD, "frame too large");

	st.batch_min = UINT16_MAX;
	k_msleep(10); /* 모듈 전원 안정화 (power-up) */
	bus_scan();
	init_until_ok();
	next_diag = k_uptime_get() + DIAG_MS;

	while (1) {
		uint8_t s1;
		uint8_t ptr[3]; /* WR_PTR, OVF_COUNTER, RD_PTR */
		uint32_t t_us;
		bool edge;

		if (k_sem_take(&int_sem, K_MSEC(WAIT_MS)) != 0) {
			st.timeouts++;
			edge = false;
			t_us = dev_time_us();
		} else {
			edge = true;
			t_us = int_t_us;
		}

		if (k_uptime_get() >= next_diag) {
			send_diag();
			next_diag += DIAG_MS;
		}

		/* INT 해제 + 칩 리셋 감지 (clear INT, detect a chip reset via PWR_RDY) */
		if (i2c_reg_read_byte_dt(&bus, REG_INT_STATUS1, &s1) != 0 ||
		    i2c_burst_read_dt(&bus, REG_FIFO_WR, ptr, sizeof(ptr)) != 0) {
			st.i2c_errors++;
			k_msleep(10);
			continue;
		}
		if (s1 & INT_PWR_RDY) {
			st.reinits++;
			init_until_ok();
			continue;
		}

		uint32_t n = (ptr[0] - ptr[2]) & (FIFO_DEPTH - 1);
		const uint32_t ovf = ptr[1] & 0x1F;

		if (n == 0 && ovf) {
			n = FIFO_DEPTH; /* 가득 참 (full: WR == RD) */
		}
		if (n == 0) {
			continue;
		}
		if (i2c_burst_read_dt(&bus, REG_FIFO_DATA, raw, n * 6) != 0) {
			st.i2c_errors++;
			k_msleep(10);
			continue;
		}

		for (uint32_t i = 0; i < n; i++) {
			const uint8_t *r = &raw[i * 6];
			const uint32_t red = sys_get_be24(&r[0]) & SAMPLE_MASK;
			const uint32_t ir = sys_get_be24(&r[3]) & SAMPLE_MASK;

			sys_put_le32(red, &out[i * 8]);
			sys_put_le32(ir, &out[i * 8 + 4]);
		}

		/* 마지막 샘플 시각 (time of the last sample), 위 머리말 참고 */
		if (edge && !ovf && n >= BATCH) {
			t_us += (n - BATCH) * period_us;
		}
		(void)frame_tx_send(FRAME_TYPE_PPG, t_us, idx, out, (uint16_t)(n * 8));

		/* 롤오버 꺼짐 → 넘친 샘플은 FIFO 뒤쪽에서 버려짐 → 번호를 건너뜀 (skip lost indices) */
		idx += n + ovf;
		st.samples += n;
		st.ovf_samples += ovf;
		st.batch_min = MIN(st.batch_min, (uint16_t)n);
		st.batch_max = MAX(st.batch_max, (uint16_t)n);
	}
}

K_THREAD_STACK_DEFINE(ppg_stack, 1536);
static struct k_thread ppg_thread_data;

int max30102_start(void)
{
	int err;

	if (!i2c_is_ready_dt(&bus) || !gpio_is_ready_dt(&irq)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&irq, GPIO_INPUT);
	if (err) {
		return err;
	}
	gpio_init_callback(&irq_cb, int_isr, BIT(irq.pin));
	err = gpio_add_callback_dt(&irq, &irq_cb);
	if (err) {
		return err;
	}
	/* ACTIVE_LOW → "active로 가는 에지" = 하강 에지 (falling edge) */
	err = gpio_pin_interrupt_configure_dt(&irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}
	k_thread_create(&ppg_thread_data, ppg_stack, K_THREAD_STACK_SIZEOF(ppg_stack),
			ppg_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	k_thread_name_set(&ppg_thread_data, "ppg");
	return 0;
}
