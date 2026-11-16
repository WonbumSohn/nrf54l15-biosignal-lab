/*
 * SPH0645 I2S 마이크 수신 + decimation (I2S capture and decimation)
 *
 * 클럭 (Clock) — nRF54L15 PS 8.11.5, Table 42:
 *   MCK = 32 MHz / N (정수 분주), LRCK = MCK / RATIO, SCK = 2 × LRCK × 32 = 64 × LRCK.
 *   요청 33 333 Hz → 드라이버가 N=15, RATIO=64 선택 → LRCK 33 333.33 Hz, SCK 2.1333 MHz
 *   (SPH0645 사양 SCK 2.048–4.096 MHz 안). 정확히 32 kHz는 만들 수 없음 (N·RATIO=1000 불가).
 *
 * 데이터 (Data) — SPH0645 datasheet:
 *   I2S 형식, 32 SCK/half-frame, MSB 먼저, 24비트 2의 보수 중 상위 18비트 유효.
 *   SEL=GND → 왼쪽(LRCK low) 채널. 오른쪽 구간은 high-Z (+100 kΩ 풀다운 → 0 근처).
 *   18비트 값: (int32_t)word >> 14  ⚠️ 진단 프레임의 비트 마스크로 정렬 확인
 *
 * 출력 (Output frames):
 *   MIC (type 2): int16 샘플들. 기본 = decimated 8 333.33 Hz, MIC_RAW=1 이면 33 333.33 Hz 원본(>>2)
 *   MIC_DIAG (0x7E): 4블록(약 31 ms)마다 통계·비트 마스크·원본 워드·레지스터 값
 */
#include "mic.h"
#include "frame_tx.h"
#include "decim_coeffs.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/byteorder.h>
#include <nrfx.h>
#include <arm_math.h>

#ifndef MIC_RAW
#define MIC_RAW 0
#endif

#define I2S_FS_REQ       33333                 /* → 33 333.33 Hz 실제 (actual) */
#define BLOCK_FRAMES     256                   /* 스테레오 프레임 / stereo frames per block (7.68 ms) */
#define BLOCK_WORDS      (BLOCK_FRAMES * 2)
#define BLOCK_BYTES      (BLOCK_WORDS * sizeof(uint32_t))
#define N_BLOCKS         8
#define DISCARD_BLOCKS   8                     /* 시작 후 약 61 ms 버림 (≥ 50 ms, datasheet) */
#define DIAG_EVERY       4                     /* 진단 프레임 주기 (blocks) */
#define OUT_PER_BLOCK    (BLOCK_FRAMES / DECIM_FACTOR) /* 64 */

BUILD_ASSERT(BLOCK_FRAMES % DECIM_FACTOR == 0, "block must be a multiple of DECIM_FACTOR");

K_MEM_SLAB_DEFINE_STATIC(i2s_slab, BLOCK_BYTES, N_BLOCKS, 4);

static const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s20));

/* decimation */
static arm_fir_decimate_instance_f32 decim;
static float32_t decim_state[DECIM_NUM_TAPS + BLOCK_FRAMES - 1];
static float32_t xin[BLOCK_FRAMES];
static float32_t yout[OUT_PER_BLOCK];
static int16_t out16[BLOCK_FRAMES];

static uint32_t i2s_errors;

/* 진단 누적값 (diagnostic accumulators) */
struct diag_acc {
	int32_t l_min;
	int32_t l_max;
	int64_t l_sum;
	uint32_t l_or;
	uint32_t l_and;
	uint32_t r_or;
	uint32_t r_and;
	uint32_t n;
};

static void diag_reset(struct diag_acc *d)
{
	d->l_min = INT32_MAX;
	d->l_max = INT32_MIN;
	d->l_sum = 0;
	d->l_or = 0;
	d->l_and = UINT32_MAX;
	d->r_or = 0;
	d->r_and = UINT32_MAX;
	d->n = 0;
}

static int16_t sat16(float32_t v)
{
	if (v > 32767.0f) {
		return 32767;
	}
	if (v < -32768.0f) {
		return -32768;
	}
	return (int16_t)v;
}

static void send_diag(const struct diag_acc *d, const uint32_t *last_words)
{
	uint8_t p[80];
	uint8_t *q = p;

	*q++ = 1;              /* version */
	*q++ = MIC_RAW ? 1 : 0; /* mode: 0 = decimated, 1 = raw */
	sys_put_le16((uint16_t)MIN(d->n, UINT16_MAX), q); q += 2;
	sys_put_le32((uint32_t)d->l_min, q); q += 4;
	sys_put_le32((uint32_t)d->l_max, q); q += 4;
	sys_put_le32((uint32_t)(int32_t)(d->n ? d->l_sum / (int64_t)d->n : 0), q); q += 4;
	sys_put_le32(d->l_or, q); q += 4;
	sys_put_le32(d->l_and, q); q += 4;
	sys_put_le32(d->r_or, q); q += 4;
	sys_put_le32(d->r_and, q); q += 4;
	for (int i = 0; i < 8; i++) {
		sys_put_le32(last_words[i], q); q += 4;
	}
	sys_put_le32(i2s_errors, q); q += 4;
	sys_put_le32(frame_tx_dropped(), q); q += 4;
	/* 드라이버가 실제로 고른 클럭 설정 (clock settings actually programmed) */
	sys_put_le32(NRF_I2S20->CONFIG.MCKFREQ, q); q += 4;
	sys_put_le32(NRF_I2S20->CONFIG.RATIO, q); q += 4;

	(void)frame_tx_send(FRAME_TYPE_MIC_DIAG, dev_time_us(), 0, p, (uint16_t)(q - p));
}

static int i2s_setup_and_start(void)
{
	struct i2s_config cfg = {
		.word_size = 32,
		.channels = 2,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = I2S_FS_REQ,
		.mem_slab = &i2s_slab,
		.block_size = BLOCK_BYTES,
		.timeout = 1000,
	};
	int err = i2s_configure(i2s_dev, I2S_DIR_RX, &cfg);

	if (err) {
		return err;
	}
	return i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
}

static void mic_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct diag_acc d;
	uint32_t last_words[8] = {0};
	uint32_t blocks = 0;
	uint32_t in_idx = 0;   /* 원본 샘플 누적 번호 (raw sample index) */
	uint32_t out_idx = 0;  /* decimated 샘플 누적 번호 */

	diag_reset(&d);

	while (1) {
		void *blk;
		size_t size;
		int err = i2s_read(i2s_dev, &blk, &size);

		if (err) {
			/* 오버런 등 → 멈추고 다시 시작 (overrun etc. → drop and restart) */
			i2s_errors++;
			(void)i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
			k_msleep(10);
			if (i2s_setup_and_start() != 0) {
				k_msleep(500);
			}
			blocks = 0;
			continue;
		}

		const uint32_t t_us = dev_time_us(); /* 블록 끝 ≈ 마지막 샘플 시각 */
		const uint32_t *w = blk;
		const uint32_t n = size / (2 * sizeof(uint32_t));

		blocks++;
		if (blocks <= DISCARD_BLOCKS || n != BLOCK_FRAMES) {
			k_mem_slab_free(&i2s_slab, blk);
			continue;
		}

		memcpy(last_words, w, sizeof(last_words));
		for (uint32_t i = 0; i < n; i++) {
			const uint32_t wl = w[2 * i];
			const uint32_t wr = w[2 * i + 1];
			const int32_t v = ((int32_t)wl) >> 14; /* 18비트 부호 있는 값 */

			d.l_min = MIN(d.l_min, v);
			d.l_max = MAX(d.l_max, v);
			d.l_sum += v;
			d.l_or |= wl;
			d.l_and &= wl;
			d.r_or |= wr;
			d.r_and &= wr;
#if MIC_RAW
			out16[i] = (int16_t)(v >> 2);
#else
			xin[i] = (float32_t)v * (1.0f / 131072.0f);
#endif
		}
		d.n += n;
		k_mem_slab_free(&i2s_slab, blk);

#if MIC_RAW
		(void)frame_tx_send(FRAME_TYPE_MIC, t_us, in_idx, out16, n * sizeof(int16_t));
#else
		arm_fir_decimate_f32(&decim, xin, yout, BLOCK_FRAMES);
		for (uint32_t i = 0; i < OUT_PER_BLOCK; i++) {
			out16[i] = sat16(yout[i] * 32768.0f);
		}
		(void)frame_tx_send(FRAME_TYPE_MIC, t_us, out_idx, out16,
				    OUT_PER_BLOCK * sizeof(int16_t));
		out_idx += OUT_PER_BLOCK;
#endif
		in_idx += n;

		if ((blocks % DIAG_EVERY) == 0) {
			send_diag(&d, last_words);
			diag_reset(&d);
		}
	}
}

K_THREAD_STACK_DEFINE(mic_stack, 2048);
static struct k_thread mic_thread_data;

int mic_start(void)
{
	int err;

	if (!device_is_ready(i2s_dev)) {
		return -ENODEV;
	}
	if (arm_fir_decimate_init_f32(&decim, DECIM_NUM_TAPS, DECIM_FACTOR, decim_coeffs,
				      decim_state, BLOCK_FRAMES) != ARM_MATH_SUCCESS) {
		return -EINVAL;
	}
	err = i2s_setup_and_start();
	if (err) {
		return err;
	}
	k_thread_create(&mic_thread_data, mic_stack, K_THREAD_STACK_SIZEOF(mic_stack),
			mic_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&mic_thread_data, "mic");
	return 0;
}
