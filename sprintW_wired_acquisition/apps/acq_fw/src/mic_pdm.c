/*
 * 내장 PDM 마이크 수신 + decimation (On-board PDM capture and decimation)
 *
 * 클럭 (Clock) — nRF54L15 PDM, PCLK32M:
 *   PDM CLK = 32 MHz / PRESCALER, fs = PDM CLK / RATIO.
 *   요청 16 667 Hz, CLK 1.2–3.2 MHz → nrfx가 PRESCALER 24, RATIO 80 선택
 *   → CLK 1.3333 MHz, fs 16 666.67 Hz (진단 프레임에 실제 레지스터 값 포함)
 *   Requested 16 667 Hz → PRESCALER 24, RATIO 80 (registers echoed in diag frames)
 *
 * 채널 (Channel):
 *   마이크의 L/R 선택 핀 연결을 몰라 스테레오로 받고 L·R 통계를 모두 보냄.
 *   출력 채널은 PDM_OUT_CH (0 = L, 1 = R). 신호가 없는 쪽은 진단 값으로 확인.
 *   Capture stereo, report stats for both edges; PDM_OUT_CH selects the output.
 *
 * 출력 (Output frames):
 *   MIC_PDM (type 5): int16 128개, 8 333.33 Hz, first_idx = 출력 샘플 번호
 *   PDM_DIAG (0x7D): 4블록(약 61 ms)마다 L/R 최소·최대·평균, 오류, 레지스터
 */
#include "mic_pdm.h"
#include "frame_tx.h"
#include "decim_coeffs_pdm.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/sys/byteorder.h>
#include <nrfx.h>
#include <arm_math.h>

#ifndef PDM_OUT_CH
#define PDM_OUT_CH 0 /* 0 = L (falling edge), 1 = R */
#endif

#define PDM_FS_REQ       16667                 /* → 16 666.67 Hz 실제 (actual) */
#define PDM_CLK_MIN      1200000               /* 저전력 모드(<1 MHz) 피함 / avoid low-power mode */
#define PDM_CLK_MAX      3200000
#define BLOCK_FRAMES     256                   /* 스테레오 프레임 / stereo frames (15.36 ms) */
#define BLOCK_SAMPLES    (BLOCK_FRAMES * 2)
#define BLOCK_BYTES      (BLOCK_SAMPLES * sizeof(int16_t))
#define N_BLOCKS         8
#define DISCARD_BLOCKS   8                     /* 시작 후 약 123 ms 버림 (mic + filter settling) */
#define DIAG_EVERY       4
#define READ_TIMEOUT_MS  200
#define OUT_PER_BLOCK    (BLOCK_FRAMES / PDM_DECIM_FACTOR) /* 128 */

BUILD_ASSERT(BLOCK_FRAMES % PDM_DECIM_FACTOR == 0, "block must be a multiple of the factor");
BUILD_ASSERT(OUT_PER_BLOCK * sizeof(int16_t) <= FRAME_MAX_PAYLOAD, "frame too large");

K_MEM_SLAB_DEFINE_STATIC(pdm_slab, BLOCK_BYTES, N_BLOCKS, 4);

static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm20));

static arm_fir_decimate_instance_f32 decim;
static float32_t decim_state[PDM_DECIM_NUM_TAPS + BLOCK_FRAMES - 1];
static float32_t xin[BLOCK_FRAMES];
static float32_t yout[OUT_PER_BLOCK];
static int16_t out16[OUT_PER_BLOCK];

static uint32_t pdm_errors;

struct ch_acc {
	int32_t min;
	int32_t max;
	int64_t sum;
};

struct pdm_diag {
	struct ch_acc ch[2];
	uint32_t n;
};

static void diag_reset(struct pdm_diag *d)
{
	for (int c = 0; c < 2; c++) {
		d->ch[c].min = INT32_MAX;
		d->ch[c].max = INT32_MIN;
		d->ch[c].sum = 0;
	}
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

static void send_diag(const struct pdm_diag *d)
{
	uint8_t p[48];
	uint8_t *q = p;

	*q++ = 1;          /* version */
	*q++ = PDM_OUT_CH; /* 출력 채널 (output channel) */
	sys_put_le16((uint16_t)MIN(d->n, UINT16_MAX), q); q += 2;
	for (int c = 0; c < 2; c++) {
		sys_put_le32((uint32_t)d->ch[c].min, q); q += 4;
		sys_put_le32((uint32_t)d->ch[c].max, q); q += 4;
		sys_put_le32((uint32_t)(int32_t)(d->n ? d->ch[c].sum / (int64_t)d->n : 0), q);
		q += 4;
	}
	sys_put_le32(pdm_errors, q); q += 4;
	sys_put_le32(frame_tx_dropped(), q); q += 4;
	/* 드라이버가 실제로 쓴 레지스터 (registers actually programmed) */
	sys_put_le32(NRF_PDM20->PRESCALER, q); q += 4;
	sys_put_le32(NRF_PDM20->RATIO, q); q += 4;
	sys_put_le32(NRF_PDM20->GAINL, q); q += 4;

	(void)frame_tx_send(FRAME_TYPE_PDM_DIAG, dev_time_us(), 0, p, (uint16_t)(q - p));
}

static int pdm_setup_and_start(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_rate = PDM_FS_REQ,
		.pcm_width = 16,
		.block_size = BLOCK_BYTES,
		.mem_slab = &pdm_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = PDM_CLK_MIN,
			.max_pdm_clk_freq = PDM_CLK_MAX,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = 2,
			.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
					   dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT),
		},
	};
	int err = dmic_configure(dmic_dev, &cfg);

	if (err) {
		return err;
	}
	return dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
}

static void mic_pdm_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct pdm_diag d;
	uint32_t blocks = 0;
	uint32_t out_idx = 0; /* decimated 샘플 누적 번호 (output sample index) */

	diag_reset(&d);

	while (1) {
		void *blk;
		size_t size;
		int err = dmic_read(dmic_dev, 0, &blk, &size, READ_TIMEOUT_MS);

		if (err) {
			/* 시간 초과·오버런 → 멈추고 다시 시작 (timeout/overrun → restart) */
			pdm_errors++;
			(void)dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
			k_msleep(20);
			if (pdm_setup_and_start() != 0) {
				k_msleep(500);
			}
			blocks = 0;
			continue;
		}

		const uint32_t t_us = dev_time_us(); /* 블록 끝 ≈ 마지막 샘플 시각 */
		const int16_t *s = blk;
		const uint32_t n = size / (2 * sizeof(int16_t));

		blocks++;
		if (blocks <= DISCARD_BLOCKS || n != BLOCK_FRAMES) {
			k_mem_slab_free(&pdm_slab, blk);
			continue;
		}

		for (uint32_t i = 0; i < n; i++) {
			for (int ch = 0; ch < 2; ch++) {
				const int32_t v = s[2 * i + ch];

				d.ch[ch].min = MIN(d.ch[ch].min, v);
				d.ch[ch].max = MAX(d.ch[ch].max, v);
				d.ch[ch].sum += v;
			}
			xin[i] = (float32_t)s[2 * i + PDM_OUT_CH] * (1.0f / 32768.0f);
		}
		d.n += n;
		k_mem_slab_free(&pdm_slab, blk);

		arm_fir_decimate_f32(&decim, xin, yout, BLOCK_FRAMES);
		for (uint32_t i = 0; i < OUT_PER_BLOCK; i++) {
			out16[i] = sat16(yout[i] * 32768.0f);
		}
		(void)frame_tx_send(FRAME_TYPE_MIC_PDM, t_us, out_idx, out16,
				    OUT_PER_BLOCK * sizeof(int16_t));
		out_idx += OUT_PER_BLOCK;

		if ((blocks % DIAG_EVERY) == 0) {
			send_diag(&d);
			diag_reset(&d);
		}
	}
}

K_THREAD_STACK_DEFINE(mic_pdm_stack, 2048);
static struct k_thread mic_pdm_thread_data;

int mic_pdm_start(void)
{
	int err;

	if (!device_is_ready(dmic_dev)) {
		return -ENODEV;
	}
	if (arm_fir_decimate_init_f32(&decim, PDM_DECIM_NUM_TAPS, PDM_DECIM_FACTOR,
				      pdm_decim_coeffs, decim_state,
				      BLOCK_FRAMES) != ARM_MATH_SUCCESS) {
		return -EINVAL;
	}
	err = pdm_setup_and_start();
	if (err) {
		return err;
	}
	k_thread_create(&mic_pdm_thread_data, mic_pdm_stack, K_THREAD_STACK_SIZEOF(mic_pdm_stack),
			mic_pdm_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&mic_pdm_thread_data, "mic_pdm");
	return 0;
}
