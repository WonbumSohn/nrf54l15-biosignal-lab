/*
 * 프레임 송신 모듈 구현 (Frame transmit module implementation)
 *
 * 구조 (Design):
 *   생산자(mic, ppg 스레드) → frame_tx_send(): 풀에서 버퍼를 받아 프레임을 만들고 큐에 넣음
 *   송신 스레드: 큐에서 꺼내 uart_tx(DMA) → TX_DONE까지 기다림 → 버퍼 반납
 *   Producers build frames into pool buffers and queue them; one TX thread
 *   sends each with uart_tx (EasyDMA) and frees the buffer on TX_DONE.
 */
#include "frame_tx.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#define HDR_LEN     15
#define CRC_LEN     2
#define BUF_SIZE    ROUND_UP(HDR_LEN + FRAME_MAX_PAYLOAD + CRC_LEN, 4)
#define POOL_COUNT  24

struct tx_item {
	uint8_t *buf;
	uint16_t len;
};

K_MEM_SLAB_DEFINE_STATIC(tx_pool, BUF_SIZE, POOL_COUNT, 4);
K_MSGQ_DEFINE(tx_q, sizeof(struct tx_item), POOL_COUNT, 4);
static K_SEM_DEFINE(tx_done, 0, 1);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart20));
static uint16_t seq_by_type[128];
static atomic_t dropped;
static atomic_t tx_errors;
static struct k_spinlock seq_lock;

uint32_t dev_time_us(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

uint32_t frame_tx_dropped(void)
{
	return (uint32_t)atomic_get(&dropped);
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (evt->type == UART_TX_DONE) {
		k_sem_give(&tx_done);
	} else if (evt->type == UART_TX_ABORTED) {
		atomic_inc(&tx_errors);
		k_sem_give(&tx_done);
	}
}

int frame_tx_send(uint8_t type, uint32_t t_dev_us, uint32_t first_idx,
		  const void *payload, uint16_t len)
{
	uint8_t *f;
	uint16_t seq;
	k_spinlock_key_t key;

	if (len > FRAME_MAX_PAYLOAD || type >= ARRAY_SIZE(seq_by_type)) {
		return -EMSGSIZE;
	}
	if (k_mem_slab_alloc(&tx_pool, (void **)&f, K_NO_WAIT) != 0) {
		atomic_inc(&dropped);
		/* seq는 증가시켜 PC가 손실을 알 수 있게 함 (advance seq so the PC sees a gap) */
		key = k_spin_lock(&seq_lock);
		seq_by_type[type]++;
		k_spin_unlock(&seq_lock, key);
		return -ENOMEM;
	}

	key = k_spin_lock(&seq_lock);
	seq = seq_by_type[type]++;
	k_spin_unlock(&seq_lock, key);

	f[0] = 0xA5;
	f[1] = 0x5A;
	f[2] = type;
	sys_put_le16(len, &f[3]);
	sys_put_le16(seq, &f[5]);
	sys_put_le32(t_dev_us, &f[7]);
	sys_put_le32(first_idx, &f[11]);
	memcpy(&f[HDR_LEN], payload, len);
	sys_put_le16(crc16_itu_t(0, f, HDR_LEN + len), &f[HDR_LEN + len]);

	struct tx_item item = {.buf = f, .len = HDR_LEN + len + CRC_LEN};

	if (k_msgq_put(&tx_q, &item, K_NO_WAIT) != 0) {
		/* 풀 크기 = 큐 크기라 보통 일어나지 않음 (pool size == queue size) */
		k_mem_slab_free(&tx_pool, f);
		atomic_inc(&dropped);
		return -ENOMEM;
	}
	return 0;
}

static void tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	struct tx_item item;

	while (1) {
		k_msgq_get(&tx_q, &item, K_FOREVER);
		if (uart_tx(uart_dev, item.buf, item.len, SYS_FOREVER_US) == 0) {
			k_sem_take(&tx_done, K_FOREVER);
		} else {
			atomic_inc(&tx_errors);
		}
		k_mem_slab_free(&tx_pool, item.buf);
	}
}

K_THREAD_STACK_DEFINE(tx_stack, 1024);
static struct k_thread tx_thread_data;

int frame_tx_init(void)
{
	int err;

	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}
	err = uart_callback_set(uart_dev, uart_cb, NULL);
	if (err) {
		return err;
	}
	/* 송신 스레드는 생산자보다 높은 우선순위 (TX thread above producers) */
	k_thread_create(&tx_thread_data, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack),
			tx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "frame_tx");
	return 0;
}
