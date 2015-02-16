/*
 * Helper function to handle flash erase/write requests
 */
#include <stdlib.h>
#include <string.h>
#include "flash_helper.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "battery.h"
#include "persistent.h"
#include "recorder.h"
#include "nrf_types.h"
#include "app_scheduler.h"

#define OP_CACHE_MAX			9

typedef enum {
	IDLE = 0,
	IN_PROGRESS,
	BUSY_WAIT,
	FAILED
} OPC_HEAD_STATUS_T;

struct flash_op_cache opc[OP_CACHE_MAX];
uint8_t opc_begin = 0;
uint8_t opc_end = 0;
uint8_t opc_queue_depth = 0;
uint8_t opc_max_queue_depth = 0;
OPC_HEAD_STATUS_T opc_head_status = IDLE;

uint16_t opc_queue_overflow_cnt = 0;
uint16_t flash_interrupt_schedule_fail_cnt = 0;
uint16_t flash_sys_evt_error_cnt = 0;

static uint8_t __HalFlashValidateReq(uint8_t page_idx)
{
	if ((page_idx != persistent_flash_first_page()) && (page_idx != (persistent_flash_first_page() + 1))) {
		persistent_record_error(PERSISTENT_ERROR_FLASH_ERASE, (uint32_t)page_idx);
		return FAIL;
	} else {
		return SUCCESS;
	}
}

// Since all tasks are invoked from scheduler, lock protection is not necessary
static uint8_t __HalFlashQueueTask(uint8_t is_write, uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len)
{
	uint16_t idx;

	if (((opc_end + 1) % OP_CACHE_MAX) == opc_begin) {
		opc_queue_overflow_cnt++;
		return FAIL;
	}

	opc[opc_end].is_write = is_write;
	opc[opc_end].pg_idx = page_idx;
	opc[opc_end].pg_offset = offset;
	opc[opc_end].len = len;
	for (idx = 0; idx < len; idx++) {
		opc[opc_end].buf[idx] = buf[idx];
	}
	opc_end = (opc_end + 1) % OP_CACHE_MAX;
	opc_queue_depth++;
	if (opc_queue_depth > opc_max_queue_depth) {
		opc_max_queue_depth = opc_queue_depth;
	}
	return SUCCESS;
}

static void __HalFlashRemoveHead(void)
{
	opc_begin = (opc_begin + 1) % OP_CACHE_MAX;
	opc_head_status = IDLE;
	opc_queue_depth--;
}

static void __HalFlashShootHead(void)
{
	uint32_t ret;
	uint32_t info;

	if (opc[opc_begin].is_write == 1) {
		ret = sd_flash_write((uint32_t *)((uint32_t)opc[opc_begin].pg_idx * 1024 + (uint32_t)opc[opc_begin].pg_offset), (uint32_t *)(opc[opc_begin].buf), (opc[opc_begin].len / 4));
	} else {
		ret = sd_flash_page_erase((uint32_t)opc[opc_begin].pg_idx);
	}
	if (ret == NRF_SUCCESS) {
		opc_head_status = IN_PROGRESS;
	} else if (ret == NRF_ERROR_BUSY) {
		// Not good time
		opc_head_status = BUSY_WAIT;
	} else {
		info = (uint32_t)opc[opc_begin].is_write << 24 | (uint32_t)opc[opc_begin].pg_idx << 16 | (uint32_t)opc[opc_begin].pg_offset;
		__HalFlashRemoveHead();
		persistent_record_error((uint8_t)ret, info);
	}
}

void HalFlashRead(uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len)
{
    uint8_t *ptr = (uint8_t *)((uint32_t)page_idx * (uint32_t)1024 + (uint32_t)offset);

    memcpy(buf, ptr, len);
}

void HalFlashErase(uint8_t page_idx)
{
	uint8_t is_head;

	if (__HalFlashValidateReq(page_idx) == FAIL) {
		return;
	}

	is_head = (opc_begin == opc_end) ? 1 : 0;
	if (__HalFlashQueueTask(0, page_idx, 0, NULL, 0) == FAIL) {
		return;
	}
	if (is_head == 1) {
		__HalFlashShootHead();
	}
}

void HalFlashWrite(uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len)
{
	uint8_t is_head;

	if (__HalFlashValidateReq(page_idx) == FAIL) {
		return;
	}

	if (len > WRITE_BUFFER_MAX_SIZE) {
		persistent_record_error(NRF_ERROR_DATA_SIZE, (uint32_t)len);
	}

	is_head = (opc_begin == opc_end) ? 1 : 0;
	if (__HalFlashQueueTask(1, page_idx, offset, buf, len) == FAIL) {
		return;
	}
	if (is_head == 1) {
		__HalFlashShootHead();
	}
}

uint8_t flash_queue_size(void)
{
	return opc_queue_depth;
}

void flash_time_to_shoot(void * p_event_data , uint16_t event_size)
{
	// We need to trigger those failed operation,
	// which are marked FAILED by flash_helper_sys_event()
	if (opc_head_status == FAILED || opc_head_status == BUSY_WAIT) {
		__HalFlashShootHead();
	}
}

// This event handler is invoked from interrupt context, need to do schedule!
void flash_radio_notification_evt_handler_t(bool radio_active)
{
    // Schedule next flash operations
	if (app_sched_event_put(NULL, 0, flash_time_to_shoot) != NRF_SUCCESS) {
		flash_interrupt_schedule_fail_cnt++;
	}
}

// This function will be called in app_scheduler
void flash_helper_sys_event(uint32_t sys_evt)
{
	switch(sys_evt)
	{
	case NRF_EVT_FLASH_OPERATION_SUCCESS:
		if (opc_head_status != IN_PROGRESS) {
			flash_sys_evt_error_cnt++;
			break;
		}
		__HalFlashRemoveHead();
		if (opc_begin != opc_end) {
			__HalFlashShootHead();
		}
		break;

	case NRF_EVT_FLASH_OPERATION_ERROR:
		if (opc_head_status != IN_PROGRESS) {
			flash_sys_evt_error_cnt++;
		}
		opc_head_status = FAILED;
		break;

	default:
		// No implementation needed.
		break;
	}
}
