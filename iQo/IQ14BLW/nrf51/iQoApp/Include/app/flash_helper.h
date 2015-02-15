/*
 * Helper function to handle flash erase/write requests
 */
#ifndef __FLASH_HELPER__
#define __FLASH_HELPER__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define WRITE_BUFFER_MAX_SIZE			256

struct flash_op_cache {
	uint8_t is_write;					// 0 - erase; 1 - write
	uint8_t pg_idx;
	uint16_t pg_offset;
	uint16_t len;
	uint16_t pad;
	uint8_t buf[WRITE_BUFFER_MAX_SIZE];
};

void HalFlashErase(uint8_t page_idx);
void HalFlashRead(uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len);
void HalFlashWrite(uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len);
uint8_t flash_queue_size(void);
void flash_radio_notification_evt_handler_t(bool radio_active);
void flash_helper_sys_event(uint32_t sys_evt);

#endif
