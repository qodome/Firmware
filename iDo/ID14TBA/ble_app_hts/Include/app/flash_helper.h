/*
 * Helper function to handle flash erase/write requests
 */
#ifndef __FLASH_HELPER__
#define __FLASH_HELPER__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define FLASH_WRITE_PAGE_LOW
#define FLASH_WRITE_PAGE_HIGH

void HalFlashErase(uint8_t page_idx);
void HalFlashRead(uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len);
void HalFlashWrite(uint32_t *addr, uint8_t *buf, uint16_t len);
void flash_radio_notification_evt_handler_t(bool radio_active);
void flash_helper_sys_event(uint32_t sys_evt);

#endif
