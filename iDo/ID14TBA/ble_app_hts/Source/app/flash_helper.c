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

#define CMD_CONCURRENT_MAX  2

uint8_t flash_page_erase_idx[2] = {0};
uint8_t flash_erase_cnt = 0;
uint8_t flash_erase_wait_ack = 0;

uint32_t flash_b_buffer[256/4] = {0};
uint32_t flash_s_buffer[32/4] = {0};
uint32_t *flash_page_write_buf[2] = {NULL, NULL};
uint32_t *flash_page_write_dst[2] = {NULL, NULL};
uint16_t flash_page_write_len[2] = {0};
uint8_t flash_write_cnt = 0;
uint8_t flash_write_wait_ack = 0;

void HalFlashRead(uint8_t page_idx, uint16_t offset, uint8_t *buf, uint16_t len)
{
    uint8_t *ptr = (uint8_t *)((uint32_t)page_idx * (uint32_t)1024 + (uint32_t)offset);

    memcpy(buf, ptr, len);
}

void HalFlashErase(uint8_t page_idx)
{
	if ((page_idx != persistent_flash_page()) &&
			(page_idx < recorder_first_page() || page_idx > recorder_last_page())) {
		persistent_record_error(PERSISTENT_ERROR_FLASH_ERASE, (uint32_t)page_idx);
		return;
	}
    if (flash_erase_cnt >= CMD_CONCURRENT_MAX) {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
        return;
    }
    if (flash_page_erase_idx[flash_erase_cnt] != 0) {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
    flash_page_erase_idx[flash_erase_cnt++] = page_idx;
}

void HalFlashWrite(uint32_t *addr, uint8_t *buf, uint16_t len)
{
	if (((uint8_t)((uint32_t)addr / 1024) != persistent_flash_page()) &&
			((uint8_t)((uint32_t)addr / 1024) < recorder_first_page() || (uint8_t)((uint32_t)addr / 1024) > recorder_last_page())) {
		persistent_record_error(PERSISTENT_ERROR_FLASH_WRITE, (uint32_t)addr);
		return;
	}
    if (flash_write_cnt >= CMD_CONCURRENT_MAX) {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
        return;
    }
    if (flash_page_write_len[flash_write_cnt] != 0) {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
    // Handle big/small buffers
    if (len > 32 && len <= 256) {
        memcpy((uint8_t *)flash_b_buffer, buf, len);
        flash_page_write_buf[flash_write_cnt] = flash_b_buffer;
    } else if (len <= 32) {
        memcpy((uint8_t *)flash_s_buffer, buf, len);
        flash_page_write_buf[flash_write_cnt] = flash_s_buffer;
    } else {
    	flash_page_write_buf[flash_write_cnt] = (uint32_t *)buf;
    }

    flash_page_write_len[flash_write_cnt] = len;
    flash_page_write_dst[flash_write_cnt] = addr;
    flash_write_cnt++;
}

// Notify radio has been turned off
void flash_radio_notification_evt_handler_t(bool radio_active)
{
	uint32_t err_code = NRF_SUCCESS;

    // Finish flash operations
	if (flash_erase_cnt > 0 && flash_erase_wait_ack == 0 && flash_write_wait_ack == 0) {
        err_code = sd_flash_page_erase((uint32_t)flash_page_erase_idx[0]);
        if (err_code == NRF_SUCCESS) {
        	flash_erase_wait_ack = 1;
        }
    } else if (flash_write_cnt > 0 && flash_erase_wait_ack == 0 && flash_write_wait_ack == 0) {
    	err_code = sd_flash_write(flash_page_write_dst[0], flash_page_write_buf[0], (flash_page_write_len[0]/4));
    	if (err_code == NRF_SUCCESS) {
    		flash_write_wait_ack = 1;
    	}
    }

	// Ask battery if need to measure battery level
	battery_on_radio_off_evt();
}

void flash_helper_sys_event(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (flash_erase_wait_ack == 1) {
                flash_erase_wait_ack = 0;
                flash_page_erase_idx[0] = 0;
                flash_erase_cnt--;
                if (flash_erase_cnt > 0) {
                    flash_page_erase_idx[0] = flash_page_erase_idx[1];
                    flash_page_erase_idx[1] = 0;
                }
            } else if (flash_write_wait_ack == 1) {
                flash_write_wait_ack = 0;
                flash_page_write_buf[0] = NULL;
                flash_page_write_dst[0] = NULL;
                flash_page_write_len[0] = 0;
                flash_write_cnt--;
                if (flash_write_cnt > 0) {
                    flash_page_write_buf[0] = flash_page_write_buf[1];
                    flash_page_write_dst[0] = flash_page_write_dst[1];
                    flash_page_write_len[0] = flash_page_write_len[1];
                    flash_page_write_buf[1] = NULL;
                    flash_page_write_dst[1] = NULL;
                    flash_page_write_len[1] = 0;
                }
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}

