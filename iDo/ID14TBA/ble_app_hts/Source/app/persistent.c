/*
 * Customized persistent data
 */
#include "persistent.h"
#include "flash_helper.h"
#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_assert.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "pstorage_platform.h"
#include "ble_flash.h"

#define PERSISTENT_PAGE_IDX     		((PSTORAGE_DATA_END_ADDR / PSTORAGE_FLASH_PAGE_SIZE) + 1)
#define _PERSISTENT_BACKUP_PAGE_IDX_	(PERSISTENT_PAGE_IDX + 1)

static uint8_t PERSISTENT_MAGIC[4] = {0xDE, 0xAD, 0xBE, 0xEF};

uint8_t pidx;
uint8_t boot_cnt;
uint16_t __debug_sizeof_persistent_page = 0;

uint8_t persistent_flash_first_page(void)
{
	return (uint8_t)PERSISTENT_PAGE_IDX;
}

static uint8 __persistent_mark_bit_map(uint8_t *buf, uint8_t len)
{
	uint8_t i, j, flag, count = 0;

	if (len >= 32) {
		return 0xFF;
	}

	for (i = 0; i < len; i++) {
		flag = 0;
		for (j = 0; j < 8; j++) {
			if ((buf[i] & (1 << j)) == (1 << j)) {
				buf[i] &= ~(1 << j);
				flag = 1;
				break;
			} else {
				count++;
			}
		}
		if (flag == 1) {
			break;
		}
	}

	return count;
}

static uint8_t __persistent_get_idx_from_bit_map(uint8_t *buf, uint8_t len)
{
	return 0;
}

// FIXME: check the backup page status
void persistent_init(void)
{
	uint8_t i;
	uint8_t buf[16];
	uint32_t magic32 = 0xEFBEADDE;		// reverse DEADBEEF
	uint32_t ido2Name1 = 0x326F4469;	// reverse iDo2
	uint32_t ido2Name2 = 0x00000000;	// string end
	uint32_t tmp32;

	__debug_sizeof_persistent_page = sizeof(struct persistent_page);

	pidx = (uint8_t)PERSISTENT_PAGE_IDX;
	HalFlashRead(pidx, 0, buf, 4);

	if (memcmp(buf, PERSISTENT_MAGIC, 4) != 0) {
		// This is invoked before softdevice and radio
		ble_flash_page_erase(pidx);
		flash_word_unprotected_write((uint32_t *)((uint32_t)pidx * 1024), magic32);
		flash_word_unprotected_write((uint32_t *)((uint32_t)pidx * 1024 + 4), ido2Name1);
		flash_word_unprotected_write((uint32_t *)((uint32_t)pidx * 1024 + 8), ido2Name2);
	}

	// Update boot count
	HalFlashRead(pidx, 24, buf, 16);
	boot_cnt = __persistent_mark_bit_map(buf, 16);

	// Write back
	for (i = 0; i < 4; i++) {
		tmp32 = ((uint32_t)buf[i * 4 + 3] << 24) | ((uint32_t)buf[i * 4 + 2] << 16) | ((uint32_t)buf[i * 4 + 1] << 8) | (uint32_t)buf[i * 4];
		flash_word_unprotected_write((uint32_t *)((uint32_t)pidx * 1024 + 24 + i * 4), tmp32);
	}
}

/*
 * Please make sure flash_helper's queue is clean!
 */
// Max length of device name: 20 (excluding tailing null)
// This function prepare contents for the backup page
void persistent_set_dev_name_prepare(uint8_t *buf, uint16_t len)
{
	uint32_t buf32[64];
	uint8_t *ptr8;

	HalFlashErase(pidx + 1);
	HalFlashRead(pidx, 0, (uint8_t *)buf32, 256);
	buf32[0] = 0xFFFFADDE;
	if (len > 20) {
		len = 20;
	}
	ptr8 = (uint8_t *)&(buf32[1]);
	memcpy(ptr8, buf, len);
	HalFlashWrite((pidx + 1), 0, (uint8_t *)buf32, 256);
	HalFlashRead(pidx, 256, (uint8_t *)buf32, 256);
	HalFlashWrite((pidx + 1), 256, (uint8_t *)buf32, 256);
	HalFlashRead(pidx, 512, (uint8_t *)buf32, 256);
	HalFlashWrite((pidx + 1), 512, (uint8_t *)buf32, 256);
	HalFlashRead(pidx, 768, (uint8_t *)buf32, 256);
	HalFlashWrite((pidx + 1), 768, (uint8_t *)buf32, 256);
	buf32[0] = 0xEFBEADDE;
	HalFlashWrite((pidx + 1), 0, (uint8_t *)buf32, 4);
}

/*
 * Please make sure flash_helper's queue is clean!
 */
// This function does the persistent page overwrite
void persistent_set_dev_name_finish(void)
{
	uint32_t buf32[64];

	HalFlashErase(pidx);
	HalFlashRead((pidx + 1), 0, (uint8_t *)buf32, 256);
	buf32[0] = 0xFFFFADDE;
	HalFlashWrite(pidx, 0, (uint8_t *)buf32, 256);
	HalFlashRead((pidx + 1), 256, (uint8_t *)buf32, 256);
	HalFlashWrite(pidx, 256, (uint8_t *)buf32, 256);
	HalFlashRead((pidx + 1), 512, (uint8_t *)buf32, 256);
	HalFlashWrite(pidx, 512, (uint8_t *)buf32, 256);
	HalFlashRead((pidx + 1), 768, (uint8_t *)buf32, 256);
	HalFlashWrite(pidx, 768, (uint8_t *)buf32, 256);
	buf32[0] = 0xFFBEADDE;
	HalFlashWrite(pidx, 0, (uint8_t *)buf32, 4);
	HalFlashErase(pidx + 1);
	buf32[0] = 0xEFBEADDE;
	HalFlashWrite(pidx, 0, (uint8_t *)buf32, 4);
}

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf)
{
	uint8_t buf_flash[20];
    uint8_t idx;

    HalFlashRead(pidx, 4, buf_flash, 20);
    for (idx = 0; idx < 20; idx++) {
    	buf[idx] = buf_flash[idx];
    }
    buf[idx] = 0;
}

void persistent_record_error(uint8_t error_idx, uint32_t error_info)
{
	uint8_t idx;
	uint32_t offset;
	struct error_record rcd;

	if (error_idx > PERSISTENT_ERROR_MAX) {
		persistent_record_error(PERSISTENT_ERROR_INTERNAL, 0);
		return;
	}

	offset = (uint32)&(((struct persistent_page *)0)->error_log[error_idx]);
	HalFlashRead(pidx, (uint16_t)offset, (uint8_t *)&rcd, sizeof(rcd));
	idx = __persistent_mark_bit_map((uint8_t *)&(rcd.count_bit), 4);
	if (idx < PERSISTENT_ERROR_ENTRY) {
		rcd.error_info[idx] = error_info;
	}

	// Write back
	HalFlashWrite(pidx, (uint16_t)offset, (uint8_t *)&rcd, sizeof(rcd));
}


void persistent_pwrmgmt_set_latest(struct pwrmgmt_data *pwr_ptr)
{

}

uint8_t persistent_pwrmgmt_get_latest(struct pwrmgmt_data *pwr_ptr)
{

}
