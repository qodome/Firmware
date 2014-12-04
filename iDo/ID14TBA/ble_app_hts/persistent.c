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

#define PERSISTENT_PAGE_IDX     ((PSTORAGE_DATA_END_ADDR / PSTORAGE_FLASH_PAGE_SIZE) + 1)

static uint8_t dn_magic[8] = {'D', 'E', 'V', '_', 'N', 'A', 'M', 'E'};
static uint8_t bc_magic[8] = {'B', 'O', 'O', 'T', '_', 'C', 'N', 'T'};

struct persistent_record prcd;
uint8_t pidx;

void persistent_init(void)
{
	pidx = (uint8_t)PERSISTENT_PAGE_IDX;
	HalFlashRead(pidx, 0, (uint8_t *)&prcd, sizeof(prcd));

	if (memcmp(prcd.dn_magic, dn_magic, 8) != 0) {
		memcpy(prcd.dn_magic, dn_magic, 8);
		memset(prcd.device_name, 0, 20);
        prcd.device_name[0] = 'i';
        prcd.device_name[1] = 'D';
        prcd.device_name[2] = 'o';
        prcd.device_name[3] = '2';
        prcd.device_name[4] = 0;
	}

#ifdef DEBUG_STATS
	if (memcmp(prcd.boot_cnt_magic, bc_magic, 8) != 0) {
		memcpy(prcd.boot_cnt_magic, bc_magic, 8);
		prcd.boot_cnt = 1;
	} else {
		prcd.boot_cnt++;
	}
#endif

	HalFlashErase(pidx);
	HalFlashWrite((uint32_t *)((uint32_t)pidx * 1024), (uint8_t *)&prcd, sizeof(prcd));
}

// Max length of device name: 20 (excluding tailing null)
void persistent_set_dev_name(uint8_t *buf, uint16_t len)
{
    uint8_t idx = 0;

    if (len > 20) {
    	len = 20;
    }
    for (idx = 0; idx < 20; idx++) {
    	prcd.device_name[idx] = 0;
    }
    for (idx = 0; idx < len; idx++) {
    	prcd.device_name[idx] = buf[idx];
    }
    HalFlashErase(pidx);
    HalFlashWrite((uint32_t *)((uint32_t)pidx * 1024), (uint8_t *)&prcd, sizeof(prcd));
}

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf)
{
    uint8_t idx = 0;

    for (idx = 0; idx < 20; idx++) {
    	buf[idx] = prcd.device_name[idx];
    }
    buf[idx] = 0;
}

