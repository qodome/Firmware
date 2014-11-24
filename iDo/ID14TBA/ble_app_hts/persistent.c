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

// Max length of device name: 20 (excluding tailing null)
void persistent_set_dev_name(uint8_t *buf, uint16_t len)
{
    uint8_t idx = 0;
    uint8_t wbuf[8 + 20] = {0};
    uint8_t pidx = (uint8_t)PERSISTENT_PAGE_IDX;

    if (len > 20) {
    	len = 20;
    }

    for (idx = 0; idx < 28; idx++) {
        wbuf[idx] = 0;
    }
    HalFlashErase(pidx);

    memcpy(wbuf, dn_magic, 8);
    memcpy((wbuf + 8), buf, len);
    HalFlashWrite((uint32_t *)((uint32_t)pidx * 1024), wbuf, 28);
}

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf)
{
    uint8_t rbuf[8 + 20];
    uint8_t idx = 0;
    uint8_t pidx = (uint8_t)PERSISTENT_PAGE_IDX;

    HalFlashRead(pidx, 0, rbuf, 28);
    if (memcmp(rbuf, dn_magic, 8) == 0) {
        for (idx = 8; idx < 28; idx++) {
            buf[idx - 8] = rbuf[idx];
        }
        buf[20] = 0;
    } else {
        buf[0] = 'i';
        buf[1] = 'D';
        buf[2] = 'o';
        buf[3] = 0;
    }
}
