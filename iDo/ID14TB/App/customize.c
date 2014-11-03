/*
 * Customize
 */
#include "customize.h"
#include "hal_flash.h"
#include "OSAL.h"

static uint8 dn_magic[8] = {'D', 'E', 'V', '_', 'N', 'A', 'M', 'E'};
static uint8 dn_default[3] = {'i', 'D', 'o'};

// FIXME: currently only device name
void custom_init(void)
{
    uint8 buf[CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN];
    uint8 idx = 0;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, CUSTOM_DN_MGIC_OFFSET, buf, (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN));
    if (osal_memcmp(buf, dn_magic, CUSTOM_DN_MAGIC_LEN) != TRUE) {
        HalFlashErase(CUSTOM_DN_PAGE_IDX);
        for (idx = 0; idx < (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN); idx++) {
            buf[idx] = 0;
        }
        osal_memcpy(buf, dn_magic, CUSTOM_DN_MAGIC_LEN);
        osal_memcpy((buf + CUSTOM_DN_MAGIC_LEN), dn_default, 3);
        HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048 + CUSTOM_DN_MGIC_OFFSET) / 4), buf, (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN) / 4);
    }
}

// Input buffer at least 21 bytes (couting tailing zero)
void custom_get_dn(uint8 *dn)
{
    uint8 buf[CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN];
    uint8 idx = 0;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, CUSTOM_DN_MGIC_OFFSET, buf, (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN));
    if (osal_memcmp(buf, dn_magic, CUSTOM_DN_MAGIC_LEN) == TRUE) {
        for (idx = CUSTOM_DN_MAGIC_LEN; idx < (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN); idx++) {
            dn[idx - CUSTOM_DN_MAGIC_LEN] = buf[idx];
        }
        dn[CUSTOM_DN_LEN] = 0;
    } else {
        dn[0] = 'i';
        dn[1] = 'D';
        dn[2] = 'o';
        dn[3] = 0;
    }
}

// Input buffer at least 21 bytes (couting tailing zero)
void custom_set_dn(uint8 *dn)
{
    uint8 idx = 0;
    uint8 buf[CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN] = {0};

    for (idx = 0; idx < (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN); idx++) {
        buf[idx] = 0;
    }
    dn[CUSTOM_DN_LEN] = 0;
    HalFlashErase(CUSTOM_DN_PAGE_IDX);

    osal_memcpy(buf, dn_magic, CUSTOM_DN_MAGIC_LEN);
    osal_memcpy((buf + CUSTOM_DN_MAGIC_LEN), dn, osal_strlen((char *)dn));
    HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048 + CUSTOM_DN_MGIC_OFFSET) / 4), buf, (CUSTOM_DN_MAGIC_LEN + CUSTOM_DN_LEN) / 4); 
}

