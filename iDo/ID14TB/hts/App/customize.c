/*
 * Customize
 */
#include "customize.h"
#include "hal_flash.h"
#include "OSAL.h"

static uint8 DN_MAGIC[CUSTOM_DN_MAGIC_LEN] = {'D', 'E', 'V', '_', 'N', 'A', 'M', 'E'};
static uint8 MGMT_MAGIC[MGMT_MAGIC_LEN] = {'M', 'G', 'M', 'T', '_', 'B', 'A', 'T'};
static uint8 DN_DEFAULT[3] = {'i', 'D', 'o'};

// FIXME: currently only device name
void custom_init(void)
{
    struct persistent_data pdata;
    uint8 idx = 0;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    if (osal_memcmp(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN) != TRUE) {
        for (idx = 0; idx < CUSTOM_DN_MAGIC_LEN; idx++) {
            pdata.dn_magic[idx] = 0;
        }
        for (idx = 0; idx < CUSTOM_DN_LEN; idx++) {
            pdata.dn[idx] = 0;
        }
        osal_memcpy(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN);
        osal_memcpy(pdata.dn, DN_DEFAULT, 3);

        HalFlashErase(CUSTOM_DN_PAGE_IDX);
        HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) / 4), (uint8 *)&pdata, sizeof(pdata) / 4);
    }
}

// Input buffer at least 21 bytes (couting tailing zero)
void custom_get_dn(uint8 *dn)
{
    struct persistent_data pdata;
    uint8 idx = 0;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    if (osal_memcmp(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN) == TRUE) {
        for (idx = 0; idx < CUSTOM_DN_LEN; idx++) {
            dn[idx] = pdata.dn[idx];
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
    struct persistent_data pdata;
    uint8 idx = 0;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));

    for (idx = 0; idx < CUSTOM_DN_MAGIC_LEN; idx++) {
        pdata.dn_magic[idx] = 0;
    }
    for (idx = 0; idx < CUSTOM_DN_LEN; idx++) {
        pdata.dn[idx] = 0;
    }
    dn[CUSTOM_DN_LEN] = 0;
    osal_memcpy(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN);
    osal_memcpy(pdata.dn, dn, osal_strlen((char *)dn));

    HalFlashErase(CUSTOM_DN_PAGE_IDX);
    HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) / 4), (uint8 *)&pdata, sizeof(pdata) / 4);
}

uint8 custom_mgmt_initialized(void)
{
    struct persistent_data pdata;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    if (osal_memcmp(pdata.mgmt_magic, MGMT_MAGIC, MGMT_MAGIC_LEN) == TRUE) {
        return 1;
    } else {
        return 0;
    }
}

void custom_mgmt_set(struct pwrmgmt_data *pwr)
{
    struct persistent_data pdata;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    osal_memcpy(pdata.mgmt_magic, MGMT_MAGIC, MGMT_MAGIC_LEN);
    osal_memcpy((uint8 *)&(pdata.mgmt), (uint8 *)pwr, sizeof(struct pwrmgmt_data));

    HalFlashErase(CUSTOM_DN_PAGE_IDX);
    HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) / 4), (uint8 *)&pdata, sizeof(pdata) / 4);
}

void custom_mgmt_get(struct pwrmgmt_data *pwr)
{
    struct persistent_data pdata;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    osal_memcpy((uint8 *)pwr, (uint8 *)&(pdata.mgmt), sizeof(struct pwrmgmt_data));
}

