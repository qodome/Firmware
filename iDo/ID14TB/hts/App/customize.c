/*
 * Customize
 */
#include "customize.h"
#include "hal_flash.h"
#include "OSAL.h"

static uint8 DN_MAGIC[CUSTOM_DN_MAGIC_LEN] = {'D', 'E', 'V', '_', 'N', 'A', 'M', 'E'};
static uint8 MGMT_MAGIC[MGMT_MAGIC_LEN] = {'M', 'G', 'M', 'T', '_', 'B', 'A', 'T'};
static uint8 DN_DEFAULT[4] = {'i', 'D', 'o', 0};
static uint8 current_mgmt_idx = 0xFF;

/*
 * custom_init will erase the flash page only when device name is not found
 * in firmware upgrade case, this function is not supposed to trigger flash
 * erase
 */
void custom_init(void)
{
    struct pwrmgmt_data mgmt;
    struct persistent_data pdata;
    uint8 idx;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    if (osal_memcmp(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN) != TRUE) {
        // This should happen only when device first powered up
        osal_memcpy(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN);
        osal_memcpy(pdata.dn, DN_DEFAULT, 4);

        HalFlashErase(CUSTOM_DN_PAGE_IDX);
        HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)512)), (uint8 *)&pdata, sizeof(pdata) / 4);
    }

    if (osal_memcmp(pdata.mgmt_magic, MGMT_MAGIC, MGMT_MAGIC_LEN) == TRUE) {
        for (idx = 0; idx < MGMT_RCD_CNT; idx++) {
            HalFlashRead(CUSTOM_DN_PAGE_IDX, (uint16)&(((struct persistent_data_storage *)0)->mgmt[idx]), (uint8 *)&mgmt, sizeof(mgmt));
            if (mgmt.initial_v_adc == 0xFFFF) {
                break;
            }
        }
        if (idx >= MGMT_RCD_CNT) {
            HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));

            HalFlashErase(CUSTOM_DN_PAGE_IDX);
            HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) / 4), (uint8 *)&pdata, sizeof(pdata) / 4);
            HalFlashWrite(((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) + (uint32)&(((struct persistent_data_storage *)0)->mgmt[0])) / 4), (uint8 *)&mgmt, sizeof(mgmt) / 4);

            current_mgmt_idx = 0;
        } else if (idx > 0) {
            current_mgmt_idx = idx - 1;
        }
    }
}

// Input buffer at least 21 bytes (couting tailing zero)
void custom_get_dn(uint8 *dn)
{
    struct persistent_data pdata;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
    if (osal_memcmp(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN) == TRUE) {
        osal_memcpy(&(dn[0]), &(pdata.dn[0]), CUSTOM_DN_LEN);
        dn[CUSTOM_DN_LEN] = 0;
    } else {
        osal_memcpy(&(dn[0]), DN_DEFAULT, 4);
    }
}

// Input buffer at least 21 bytes (couting tailing zero)
void custom_set_dn(uint8 *dn)
{
    struct persistent_data pdata;
    struct pwrmgmt_data mgmt;

    HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));

    dn[CUSTOM_DN_LEN] = 0;
    osal_memcpy(pdata.dn_magic, DN_MAGIC, CUSTOM_DN_MAGIC_LEN);
    osal_memcpy(pdata.dn, dn, osal_strlen((char *)dn) + 1);

    if (current_mgmt_idx != 0xFF) {
        HalFlashRead(CUSTOM_DN_PAGE_IDX, (uint16)&(((struct persistent_data_storage *)0)->mgmt[current_mgmt_idx]), (uint8 *)&mgmt, sizeof(mgmt));
    }

    HalFlashErase(CUSTOM_DN_PAGE_IDX);
    HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) / 4), (uint8 *)&pdata, sizeof(pdata) / 4);

    if (current_mgmt_idx != 0xFF) {
        HalFlashWrite(((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) + (uint32)&(((struct persistent_data_storage *)0)->mgmt[0])) / 4), (uint8 *)&mgmt, sizeof(mgmt) / 4);
        current_mgmt_idx = 0;
    }
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

    if ((current_mgmt_idx == 0xFF) || ((current_mgmt_idx + 1) >= MGMT_RCD_CNT)) {
        HalFlashRead(CUSTOM_DN_PAGE_IDX, 0, (uint8 *)&pdata, sizeof(pdata));
        osal_memcpy(pdata.mgmt_magic, MGMT_MAGIC, MGMT_MAGIC_LEN);

        HalFlashErase(CUSTOM_DN_PAGE_IDX);
        HalFlashWrite((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) / 4), (uint8 *)&pdata, sizeof(pdata) / 4);
        current_mgmt_idx = 0;
        HalFlashWrite(((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) + (uint32)&(((struct persistent_data_storage *)0)->mgmt[current_mgmt_idx])) / 4), (uint8 *)pwr, sizeof(struct pwrmgmt_data) / 4);
    } else {
        current_mgmt_idx++;
        HalFlashWrite(((((uint32)CUSTOM_DN_PAGE_IDX * (uint32)2048) + (uint32)&(((struct persistent_data_storage *)0)->mgmt[current_mgmt_idx])) / 4), (uint8 *)pwr, sizeof(struct pwrmgmt_data) / 4);
    }
}

uint8 custom_mgmt_get(struct pwrmgmt_data *pwr)
{
    if (current_mgmt_idx == 0xFF) {
        // Nothing is here
        return 1;
    }

    HalFlashRead(CUSTOM_DN_PAGE_IDX, (uint16)&(((struct persistent_data_storage *)0)->mgmt[current_mgmt_idx]), (uint8 *)pwr, sizeof(struct pwrmgmt_data));
    return 0;
}

