/*
 * Customize device name etc.
 * The flash pages for customization is 
 * 123, 124
 */
#ifndef __CUSTOM__
#define __CUSTOM__

#include "hal_types.h"
#include "pwrmgmt.h"

#define CUSTOM_DN_PAGE_IDX      123
// NOTICE: flash content shall be 4 bytes rounded
#define CUSTOM_DN_MAGIC_LEN     8 
#define CUSTOM_DN_LEN           20 
#define MGMT_MAGIC_LEN          8
#define MGMT_RCD_CNT            100

void custom_init(void);
void custom_get_dn(uint8 *dn);
void custom_set_dn(uint8 *dn);

struct persistent_data {
    uint8 dn_magic[CUSTOM_DN_MAGIC_LEN];
    uint8 dn[CUSTOM_DN_LEN];
    uint8 mgmt_magic[MGMT_MAGIC_LEN];
};

struct persistent_data_storage {
    uint8 dn_magic[CUSTOM_DN_MAGIC_LEN];
    uint8 dn[CUSTOM_DN_LEN];
    uint8 mgmt_magic[MGMT_MAGIC_LEN];
    struct pwrmgmt_data mgmt[MGMT_RCD_CNT];
};

uint8 custom_mgmt_initialized(void);
void custom_mgmt_set(struct pwrmgmt_data *pwr);
uint8 custom_mgmt_get(struct pwrmgmt_data *pwr);

#endif
