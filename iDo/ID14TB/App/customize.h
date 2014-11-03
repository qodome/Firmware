/*
 * Customize device name etc.
 * The flash pages for customization is 
 * 123, 124
 */
#ifndef __CUSTOM__
#define __CUSTOM__

#include "hal_types.h"

#define CUSTOM_DN_PAGE_IDX      123
// NOTICE: flash content shall be 4 bytes rounded
#define CUSTOM_DN_MGIC_OFFSET   0
#define CUSTOM_DN_MAGIC_LEN     8 
#define CUSTOM_DN_LEN           20 

void custom_init(void);
void custom_get_dn(uint8 *dn);
void custom_set_dn(uint8 *dn);

#endif
