/*
 * Keep user persistent data in 0x3BC00 - 0x3C000
 */
#ifndef __PERSISTENT__
#define __PERSISTENT__

#include <stdint.h>
#include <stdlib.h>

struct persistent_record {
	uint8_t dn_magic[8];
	uint8_t device_name[20];
#ifdef DEBUG_STATS
	uint8_t boot_cnt_magic[8];
	uint32_t boot_cnt;
#endif
} __attribute__((packed));

// Initialize persistent data
void persistent_init(void);

// Max length of device name: 20 (excluding tailing null)
void persistent_set_dev_name(uint8_t *buf, uint16_t len);

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf);

#endif
