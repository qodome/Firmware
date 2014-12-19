/*
 * Keep user persistent data in 0x3BC00 - 0x3C000
 */
#ifndef __PERSISTENT__
#define __PERSISTENT__

#include <stdint.h>
#include <stdlib.h>

#define PERSISTENT_ERROR_SYS_MAX		18

#define PERSISTENT_ERROR_ASSERT			(1 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_FLASH_ERASE	(2 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_FLASH_WRITE	(3 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_INTERNAL		(4 + PERSISTENT_ERROR_SYS_MAX)

#define PERSISTENT_ERROR_MAX			30

#define PERSISTENT_ERROR_ENTRY			7

struct error_record {
	uint32_t count;
	uint32_t error_info[PERSISTENT_ERROR_ENTRY];
};

struct persistent_record {
	uint8_t dn_magic[8];
	uint8_t device_name[20];
	uint8_t pad0[4];
#ifdef DEBUG_STATS
	uint8_t boot_cnt_magic[8];
	uint32_t boot_cnt;
	uint8_t pad1[20];
#endif
	struct error_record error_log[PERSISTENT_ERROR_MAX];
} __attribute__((packed));

// Initialize persistent data
void persistent_init(void);

// Max length of device name: 20 (excluding tailing null)
void persistent_set_dev_name(uint8_t *buf, uint16_t len);

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf);

// Record error info
void persistent_record_error(uint8_t error_idx, uint32_t error_info);

uint8_t persistent_flash_page(void);

#endif
