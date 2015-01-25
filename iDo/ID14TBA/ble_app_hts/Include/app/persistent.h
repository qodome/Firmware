/*
 * Keep user persistent data in 0x3BC00 - 0x3C000
 */
#ifndef __PERSISTENT__
#define __PERSISTENT__

#include <stdint.h>
#include <stdlib.h>
#include "nrf_types.h"
#include "pwrmgmt.h"

#define PERSISTENT_ERROR_SYS_MAX		18

#define PERSISTENT_ERROR_ASSERT			(1 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_FLASH_ERASE	(2 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_FLASH_WRITE	(3 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_INTERNAL		(4 + PERSISTENT_ERROR_SYS_MAX)
#define PERSISTENT_ERROR_DEADBEEF		(5 + PERSISTENT_ERROR_SYS_MAX)

#define PERSISTENT_ERROR_MAX			30
#define PERSISTENT_ERROR_ENTRY			3
#define PERSISTENT_PWR_MAX				25

struct error_record {
	uint32_t count_bit;										// One bit for one error count
	uint32_t error_info[PERSISTENT_ERROR_ENTRY];
};

struct persistent_page {
	/* DEADBEEF */
	uint8_t page_magic[4];
	/* Device name */
	uint8_t dev_name[20];
	/* Boot count for debug */
	uint8_t boot_cnt_bits[16];								// One bit for one boot count
	/* Error log */
	struct error_record error_log[PERSISTENT_ERROR_MAX];	// This structure does not get refreshed
	/* Power management */
	uint8_t pwr_idx_bits[4];									// Number of 0bits indicates current index
	struct pwrmgmt_data pwr_log[PERSISTENT_PWR_MAX];		// This structure gets refreshed every 25 hours
} __attribute__((packed));

// Initialize persistent data
void persistent_init(void);

// Max length of device name: 20 (excluding tailing null)
void persistent_flash_backup_prepare(uint16_t offset, uint8_t *buf, uint16_t len);
void persistent_flash_backup_finish(void);

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf);

// Record error info
void persistent_record_error(uint8_t error_idx, uint32_t error_info);

// Power management API
void persistent_pwrmgmt_set_latest(struct pwrmgmt_data *pwr_ptr);
uint8_t persistent_pwrmgmt_get_latest(struct pwrmgmt_data *pwr_ptr);

// Flash range validation
uint8_t persistent_flash_first_page(void);

#endif
