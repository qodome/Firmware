/*
 * Data recorder keeps track of writing data into
 * flash and query for history data
 */
#include <stdlib.h>
#include <string.h>
#include "recorder.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_assert.h"
#include "temp_date_time.h"
#include "UTCtime_convert.h"
#include "flash_helper.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "pstorage_platform.h"
     
extern uint32_t CodeOrigin;
extern uint32_t CodeLength;

// Member variables
struct record_position rec_next_write;
struct record_read_temperature rec_read;
struct record_entry rec_latest;
uint32_t latest_sample_unix_ts;
int16_t latest_sample_temp;
int8_t rec_latest_entry_idx = -1;
extern int8_t rssi;

static uint8_t __recorder_get_first_page(void)
{
	uint32_t code_origin = 0;
	uint32_t code_length = 0;
	uint32_t page_size = 0;

	code_origin = (uint32_t)&CodeOrigin;
	code_length = (uint32_t)&CodeLength;
	page_size = PSTORAGE_FLASH_PAGE_SIZE;

    return (uint8_t)(((code_origin + code_length) / page_size) & 0xFF);
}

/*
 * Get the next page idx
 */
static uint8_t __recorder_get_next_page(uint8_t pg_idx)
{
	uint8_t idx_end;

	idx_end = (uint8_t)((PSTORAGE_DATA_START_ADDR / PSTORAGE_FLASH_PAGE_SIZE) & 0xFF);
    if (pg_idx >= (idx_end - 1)) {
        return __recorder_get_first_page();
    }

    return (pg_idx + 1);
}

uint8_t recorder_first_page(void)
{
	return __recorder_get_first_page();
}

uint8_t recorder_last_page(void)
{
	return (uint8_t)((PSTORAGE_DATA_START_ADDR / PSTORAGE_FLASH_PAGE_SIZE) & 0xFF) - 1;
}

static int8_t __recorder_get_record_buf(uint8_t page_idx, uint8_t record_entry_idx, int8_t data_entry_idx, uint8_t *buf_out)
{
    uint8_t magic[2] = {0};

    if (rec_next_write.page_idx == page_idx && 
        rec_next_write.record_entry_idx == record_entry_idx) {
        if (rec_latest_entry_idx > data_entry_idx) {
            if (data_entry_idx == -1) {
                memcpy(buf_out, (uint8_t *)&rec_latest, REC_DATA_ENTRY_OFFSET);
            } else {
                memcpy(buf_out, (uint8_t *)&(rec_latest.entries[data_entry_idx]), sizeof(struct data_entry));
            }
        } else {
            return -1;
        }
    } else {
        if (data_entry_idx == -1) {
            HalFlashRead(page_idx, (uint16_t)(record_entry_idx + 1) * sizeof(struct record_entry) - 2, magic, 2);
            if (magic[0] == 0x52 && magic[1] == 0x44) {
                HalFlashRead(page_idx, (uint16_t)record_entry_idx * sizeof(struct record_entry), buf_out, REC_DATA_ENTRY_OFFSET);
            } else {
                // magic not found
                return -1;
            }
        } else {
            HalFlashRead(page_idx, (uint16_t)record_entry_idx * sizeof(struct record_entry) + REC_DATA_ENTRY_OFFSET + (uint16_t)data_entry_idx * sizeof(struct data_entry),
                        buf_out, sizeof(struct data_entry));
        }
    }
    return 0;
}

/*
 * This function initialize the flash page after
 * power up: 
 * 1. Find the first flash page whose first 4 bytes are 0xFF
 *    (after flash erase, the content is cleared as 1)
 * 2. Erase the found page and the next circular page
 * 
 * WARNING: this function must be invoked before begin temperature
 * measure
 */
void recorder_init(void)
{
    uint8_t available_page_idx = 0;
    uint8_t buf[4];
    uint8_t page_idx = __recorder_get_first_page();

    do {
       HalFlashRead(page_idx, 0, buf, 4);
       if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) {
           available_page_idx = page_idx;
           break;
       }        
       page_idx = __recorder_get_next_page(page_idx);
    } while (page_idx != __recorder_get_first_page());
    
    if (available_page_idx == 0) {
        available_page_idx = __recorder_get_first_page();
    }
    
    HalFlashErase(available_page_idx);
    rec_next_write.page_idx = available_page_idx;
    rec_next_write.record_entry_idx = 0;
    memset(&rec_latest, 0, sizeof(struct record_entry));
    rec_latest_entry_idx = -1;

    rec_read.page_idx = available_page_idx;
    rec_read.record_entry_idx = 0;
    rec_read.data_entry_idx = -1;
    rec_read.last_data_entry_unix_ts = 0;
    rec_read.last_data_entry_temp = 0;
}

void recorder_add_temperature(int16_t temp)
{
    uint32_t ts = 0;
    uint32_t addr = 0;

    if (flag_time_stamp_get() == 0) {
        // If wall time is not initialized yet, skip recording
        return;
    }

    ts = date_time_get();

    if (rec_latest_entry_idx == -1) {
        rec_latest.unix_time = ts;
        rec_latest.base_temp = temp;
        rec_latest.rssi = rssi;
        rec_latest.dummy = 0;
        rec_latest_entry_idx++;
    } else {
        if ((ts - latest_sample_unix_ts) >= 32) {
            // Temperature recording stopped for a while, open new block
            
            // First dump data into flash
            addr = (uint32_t)rec_next_write.page_idx * 1024 + (uint32_t)rec_next_write.record_entry_idx * sizeof(struct record_entry);
            rec_latest.magic[0] = 0x52;
            rec_latest.magic[1] = 0x44;
            HalFlashWrite((uint32_t *)addr, (uint8_t *)&rec_latest, sizeof(struct record_entry));
                
            // Cleanup stale temperature records
            memset(&rec_latest, 0, sizeof(struct record_entry));
                
            rec_next_write.record_entry_idx++;
            if (rec_next_write.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                // Current flash page full, move on to the next page
                rec_next_write.page_idx = __recorder_get_next_page(rec_next_write.page_idx);
                HalFlashErase(__recorder_get_next_page(rec_next_write.page_idx));
                rec_next_write.record_entry_idx = 0;
            }
            
            rec_latest.unix_time = ts;
            rec_latest.base_temp = temp;
            rec_latest.rssi = rssi;
            rec_latest.dummy = 0;
            rec_latest_entry_idx = 0;            
        } else {
            // Normal record entry
            rec_latest.entries[rec_latest_entry_idx].delta_time_temp_h = 
                (((uint8_t)((ts - latest_sample_unix_ts) & 0x1F)) << 3) | 
                    ((uint8_t)(((temp - latest_sample_temp) >> 8) & 0x07));
            rec_latest.entries[rec_latest_entry_idx].delta_temp_l = 
                ((uint8_t)((temp - latest_sample_temp) & 0xFF));
            rec_latest.entries[rec_latest_entry_idx].rssi = rssi;
            
            rec_latest_entry_idx++;
            if (rec_latest_entry_idx >= REC_DATA_PER_ENTRY) {
                // Latest record full, dump into flash
                addr = (uint32_t)rec_next_write.page_idx * 1024 + (uint32_t)rec_next_write.record_entry_idx * sizeof(struct record_entry);
                rec_latest.magic[0] = 0x52;
                rec_latest.magic[1] = 0x44;
                HalFlashWrite((uint32_t *)addr, (uint8_t *)&rec_latest, sizeof(struct record_entry));
                
                // Cleanup stale temperature records
                memset(&rec_latest, 0, sizeof(struct record_entry));
                
                rec_next_write.record_entry_idx++;
                if (rec_next_write.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                    // Current flash page full, move on to the next page
                    rec_next_write.page_idx = __recorder_get_next_page(rec_next_write.page_idx);
                    HalFlashErase(__recorder_get_next_page(rec_next_write.page_idx));
                    rec_next_write.record_entry_idx = 0;
                }
                rec_latest_entry_idx = -1;
            }
        }
    }

    latest_sample_unix_ts = ts;
    latest_sample_temp = temp;
}

/*
 * If lookup successful, this function
 * returns both the temperature value
 * and calendar
 *
 * Return the next temperature value in flash
 * if that record looks good, otherwise return
 * 0 temperature and do not touch tc_out
 */
int16_t recorder_get_temperature(ble_date_time_t *tc_out, int8_t *p_rssi)
{
    int16_t ret_temp = 0, delta_temp = 0;
    uint8_t buf[16];
    uint32_t tmp_ts;

    // If data_entry_idx is [0, (REC_DATA_PER_ENTRY - 1)], then
    // current data entry must be valid
    if (rec_read.data_entry_idx >= 0 && 
        rec_read.data_entry_idx <= (REC_DATA_PER_ENTRY - 1)) {
        if (__recorder_get_record_buf(rec_read.page_idx, rec_read.record_entry_idx, rec_read.data_entry_idx, buf) != 0) {
            ret_temp = 0;
            return ret_temp;
        }
        // The current temperature is invalid, skip to the next block
        if (buf[0] == 0x00 && buf[1] == 0x00) {
            rec_read.data_entry_idx = -1;
            rec_read.record_entry_idx++;
            if (rec_read.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                rec_read.record_entry_idx = 0;
                rec_read.page_idx = __recorder_get_next_page(rec_read.page_idx);
            }
            goto recheck;
        }
        if (buf[0] & 0x04) {
            // negative
            delta_temp = (((uint16_t)(buf[0] & 0x07) << 8) | 0xF800) | buf[1];
        } else {
            // positive
            delta_temp = ((uint16_t)(buf[0] & 0x07) << 8) | buf[1];
        }
        ret_temp = rec_read.last_data_entry_temp + delta_temp;
        *p_rssi = (int8_t)buf[2];
        
        osal_ConvertUTCTime(tc_out, rec_read.last_data_entry_unix_ts + ((buf[0] >> 3) & 0x1F));
        
        rec_read.last_data_entry_unix_ts = rec_read.last_data_entry_unix_ts + ((buf[0] >> 3) & 0x1F);
        rec_read.last_data_entry_temp = ret_temp;
        rec_read.data_entry_idx++;
        if (rec_read.data_entry_idx > (REC_DATA_PER_ENTRY - 1)) {
            rec_read.data_entry_idx = -1;
            rec_read.record_entry_idx++;
            if (rec_read.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                rec_read.record_entry_idx = 0;
                rec_read.page_idx = __recorder_get_next_page(rec_read.page_idx);
            }
        }
    } else if (rec_read.data_entry_idx == -1) {
recheck:
        // The first data entry on the none-first record
        if (__recorder_get_record_buf(rec_read.page_idx, rec_read.record_entry_idx, rec_read.data_entry_idx, buf) != 0) {
            rec_read.data_entry_idx = -1;
            rec_read.record_entry_idx++;
            if (rec_read.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                rec_read.record_entry_idx = 0;
                rec_read.page_idx = __recorder_get_next_page(rec_read.page_idx);
            }
            return 0;
        }
        ret_temp = ((uint16_t)buf[5] << 8) | buf[4];
        tmp_ts = (uint32_t)buf[3] << 24 | (uint32_t)buf[2] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[0];
        *p_rssi = (int8_t)buf[6];
        rec_read.last_data_entry_temp = ret_temp;
        rec_read.last_data_entry_unix_ts = tmp_ts;
        
        osal_ConvertUTCTime(tc_out, rec_read.last_data_entry_unix_ts);
        
        rec_read.data_entry_idx = 0;
    } else if (rec_read.data_entry_idx > (REC_DATA_PER_ENTRY - 1)) {
        // BUG
        ret_temp = 0;
    }

    return ret_temp;
}

/*
 * Modify read pointer based on input
 */
void recorder_set_read_base_ts(ble_date_time_t *tc)
{
    uint32_t ts = 0;
    uint8_t page_idx = 0, last_page_idx = 0;
    uint8_t rec_idx = 0, last_rec_idx = 0;
    uint8_t check_previous = 0;
    uint32_t buf32;
    // Get the latest recorder before the search point
    uint8_t magic[2] = {0};
    uint8_t page_idx_ancient = 0;
    uint8_t rec_idx_ancient = 0;
    uint32_t delta_ancient = 0xFFFFFFFF;

    if (tc != NULL) {
        ts = osal_ConvertUTCSecs(tc);
        // Search the ts over all flash pages
        page_idx = __recorder_get_first_page();
        do {
            for (rec_idx = 0; rec_idx < REC_ENTRY_PER_PAGE; rec_idx++) {
                // First check magic "RD"
                HalFlashRead(page_idx, (uint16_t)(rec_idx + 1) * sizeof(struct record_entry) - 2, magic, 2);
                if (magic[0] == 0x52 && magic[1] == 0x44) {
                	HalFlashRead(page_idx, (uint16_t)rec_idx * sizeof(struct record_entry), (uint8_t *)&buf32, 4);
                    if (ts >= buf32) {
                        if ((ts - buf32) < delta_ancient) {
                            delta_ancient = (ts - buf32);
                            page_idx_ancient = page_idx;
                            rec_idx_ancient = rec_idx;
                        }
                    }
                	if ((buf32 <= ts) && ((ts - buf32) < 2000)) {
                		check_previous = 1;
                		last_page_idx = page_idx;
                		last_rec_idx = rec_idx;
                	} else if (check_previous == 1 && (buf32 >= ts) && ((buf32 - ts) < 2000)) {
                		rec_read.page_idx = last_page_idx;
                		rec_read.record_entry_idx = last_rec_idx;
                		rec_read.data_entry_idx = -1;
                		rec_read.last_data_entry_unix_ts = 0;
                		rec_read.last_data_entry_temp = 0;
                		return;
                	} else {
                		check_previous = 0;
                	}
                }
            }
            page_idx = __recorder_get_next_page(page_idx);
        } while (page_idx != __recorder_get_first_page());

        if (page_idx_ancient != 0) {
            rec_read.page_idx = page_idx_ancient;
            rec_read.record_entry_idx = rec_idx_ancient;
            rec_read.data_entry_idx = -1;
            rec_read.last_data_entry_unix_ts = 0;
            rec_read.last_data_entry_temp = 0;
        }
    }
}

