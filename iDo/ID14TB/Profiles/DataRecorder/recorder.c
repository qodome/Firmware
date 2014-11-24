/*
 * Data recorder keeps track of writing data into
 * flash and query for history data
 */

#include "hal_flash.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_defs.h"
#include "recorder.h"
#include "OSAL.h"
#include "bcomdef.h"
#include "oad.h"
#include "oad_target.h"
#ifdef DEBUG_STATS
#include "statistics.h"
#endif
#include "OSAL_Clock.h"
     
/*
 * Notice: page 123, 124 is reserved for custom device name
 */

#if defined(HAL_IMAGE_A) || defined(HAL_IMAGE_B)

// Member variables
struct record_position rec_next_write;
struct record_read_temperature rec_read;
struct record_entry rec_latest;
uint32 latest_sample_unix_ts;
int16 latest_sample_temp;
int8 rec_latest_entry_idx = -1;

#ifdef DEBUG_WATCHDOG
static uint8 __recorder_get_wd_page(void)
{
#if defined(HAL_IMAGE_A)
    return 8;               // page 8 is reserved for watchdog
#else
    return 1;               // page 1 is reserved for watchdog
#endif    
}
#endif

static uint8 __recorder_get_first_page(void)
{
#if defined(HAL_IMAGE_A)
#ifdef DEBUG_WATCHDOG
    return 9;
#else
    return 8;
#endif
#else
#ifdef DEBUG_WATCHDOG
    return 2;
#else
    return 1;
#endif
#endif
}

#ifdef DEBUG_WATCHDOG
// Update watch dog restart statistics
void recorder_touch_wd_page(void)
{
    uint8 buf[8];
    
    // The first 4 bytes should be WDG<OAD_IMAGE_VERSION_tail>
    HalFlashRead(__recorder_get_wd_page(), 0, buf, 8);
    
    if (buf[0] == 'W' && buf[1] == 'D' && buf[2] == 'G' && buf[3] == (uint8)((OAD_IMAGE_VERSION) & 0xFF)) {
        ((uint32 *)buf)[1]++;
    } else {
        buf[0] = 'W';
        buf[1] = 'D';
        buf[2] = 'G';
        buf[3] = (uint8)((OAD_IMAGE_VERSION) & 0xFF);
        ((uint32 *)buf)[1] = 1;
    }
    
    HalFlashErase(__recorder_get_wd_page());    
    HalFlashWrite(((((uint32)__recorder_get_wd_page()) * 2048) / 4), buf, 2);    
}

uint32 recorder_get_wd_boot_count(void)
{
    uint8 buf[8];
    uint32 ret = 0;
    
    // The first 4 bytes should be WDG<OAD_IMAGE_VERSION_tail>
    HalFlashRead(__recorder_get_wd_page(), 0, buf, 8);
    
    if (buf[0] == 'W' && buf[1] == 'D' && buf[2] == 'G' && buf[3] == (uint8)((OAD_IMAGE_VERSION) & 0xFF)) {
        ret = ((uint32 *)buf)[1];
    }    
    return ret;
}
#endif

/*
 * Get the next page idx
 */
static uint8 __recorder_get_next_page(uint8 pg_idx)
{
#if defined(HAL_IMAGE_A)
    if (pg_idx == 68) {
        return __recorder_get_first_page();
    }
#else
    if (pg_idx == 7) {
        return 69;
    } else if (pg_idx == 122) {
        return __recorder_get_first_page();
    }
#endif

    return (pg_idx + 1);
}

static int __recorder_get_record_buf(int8 page_idx, int8 record_entry_idx, int8 data_entry_idx, uint8 *buf_out)
{
    uint8 magic[2] = {0};

    if (rec_next_write.page_idx == page_idx && 
        rec_next_write.record_entry_idx == record_entry_idx) {
        if (rec_latest_entry_idx > data_entry_idx) {
            if (data_entry_idx == -1) {
                osal_memcpy(buf_out, (uint8 *)&rec_latest, 6);
            } else {
                osal_memcpy(buf_out, (uint8 *)&(rec_latest.entries[data_entry_idx]), sizeof(struct data_entry));
            }
        } else {
            return -1;
        }
    } else {
        if (data_entry_idx == -1) {
            HalFlashRead(page_idx, (int16)(record_entry_idx + 1) * sizeof(struct record_entry) - 2, magic, 2);
            if (magic[0] == 0x52 && magic[1] == 0x44) {
                HalFlashRead(page_idx, (int16)record_entry_idx * sizeof(struct record_entry), buf_out, 6);                
            } else {
                // magic not found
                return -1;
            }
        } else {
            HalFlashRead(page_idx, (int16)record_entry_idx * sizeof(struct record_entry) + REC_DATA_ENTRY_OFFSET + (int16)data_entry_idx * sizeof(struct data_entry), 
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
    int available_page_idx = -1;
    uint8 buf[4];
    int page_idx = __recorder_get_first_page();
    
    do {
       HalFlashRead(page_idx, 0, buf, 4);
       if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) {
           available_page_idx = page_idx;
           break;
       }        
       page_idx = __recorder_get_next_page(page_idx);
    } while (page_idx != __recorder_get_first_page());
    
    if (available_page_idx == -1) {
        available_page_idx = __recorder_get_first_page();
    }
    
    HalFlashErase(available_page_idx);
    HalFlashErase(__recorder_get_next_page(available_page_idx));
    rec_next_write.page_idx = available_page_idx;
    rec_next_write.record_entry_idx = 0;
    osal_memset(&rec_latest, 0, sizeof(struct record_entry));
    rec_latest_entry_idx = -1;

    rec_read.page_idx = available_page_idx;
    rec_read.record_entry_idx = 0;
    rec_read.data_entry_idx = -1;
    rec_read.last_data_entry_unix_ts = 0;
    rec_read.last_data_entry_temp = 0;
}

void recorder_add_temperature(int16 temp)
{
    uint32 ts = 0;
    uint16 addr = 0;

    if (osal_TimeInitialized() == 0) {
        // If wall time is not initialized yet, skip recording
        return;
    }

    ts = osal_getClock();

    if (rec_latest_entry_idx == -1) {
        rec_latest.unix_time = ts;
        rec_latest.base_temp = temp;
        rec_latest_entry_idx++;
    } else {
        if ((ts - latest_sample_unix_ts) >= 32) {
            // Temperature recording stopped for a while, open new block
            
            // First dump data into flash
            addr = ((uint32)rec_next_write.page_idx * 2048 + (uint32)rec_next_write.record_entry_idx * sizeof(struct record_entry)) / 4;
            rec_latest.magic[0] = 0x52;
            rec_latest.magic[1] = 0x44;
            HalFlashWrite(addr, (uint8 *)&rec_latest, (sizeof(struct record_entry)/4));
                
            // Cleanup stale temperature records
            osal_memset(&rec_latest, 0, sizeof(struct record_entry));
                
            rec_next_write.record_entry_idx++;
            if (rec_next_write.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                // Current flash page full, move on to the next page
                rec_next_write.page_idx = __recorder_get_next_page(rec_next_write.page_idx);
                HalFlashErase(__recorder_get_next_page(rec_next_write.page_idx));
                rec_next_write.record_entry_idx = 0;
            }
            
            rec_latest.unix_time = ts;
            rec_latest.base_temp = temp;
            rec_latest_entry_idx = 0;            
        } else {
            // Normal record entry
            rec_latest.entries[rec_latest_entry_idx].delta_time_temp_h = 
                (((uint8)((ts - latest_sample_unix_ts) & 0x1F)) << 3) | 
                    ((uint8)(((temp - latest_sample_temp) >> 8) & 0x07));
            rec_latest.entries[rec_latest_entry_idx].delta_temp_l = 
                ((uint8)((temp - latest_sample_temp) & 0xFF));
            
            rec_latest_entry_idx++;
            if (rec_latest_entry_idx >= REC_DATA_PER_ENTRY) {
                // Latest record full, dump into flash
                addr = ((uint32)rec_next_write.page_idx * 2048 + (uint32)rec_next_write.record_entry_idx * sizeof(struct record_entry)) / 4;
                rec_latest.magic[0] = 0x52;
                rec_latest.magic[1] = 0x44;
                HalFlashWrite(addr, (uint8 *)&rec_latest, (sizeof(struct record_entry)/4));
                
                // Cleanup stale temperature records
                osal_memset(&rec_latest, 0, sizeof(struct record_entry));
                
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
int16 recorder_get_temperature(UTCTimeStruct *tc_out)
{
    int16 ret_temp = 0, delta_temp = 0;
    uint8 buf[16];
    uint32 tmp_ts;

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
            delta_temp = (((uint16)(buf[0] & 0x07) << 8) | 0xF800) | buf[1];
        } else {
            // positive
            delta_temp = ((uint16)(buf[0] & 0x07) << 8) | buf[1];
        }
        ret_temp = rec_read.last_data_entry_temp + delta_temp; 
        
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
        ret_temp = ((uint16)buf[5] << 8) | buf[4];
        tmp_ts = (uint32)buf[3] << 24 | (uint32)buf[2] << 16 | (uint32)buf[1] << 8 | (uint32)buf[0];
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
void recorder_set_read_base_ts(UTCTimeStruct *tc)
{
    uint32 ts = 0;
    uint8 page_idx = 0, last_page_idx = 0;
    uint8 rec_idx = 0, last_rec_idx = 0;
    uint8 check_previous = 0;
    uint8 buf[4];
    // Get the latest recorder before the search point
    uint8 magic[2] = {0};
    uint8 page_idx_ancient = 0;
    uint8 rec_idx_ancient = 0;
    uint32 delta_ancient = 0xFFFFFFFF;

    if (tc != NULL) {
        ts = osal_ConvertUTCSecs(tc);
        // Search the ts over all flash pages
        page_idx = __recorder_get_first_page();
        do {
            for (rec_idx = 0; rec_idx < REC_ENTRY_PER_PAGE; rec_idx++) {
                // First check magic "RD"
                HalFlashRead(page_idx, (uint16)(rec_idx + 1) * sizeof(struct record_entry) - 2, magic, 2);
                if (magic[0] == 0x52 && magic[1] == 0x44) {
                    HalFlashRead(page_idx, (uint16)rec_idx * sizeof(struct record_entry), buf, 4);
                    if (ts >= *(uint32 *)buf) {
                        if ((ts - *(uint32 *)buf) < delta_ancient) {
                            delta_ancient = (ts - *(uint32 *)buf);
                            page_idx_ancient = page_idx;
                            rec_idx_ancient = rec_idx;
                        }
                    }
                    if ((*(uint32 *)buf <= ts) && ((ts - *(uint32 *)buf) < 2000)) {
                        check_previous = 1;
                        last_page_idx = page_idx;
                        last_rec_idx = rec_idx;
                    } else if (check_previous == 1 && (*(uint32 *)buf >= ts) && ((*(uint32 *)buf - ts) < 2000)) {
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

#else
/* 
 * stub for non-OAD target
 */
void recorder_init(void)
{

}

void recorder_add_temperature(int16 temp)
{

}

#endif

