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
#include "iDo.h"
#include "temperature.h"
#include "hal_sleep.h"
     
#define TEMP_SAMPLE_GAP_THRESHOLD   20      // FIXME: hard code

/*
 * Notice: page 123, 124 is reserved for custom device name
 * page 125, 126 are used by osal_snv.c
 */

#if defined(HAL_IMAGE_A) || defined(HAL_IMAGE_B)

// Member variables
struct record_position rec_next_write;
struct record_entry rec_latest;
uint32 latest_sample_unix_ts;
int16 latest_sample_temp;
int8 rec_latest_entry_idx = -1;
uint8 all_ff[4] = {0xff, 0xff, 0xff, 0xff};
#ifdef TRACE_API_TIME
uint32 recorder_api_task_period = 0;
#endif

struct record_read_temperature rec_read;
struct query_db qdb;
struct query_criteria record_query_result_buffer;

uint8 __recorder_get_current_tail_next_head_info(uint8 pg_idx, uint8 rec_idx, uint32 head_ts, uint32 *tail_ts_ptr, 
                                                uint8 *npg_ptr, uint8 *nrec_ptr, uint32 *nhead_ts_ptr);

#define RECORDER_FOR_CONTINUE   1
#define RECORDER_FOR_RETURN     2
//#define RECORDER_FOR_BREAK      3
typedef uint8 (*recorder_for_callback)(uint8 pg_idx, uint8 rec_idx, uint8 *cb_ret_ptr, struct for_task_param *task_param);

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

	// FIXME: need to make sure erase finished before write!
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
    if (pg_idx >= 68 || pg_idx <= 7) {
        return __recorder_get_first_page();
    }
#else
    if (pg_idx >= 7 && pg_idx < 69) {
        return 69;
    } else if (pg_idx >= 122 || pg_idx == 0) {
        return __recorder_get_first_page();
    }
#endif

    return (pg_idx + 1);
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
    uint8 available_page_idx = 0;
    uint8 buf[4];
    uint8 page_idx = __recorder_get_first_page();
    
    do {
       HalFlashRead(page_idx, 0, buf, 4);
       if (osal_memcmp(buf, all_ff, 4) == TRUE) {
           available_page_idx = page_idx;
           break;
       }        
       page_idx = __recorder_get_next_page(page_idx);
    } while (page_idx != __recorder_get_first_page());
    
    if (available_page_idx == 0) {
        available_page_idx = __recorder_get_first_page();
    }
    
    HalFlashErase(available_page_idx);
    HalFlashErase(__recorder_get_next_page(available_page_idx));
    rec_next_write.page_idx = available_page_idx;
    rec_next_write.record_entry_idx = 0;
    osal_memset(&rec_latest, 0, sizeof(struct record_entry));
    rec_latest_entry_idx = -1;

    osal_memset((uint8 *)&rec_read, 0, sizeof(rec_read));
    rec_read.page_idx = available_page_idx;
    rec_read.data_entry_idx = -1;
    osal_memset((uint8 *)&qdb, 0, sizeof(qdb));
}

void recorder_add_temperature(int16 temp)
{
    uint32 ts;
    uint16 addr;

    if (osal_TimeInitialized() == 0) {
        // If wall time is not initialized yet, skip recording
        return;
    }

    ts = osal_getClock();

    if (rec_latest_entry_idx == -1) {
        rec_latest.unix_time = ts;
        rec_latest.base_temp = temp;
        rec_latest_entry_idx++;
        rec_latest.end_delta = 0;
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
            rec_latest.end_delta = 0;
        } else {
            // Normal record entry
            rec_latest.entries[rec_latest_entry_idx].delta_time_temp_h = 
                (((uint8)((ts - latest_sample_unix_ts) & 0x1F)) << 3) | 
                    ((uint8)(((temp - latest_sample_temp) >> 8) & 0x07));
            rec_latest.entries[rec_latest_entry_idx].delta_temp_l = 
                ((uint8)((temp - latest_sample_temp) & 0xFF));
            
            rec_latest.end_delta = (uint16)(ts - rec_latest.unix_time);
            
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

/******************************************************************************/
/***********     T E M P E R A T U R E    Q U E R Y    A P I      *************/
/******************************************************************************/

/*
 * Utility function to calculate +delta temp/ts
 */
static void __recorder_add_delta(int16 *temp_ptr, uint32 *ts_ptr, struct data_entry *entry)
{
    int16 delta_temp;
    
    if (entry->delta_time_temp_h & 0x04) {
        // negative
        delta_temp = (int16)(((((uint16)(entry->delta_time_temp_h & 0x07)) << 8) | 0xF800) | ((uint16)entry->delta_temp_l));
    } else {
        // positive
        delta_temp = (int16)((((uint16)(entry->delta_time_temp_h & 0x07)) << 8) | ((uint16)entry->delta_temp_l));
    }
    *temp_ptr += delta_temp;
    *ts_ptr += (uint32)((entry->delta_time_temp_h >> 3) & 0x1F);    
}

/*
 * Utility to get one sample info
 */
static uint8 __recorder_get_sample_info(uint8 pg_idx, uint8 rec_idx, int8 entry_idx,
                                        int16 *temp_ptr, uint32 *ts_ptr)
{
    uint8 magic[2];
    static struct record_entry rec_util;
    static uint8 pg_idx_util = 0;
    static uint8 rec_idx_util = 0;
    static int8 entry_idx_util = -1;
    static uint8 rec_valid_flag_util = 0;    
    static int16 temp_util = 0;
    static uint32 ts_util = 0;
    
    if ((pg_idx_util == pg_idx) && (rec_idx_util == rec_idx) && (rec_valid_flag_util == 1)) {
        // Fall through
    } else {
        // Read flash block
        rec_valid_flag_util = 0;
        pg_idx_util = pg_idx;
        rec_idx_util = rec_idx;
        entry_idx_util = -1;
        
        HalFlashRead(pg_idx_util, (uint16)(rec_idx_util + 1) * sizeof(struct record_entry) - 2, magic, 2);
        if (magic[0] == 0x52 && magic[1] == 0x44) {
            HalFlashRead(pg_idx_util, (uint16)rec_idx_util * sizeof(struct record_entry), (uint8 *)&rec_util, sizeof(rec_util));
            rec_valid_flag_util = 1;
            temp_util = rec_util.base_temp;
            ts_util = rec_util.unix_time; 
            // Fall through
        } else {
            return RECORDER_ERROR;
        }        
    }
    
    // Calculate result
    if (entry_idx == entry_idx_util) {
        *temp_ptr = temp_util;
        *ts_ptr = ts_util;
        
        return RECORDER_SUCCESS;
    } else if ((entry_idx_util + 1) == entry_idx) {
        __recorder_add_delta(&temp_util, &ts_util, &(rec_util.entries[entry_idx]));
    } else {
        temp_util = rec_util.base_temp;
        ts_util = rec_util.unix_time;
        for (uint8 idx = 0; idx <= entry_idx; idx++) {
            __recorder_add_delta(&temp_util, &ts_util, &(rec_util.entries[idx])); 
        }
    }
    *temp_ptr = temp_util;
    *ts_ptr = ts_util;
    entry_idx_util = entry_idx;
    
    return RECORDER_SUCCESS;                       
}

/*
 * Utility function to query information regarding the current && next data sample
 */
static uint8 __recorder_get_current_next_info(uint8 pg_idx, uint8 rec_idx, int8 entry_idx,
                                              int16 *temp_ptr, uint32 *ts_ptr,
                                              uint8 *npg_ptr, uint8 *nrec_ptr, int8 *nentry_ptr,
                                              int16 *ntemp_ptr, uint32 *nts_ptr)
{    
    // Update IDX
    *npg_ptr = pg_idx;
    *nrec_ptr = rec_idx;
    if (entry_idx >= (REC_DATA_PER_ENTRY - 1)) {
        *nentry_ptr = -1;
        if (rec_idx >= (REC_ENTRY_PER_PAGE - 1)) {
            *npg_ptr = __recorder_get_next_page(pg_idx);
            *nrec_ptr = 0;
        } else {
            *nrec_ptr = rec_idx + 1;
        }
    } else {
        *nentry_ptr = entry_idx + 1;
    }
    
    // In case we need current sample info
    if (temp_ptr != NULL || ts_ptr != NULL) {
        if (__recorder_get_sample_info(pg_idx, rec_idx, entry_idx, temp_ptr, ts_ptr) != RECORDER_SUCCESS) {
            goto error;
        }
    }
    
    if (__recorder_get_sample_info(*npg_ptr, *nrec_ptr, *nentry_ptr, ntemp_ptr, nts_ptr) != RECORDER_SUCCESS) {
        goto error;
    } else {
        return RECORDER_SUCCESS;
    }

error:
    // Restore IDX
    *npg_ptr = pg_idx;
    *nrec_ptr = rec_idx;
    *nentry_ptr = entry_idx;
    return RECORDER_ERROR;
}

/*
 * Do loop task and return
 */
static uint8 recorder_for_task(recorder_for_callback callback, struct for_task_param *task_param)
{
    uint8 page_idx;
    uint8 magic[2];
    uint8 ret;
    uint8 callback_ret = RECORDER_ERROR;
    
    page_idx = __recorder_get_first_page();
    do {
        for (uint8 rec_idx = 0; rec_idx < REC_ENTRY_PER_PAGE; rec_idx++) {
            // First check magic "RD"
            HalFlashRead(page_idx, (uint16)(rec_idx + 1) * sizeof(struct record_entry) - 2, magic, 2);
            if (magic[0] == 0x52 && magic[1] == 0x44) {
                ret = (*callback)(page_idx, rec_idx, &callback_ret, task_param);
                if (ret == RECORDER_FOR_RETURN) {
                    return callback_ret;
                }
            }
        }
        page_idx = __recorder_get_next_page(page_idx);
    } while (page_idx != __recorder_get_first_page());
    
    return callback_ret;
}

uint8 __recorder_find_ts_callback(uint8 pg_idx, uint8 rec_idx, uint8 *cb_ret_ptr, struct for_task_param *task_param)
{
    int16 current_temp, next_temp;
    uint32 current_ts, next_ts, current_tail_ts;
    uint8 npage_idx;
    uint8 nrec_idx;
    int8 nentry_idx, entry_idx;    
    uint8 ret;
    
    if ((__recorder_get_sample_info(pg_idx, rec_idx, -1, &current_temp, &current_ts) == RECORDER_SUCCESS) && (current_ts <= task_param->ts)) {

        ret = __recorder_get_current_tail_next_head_info(pg_idx, rec_idx, current_ts, &current_tail_ts, &npage_idx, &nrec_idx, &next_ts);
        if (ret == RECORDER_ERROR) {
            if (__recorder_get_sample_info(pg_idx, rec_idx, (REC_DATA_PER_ENTRY - 1), &next_temp, &next_ts) != RECORDER_SUCCESS) {
                    // This should not happen, but in case of flash been erased and written 
                    // while we are doing the query
                    *cb_ret_ptr = RECORDER_ERROR;
                    return RECORDER_FOR_RETURN;
            }
        }
        if (task_param->ts < next_ts) {
            for (entry_idx = -1; entry_idx < REC_DATA_PER_ENTRY; entry_idx++) {
                if (__recorder_get_current_next_info(pg_idx, rec_idx, entry_idx, &current_temp, &current_ts, &npage_idx, &nrec_idx, &nentry_idx, &next_temp, &next_ts) != RECORDER_SUCCESS) {
                    // This should not happen, but in case of flash been erased and written 
                    // while we are doing the query
                    *cb_ret_ptr = RECORDER_ERROR;
                    return RECORDER_FOR_RETURN;
                }
                if (next_ts > task_param->ts) {
                    break;
                }
            }
            if ((entry_idx == REC_DATA_PER_ENTRY) && ((next_ts - task_param->ts) > 60)) {
                // No data sample between ts and next available ts
                *cb_ret_ptr = RECORDER_ERROR;
                return RECORDER_FOR_RETURN;
            } if (entry_idx < REC_DATA_PER_ENTRY) {
                task_param->rec_ptr->page_idx = pg_idx;
                task_param->rec_ptr->record_entry_idx = rec_idx;
                task_param->rec_ptr->data_entry_idx = entry_idx;
                task_param->rec_ptr->unix_ts = current_ts;
                                
                *cb_ret_ptr = RECORDER_SUCCESS;
                return RECORDER_FOR_RETURN;               
            } else {
                task_param->rec_ptr->page_idx = npage_idx;
                task_param->rec_ptr->record_entry_idx = nrec_idx;
                task_param->rec_ptr->data_entry_idx = nentry_idx;
                task_param->rec_ptr->unix_ts = next_ts;
                                
                *cb_ret_ptr = RECORDER_SUCCESS;
                return RECORDER_FOR_RETURN;
            }
        }
    }
    return RECORDER_FOR_CONTINUE;
}

/*
 * If we found valid sample, rec_read points to the sample right before the search point
 */
static uint8 __recorder_find_ts(UTCTimeStruct *tc, struct record_read_temperature *rec_ptr)
{
    struct for_task_param task_param;
    
    // Cleanup rec_read
    osal_memset((uint8 *)rec_ptr, 0, sizeof(struct record_read_temperature));
    
    if (tc != NULL) {
        task_param.ts = osal_ConvertUTCSecs(tc);
        task_param.rec_ptr = rec_ptr;
        return recorder_for_task(__recorder_find_ts_callback, &task_param);
    }    
    
    return RECORDER_ERROR;
}

uint8 __recorder_get_current_head_info(uint8 pg_idx, uint8 rec_idx, uint32 *ts_ptr)
{
    uint8 magic[2];
    
    HalFlashRead(pg_idx, (uint16)(rec_idx + 1) * sizeof(struct record_entry) - 2, magic, 2);
    if (magic[0] == 0x52 && magic[1] == 0x44) {
        HalFlashRead(pg_idx, (uint16)rec_idx * sizeof(struct record_entry), (uint8 *)ts_ptr, sizeof(uint32));
        return RECORDER_SUCCESS;
    } else {
        return RECORDER_ERROR;
    }        
}

uint8 __recorder_get_current_tail_next_head_info(uint8 pg_idx, uint8 rec_idx, uint32 head_ts, uint32 *tail_ts_ptr, 
                                                uint8 *npg_ptr, uint8 *nrec_ptr, uint32 *nhead_ts_ptr)
{
    uint8 magic[2];
    uint16 delta;

    // Update IDX
    if (rec_idx >= (REC_ENTRY_PER_PAGE - 1)) {
        *npg_ptr = __recorder_get_next_page(pg_idx);
        *nrec_ptr = 0;
    } else {
        *npg_ptr = pg_idx;
        *nrec_ptr = rec_idx + 1;
    }
    
    HalFlashRead(pg_idx, (uint16)(rec_idx + 1) * sizeof(struct record_entry) - 4, (uint8 *)&delta, sizeof(uint16));
    *tail_ts_ptr = head_ts + (uint32)delta;
    
    // Check NEXT magic number, current magic is always valid!
    HalFlashRead(*npg_ptr, (uint16)(*nrec_ptr + 1) * sizeof(struct record_entry) - 2, magic, 2);
    if (magic[0] == 0x52 && magic[1] == 0x44) {
        HalFlashRead(*npg_ptr, (uint16)*nrec_ptr * sizeof(struct record_entry), (uint8 *)nhead_ts_ptr, sizeof(uint32));
        return RECORDER_SUCCESS;
    } else {
        return RECORDER_ERROR;
    }        
}

uint8 __recorder_get_boundary_unix_time_callback(uint8 pg_idx, uint8 rec_idx, uint8 *cb_ret_ptr, struct for_task_param *task_param)
{
    uint32 head_ts, next_ts, current_ts;
    uint8 npage_idx;
    uint8 nrec_idx;
    
    if (__recorder_get_current_head_info(pg_idx, rec_idx, &head_ts) == RECORDER_SUCCESS) {
        if (task_param->search_mode == 1) {
            // Search for head
            if ((head_ts > task_param->last_known_ts) && (head_ts < task_param->head_ts)) {
                task_param->head_ts = head_ts;
                *cb_ret_ptr = RECORDER_SUCCESS;
            }
        } else if (task_param->search_mode == 2) {
            // Head is known, search for tail
            if (head_ts == task_param->last_known_ts) {
                uint16 cnt = 0;
                do {
                    if (__recorder_get_current_tail_next_head_info(pg_idx, rec_idx, head_ts, &current_ts, &npage_idx, &nrec_idx, &next_ts) == RECORDER_SUCCESS) {
                        if ((next_ts == 0) || (next_ts < current_ts) || ((next_ts - current_ts) > TEMP_SAMPLE_GAP_THRESHOLD)) {
                            // We got the target, page_idx/rec_idx is the last block
                            task_param->tail_ts = current_ts;
                            *cb_ret_ptr = RECORDER_SUCCESS;
                            return RECORDER_FOR_RETURN;
                        } else {
                            pg_idx = npage_idx;
                            rec_idx = nrec_idx;
                            head_ts = next_ts;
                            cnt++;
#ifndef DISABLE_WD
                            if ((cnt % 16) == 0) {
                                WD_KICK();
                            }
#endif
                        }    
                    } else if (__recorder_get_current_head_info(pg_idx, rec_idx, &current_ts) == RECORDER_SUCCESS) {
                        task_param->tail_ts = current_ts;
                        *cb_ret_ptr = RECORDER_SUCCESS;
                        return RECORDER_FOR_RETURN;
                    } else {
                        if (current_ts != 0) {
                            task_param->tail_ts = current_ts;
                            *cb_ret_ptr = RECORDER_SUCCESS;
                            return RECORDER_FOR_RETURN;    
                        } else {
                            *cb_ret_ptr = RECORDER_ERROR;
                            return RECORDER_FOR_RETURN; 
                        }
                    }                    
                } while (1); 
            }
        }        
    }    
        
    return RECORDER_FOR_CONTINUE;
}

static uint32 __recorder_get_boundary_unix_time(uint8 search_mode, uint32 last_known_ts)
{
    struct for_task_param task_param;
    uint8 ret;
    
    task_param.head_ts = 0xFFFFFFFF;
    task_param.tail_ts = 0; 
    task_param.search_mode = search_mode;
    task_param.last_known_ts = last_known_ts;
    ret = recorder_for_task(__recorder_get_boundary_unix_time_callback, &task_param);    

    if (ret == RECORDER_SUCCESS) {
        if (search_mode == 1) {
            return task_param.head_ts;
        } else if (search_mode == 2) {
            return task_param.tail_ts;
        } else {
            return RECORDER_ERROR;
        }
    } else {
        return RECORDER_ERROR;
    }
}

static uint8 __recorder_get_next_end_ts(struct record_read_temperature *rec_ptr)
{
    //uint8 ret_status = 0;   // 0 - NA; 1 - begin; 2 - end
    uint32 prev_unix_ts = rec_ptr->unix_ts;

    rec_ptr->unix_ts = __recorder_get_boundary_unix_time(qdb.query_time_interval_flag, rec_ptr->unix_ts);
    if (rec_ptr->unix_ts == RECORDER_ERROR) {
        if (qdb.query_time_interval_flag == 1) {
            if (prev_unix_ts == 0) {
                // New search, try to get the begining of time interval
                return RECORDER_ERROR;
            } else {
                // Hit end of history, rollback to begining and retry
                return __recorder_get_next_end_ts(rec_ptr);
            }
        } else if (qdb.query_time_interval_flag == 2) {
            // In case we are searching for the tail, it is likely the head
            // flash block has been erased. Back to beginning query head
            qdb.query_time_interval_flag = 1;
            return __recorder_get_next_end_ts(rec_ptr);
        }
    } else {
        // Query success
        if (qdb.query_time_interval_flag == 1) {
            qdb.query_time_interval_flag = 2;
            return 1;
        } else if (qdb.query_time_interval_flag == 2) {
            qdb.query_time_interval_flag = 1;
            return 2;
        }
    }
        
    // BUG
    rec_ptr->unix_ts = 0;
    return RECORDER_ERROR;
}

void recorder_API_task(void)
{
#ifdef TRACE_API_TIME
    uint32 time_tmp;
#endif    
    uint8 loop_cnt_max, idx;
    
#ifdef TRACE_API_TIME    
    recorder_api_task_period = halSleepReadTimer();
#endif    
    
    if (qdb.query_time_boundary_continue < 2) {        
        // Count how many fragments do we have?
        // Return at most 255
        if (qdb.query_time_boundary_continue == 0) {
            __recorder_get_next_end_ts(&rec_read);
            if (rec_read.unix_ts == 0) {
                qdb.query_time_interval = 1;
                qdb.query_time_interval_flag = 1;
                osal_memset((void *)&record_query_result_buffer, 0, sizeof(record_query_result_buffer));        
                return;
            }
            loop_cnt_max = 5;
            qdb.query_time_boundary_initial_ts = rec_read.unix_ts;
            qdb.query_time_boundary_continue = 1;
            qdb.query_time_boundary_count = 1;
        } else {
            loop_cnt_max = 6;
        }
        
        for (idx = 0; idx < loop_cnt_max; idx++) {
            if (__recorder_get_next_end_ts(&rec_read) == RECORDER_ERROR) {
                break;
            }
            qdb.query_time_boundary_count++;
            if (rec_read.unix_ts == qdb.query_time_boundary_initial_ts) {
                qdb.query_time_boundary_continue = 2;
                rec_read.page_idx = qdb.query_time_boundary_count > 510 ? 0xFF : (uint8)(qdb.query_time_boundary_count / 2);
                rec_read.unix_ts = 0;
                qdb.query_time_interval = 1;
                qdb.query_time_interval_flag = 1;
                osal_memset((void *)&record_query_result_buffer, 0, sizeof(record_query_result_buffer));
                break;
            }
        }
        iDo_schedule_recorder_API_task();
    } else {
        record_query_result_buffer.stats_mode = __recorder_get_next_end_ts(&rec_read);
        if ((rec_read.unix_ts != 0) && (rec_read.page_idx > 0)) {
            record_query_result_buffer.stats_sample_cnt = rec_read.page_idx;
            record_query_result_buffer.stats_period_0 = qdb.query_time_interval_cnt / 2;
            osal_ConvertUTCTime(&(record_query_result_buffer.tc), rec_read.unix_ts);
            qdb.query_time_interval_cnt++;
            if (qdb.query_time_interval_cnt >= ((uint16)rec_read.page_idx * 2)) {
                osal_memset((uint8 *)&qdb, 0, sizeof(qdb));
                osal_memset((uint8 *)&rec_read, 0, sizeof(struct record_read_temperature));            
                qdb.query_time_interval = 1;
                qdb.query_time_interval_flag = 1; 
            }
        } else {
            record_query_result_buffer.stats_mode = 0;            
        }
        
#ifdef TRACE_API_TIME        
        time_tmp = halSleepReadTimer();
        if (time_tmp < recorder_api_task_period) {
            time_tmp += 0x01000000;
        }
        recorder_api_task_period = time_tmp - recorder_api_task_period;   
#endif
    }
}

/*
 * Set query criteria and move rec_read to the point
 */
void recorder_set_query_criteria(struct query_criteria *query_ptr)
{
    osal_memset((uint8 *)&qdb, 0, sizeof(qdb));
    osal_memset((uint8 *)&rec_read, 0, sizeof(struct record_read_temperature));

    // Query valid sample time intervals
    if (query_ptr->stats_mode == 0xFF) {
        qdb.query_time_interval = 1;
        qdb.query_time_interval_flag = 1;
        osal_memset((void *)&record_query_result_buffer, 0xFF, sizeof(record_query_result_buffer));
        iDo_schedule_recorder_API_task();
        return;
    }

    qdb.query_mode = STATS_MAX;
    if (query_ptr->stats_mode & STATS_AVERAGE) {
        qdb.query_mode = STATS_AVERAGE;
    }
    if (query_ptr->stats_mode & STATS_MIN) {
        qdb.query_mode = STATS_MIN;
    }
    if (query_ptr->stats_mode & STATS_MAX) {
        qdb.query_mode = STATS_MAX;
    }
    qdb.query_sample_cnt = query_ptr->stats_sample_cnt;
    osal_memcpy((uint8 *)&(qdb.query_period), &(query_ptr->stats_period_0), 3);
    if (qdb.query_period > 604800) {
        qdb.query_valid = 0;
    } else {
        qdb.query_valid = __recorder_find_ts(&(query_ptr->tc), &rec_read);
        qdb.query_start_point_ts = rec_read.unix_ts;
    }
}

/*
 * Run one query
 */
void recorder_get_query_result(struct query_criteria *query_result)
{
    uint16 read_cnt;
    int16 min_temp = 0x7FFF, max_temp = -1;
    int32 acc_temp = 0;
    uint32 min_ts, max_ts;
    int16 current_temp, next_temp, result_temp;
    uint32 current_ts, next_ts, result_ts;
    UTCTimeStruct tm;

    osal_memset((uint8 *)query_result, 0, sizeof(struct query_criteria));
    
    if (qdb.query_time_interval == 1) {
        osal_memcpy((void *)query_result, (void *)&record_query_result_buffer, sizeof(record_query_result_buffer));
        if (qdb.query_time_boundary_continue >= 2) {
            iDo_schedule_recorder_API_task();
        }
    } else if (qdb.query_valid == 1) {
        read_cnt = 0;
        
        while (1) {
            uint8 pg_backup = rec_read.page_idx;
            uint8 rec_backup = rec_read.record_entry_idx;
            int8 entry_backup = rec_read.data_entry_idx;

            // Get next temp and ts
            if (__recorder_get_current_next_info(rec_read.page_idx, rec_read.record_entry_idx, rec_read.data_entry_idx, &current_temp, &current_ts, &(rec_read.page_idx), &(rec_read.record_entry_idx), &(rec_read.data_entry_idx), &next_temp, &next_ts) != RECORDER_SUCCESS) {
                return;
            }
            if ((current_ts >= next_ts) || ((next_ts - current_ts) > TEMP_SAMPLE_GAP_THRESHOLD)) {
                // Do not allow time rollback, time jump etc.
                rec_read.page_idx = pg_backup;
                rec_read.record_entry_idx = rec_backup;
                rec_read.data_entry_idx = entry_backup;
                return;
            }
            read_cnt++;
#ifndef DISABLE_WD            
            if ((read_cnt % 200) == 0) {
                WD_KICK();
            }
#endif            
            
            // Go through samples one by one
            if (qdb.query_period == 0 && qdb.query_sample_cnt == 0) {
                // Send back result one by one
                osal_ConvertUTCTime(&tm, current_ts);
#ifdef ATTACH_DETECTION                
                Temp_FinishPacket((uint8 *)query_result, current_temp, 0, &tm);
#else
                Temp_FinishPacket((uint8 *)query_result, current_temp, &tm);
#endif
                return;
            } 
            
            // Apply stats
            if (qdb.query_mode == STATS_MAX) {
                if (current_temp > max_temp) {
                    max_temp = current_temp;
                    max_ts = current_ts;
                }
            } else if (qdb.query_mode == STATS_MIN) {
                if (current_temp < min_temp) {
                    min_temp = current_temp;
                    min_ts = current_ts;
                }
            } else if (qdb.query_mode == STATS_AVERAGE) {
                acc_temp += (int32)current_temp;
            }

            if (qdb.query_sample_cnt != 0) {
                if (read_cnt == qdb.query_sample_cnt) {
                    break;
                }
            } else {
                if (current_ts >= (qdb.query_start_point_ts + qdb.query_period)) {
                    break;
                }
            }
        }
    
        // Final results
        if (qdb.query_mode == STATS_MAX) {
            result_ts = max_ts;
            result_temp = max_temp;
        } else if (qdb.query_mode == STATS_MIN) {
            result_ts = min_ts;
            result_temp = min_temp;
        } else if (qdb.query_mode == STATS_AVERAGE) {
            result_ts = qdb.query_start_point_ts;
            result_temp = (int16)(acc_temp / (uint32)read_cnt);
        }
        
        osal_ConvertUTCTime(&tm, result_ts);
#ifdef ATTACH_DETECTION        
        Temp_FinishPacket((uint8 *)query_result, result_temp, 0, &tm);
#else
        Temp_FinishPacket((uint8 *)query_result, result_temp, &tm);
#endif

        if (qdb.query_period != 0) {
            osal_ConvertUTCTime(&tm, (qdb.query_start_point_ts + qdb.query_period));
            qdb.query_valid = __recorder_find_ts(&tm, &rec_read);
            qdb.query_start_point_ts = rec_read.unix_ts;
        }
        return;
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

