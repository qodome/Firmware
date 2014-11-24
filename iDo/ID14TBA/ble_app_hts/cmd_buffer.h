/*
 * cmd buffer is used to store temperature measurement data
 */
#ifndef __CMD_BUFFER__
#define __CMD_BUFFER__

// #include "recorder.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51.h"
#include "ble.h"
#include "app_util.h"
#include "ble_date_time.h"
#include "cmd_buffer.h"

#define TIME_VALID_FLAG		0x01
#define TYPE_VALID_FLAG		0x02

#ifdef DEBUG_STATS
#define CB_CIRCULAR_BUFFER_DEPTH 16
#else
#define CB_CIRCULAR_BUFFER_DEPTH 32
#endif

// status bit flags
#define CB_WRITE_DONE           0x01

typedef struct temp_cache_s {
    int16_t tempValue;
    uint8_t validFlag;
    ble_date_time_t timeTick;
}temp_cache_t;
    
typedef struct cmd_buffer_s {
    uint8_t status_flag;                       // Index of the bit within the Byte;
    temp_cache_t t_buf;
}cmd_buffer_t;

typedef struct cmd_buffer_stats_s {
    uint16_t push_samples;
    uint16_t pull_samples;
    uint16_t overflow_counts;
    uint16_t underflow_counts;
    uint16_t bug_count;
}cmd_buffer_stats_t;


/*
 * Initialize Command Buffer
 */
void cmd_buffer_init(void);

/*
 * Do cleanup when new peer connected
 */
void cmd_buffer_clean_up(void);

/*
 * Invoked by USB TX to get the next cmd buffer
 * for USB transaction
 */
cmd_buffer_t *CBGetNextBufferForTX (int16_t *tempPtr, uint8_t *flag, ble_date_time_t *tempTSPtr);

/*
 * Manage ref_cnt
 */
void cmd_put_buffer(cmd_buffer_t *pCb);

/*
 * Invoked by sampling routine to push sampled data
 */
void cmd_buffer_push_temp(int16_t temp);

/*
 * Check to see if there is data available in the buffer
 */
uint8_t cmd_buffer_data_avalid(void);

void cmd_buffer_query_status(uint8_t *ptr);

#endif
