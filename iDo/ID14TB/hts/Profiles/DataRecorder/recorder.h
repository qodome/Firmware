/*
 * Data recorder keeps track of writing data into
 * flash and query for history data
 */

#ifndef __DATA_RECORDER__
#define __DATA_RECORDER__

#include "OSAL_Clock.h"

#define REC_ENTRY_PER_PAGE      (2048 / 256)
#define REC_DATA_PER_ENTRY      123
#define REC_DATA_ENTRY_OFFSET   6

#define RECORDER_ERROR          0
#define RECORDER_SUCCESS        1

struct record_read_temperature {
    uint8 page_idx;
    uint8 record_entry_idx;
    int8 data_entry_idx;
    uint32 unix_ts;
};

struct for_task_param {
    uint32 ts;
    struct record_read_temperature *rec_ptr;
    uint8 search_mode;
    uint32 last_known_ts;
    uint32 head_ts;
    uint32 tail_ts;
};

struct record_position {
    int8 page_idx;
    int8 record_entry_idx;
};

struct data_entry {
    uint8     delta_time_temp_h;
    uint8     delta_temp_l;
};

struct record_entry {
    uint32 unix_time;
    int16 base_temp;
    struct data_entry entries[REC_DATA_PER_ENTRY];
    uint16 end_delta;
    uint8 magic[2];
};

#define STATS_MAX               0x10
#define STATS_MIN               0x20
#define STATS_AVERAGE           0x40
#define STATS_DEFAULT           STATS_MAX
#define QUERY_MAX_SEGMENT_CNT   255

struct query_criteria {    
    uint8 stats_mode;
    uint8 stats_sample_cnt;
    uint8 stats_period_0;
    uint8 stats_period_1;
    uint8 stats_period_2;
    UTCTimeStruct tc;
};

struct query_db {
    uint8 query_time_interval;
    uint8 query_time_interval_flag;
    uint16 query_time_interval_cnt;
    uint8 query_valid;
    uint8 query_mode;
    uint8 query_sample_cnt;
    uint32 query_period;
    uint32 query_start_point_ts;
};

void recorder_init(void);
void recorder_add_temperature(int16 temp);
void recorder_set_query_criteria(struct query_criteria *query_ptr);
void recorder_get_query_result(struct query_criteria *query_result);

void recorder_touch_wd_page(void);
uint32 recorder_get_wd_boot_count(void);
void recorder_API_task(void);

#endif
