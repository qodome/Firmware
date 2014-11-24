/*
 * Data recorder keeps track of writing data into
 * flash and query for history data
 */

#ifndef __DATA_RECORDER__
#define __DATA_RECORDER__

#include "OSAL_Clock.h"

#define REC_ENTRY_PER_PAGE (2048 / 256)
#define REC_DATA_PER_ENTRY  124
#define REC_DATA_ENTRY_OFFSET   6

struct record_read_temperature {
    int8 page_idx;
    int8 record_entry_idx;
    int8 data_entry_idx;
    uint32 last_data_entry_unix_ts;
    int16 last_data_entry_temp;
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
    uint8 magic[2];
};

void recorder_init(void);
void recorder_add_temperature(int16 temp);
void recorder_set_read_base_ts(UTCTimeStruct *tc);
int16 recorder_get_temperature(UTCTimeStruct *tc_out);

void recorder_touch_wd_page(void);
uint32 recorder_get_wd_boot_count(void);

#endif
