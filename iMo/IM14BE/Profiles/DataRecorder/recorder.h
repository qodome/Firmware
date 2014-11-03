/*
 * Data recorder keeps track of writing data into
 * flash and query for history data
 */

#ifndef __DATA_RECORDER__
#define __DATA_RECORDER__

#define REC_ENTRY_PER_PAGE (2048 / 256)
#define REC_DATA_PER_ENTRY  125
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
    struct data_entry entries[125];
};

struct calendar {
    uint16 year; 
    uint8 month; 
    uint8 day;
    uint8 hours; 
    uint8 minutes; 
    uint8 seconds;
};

struct debug_rec {
    int8 read_page_idx;
    int8 read_record_entry_idx;
    int8 read_data_entry_idx;
    int8 write_page_idx;
    int8 write_record_entry_idx;
    int8 write_data_entry_idx;
    uint32 last_read_ts;
    int16 last_read_temp;
    uint32 current_ts;
    uint8 sleepsta;
};

void recorder_init(void);
void recorder_add_temperature(int16 temp);
void recorder_rtc_tick(void);
void recorder_set_read_base_ts(struct calendar *tc);
int16 recorder_get_temperature(struct calendar *tc_out);
void recorder_get_calendar_time(struct calendar *tc);
void recorder_set_calendar_time(struct calendar *tc);
uint32 recorder_get_debug_info(void);
void recorder_get_debug_rec(struct debug_rec* dbg);

#endif
