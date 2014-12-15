/*
 * Data recorder keeps track of writing data into
 * flash and query for history data
 */

#ifndef __DATA_RECORDER__
#define __DATA_RECORDER__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"

#define REC_ENTRY_PER_PAGE (1024 / 256)
#define REC_DATA_PER_ENTRY  82
#define REC_DATA_ENTRY_OFFSET   8

struct record_read_temperature {
    uint8_t page_idx;
    uint8_t record_entry_idx;
    int8_t data_entry_idx;
    uint32_t last_data_entry_unix_ts;
    int16_t last_data_entry_temp;
};

struct record_position {
    uint8_t page_idx;
    uint8_t record_entry_idx;
} __attribute__((packed));

struct data_entry {
    uint8_t     delta_time_temp_h;
    uint8_t     delta_temp_l;
    int8_t     rssi;
} __attribute__((packed));

struct record_entry {
    uint32_t unix_time;
    int16_t base_temp;
    int8_t rssi;
    uint8_t dummy;
    struct data_entry entries[REC_DATA_PER_ENTRY];
    uint8_t magic[2];
} __attribute__((packed));

void recorder_radio_notification_evt_handler_t(bool radio_active);
void recorder_init(void);
void recorder_add_temperature(int16_t temp);
void recorder_set_read_base_ts(ble_date_time_t *tc);
int16_t recorder_get_temperature(ble_date_time_t *tc_out, int8_t *p_rssi);

void recorder_touch_wd_page(void);
uint32_t recorder_get_wd_boot_count(void);

#endif
