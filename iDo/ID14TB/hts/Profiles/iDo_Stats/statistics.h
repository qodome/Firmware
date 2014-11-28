/*
 * Statistics profile
 */
#ifdef DEBUG_STATS

#ifndef __STATISTICS__
#define __STATISTICS__

#include "hal_types.h"
#include "bcomdef.h"
#include "recorder.h"

struct stats_t {
    uint32 adv_seconds;
    uint32 connected_ios_seconds;
    uint32 adt_read_count;
    uint32 wd_boot_count;    
    uint32 power_on_ticks;
    uint32 pkt_count;
    uint32 connected_android_seconds;
    uint32 connected_default_seconds;
    uint8 adt_abnormal_cnt;
    uint32 temp_measure_cycle;
    uint32 intermediate_cycle;
    uint32 power_on_ticks_max;
    uint32 power_on_ticks_max_count;
    uint16 pwr_hold_timeout_cnt;
    uint16 sleep_hold_timeout_cnt; 
};

extern struct stats_t s;

void Stats_Init();

#endif

#endif
