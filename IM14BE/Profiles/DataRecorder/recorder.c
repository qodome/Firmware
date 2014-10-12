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

#if defined(HAL_IMAGE_A) || defined(HAL_IMAGE_B)

// Member variables
struct record_position rec_head;
struct record_position rec_next_write;
struct record_read_temperature rec_read;
struct record_entry rec_latest;
uint32 latest_sample_unix_ts;
int16 latest_sample_temp;
int8 rec_latest_entry_idx = -1;
static uint32 unix_ts = 0;
static uint8 last_st1_h = 0;

// magic unix time and date time relation
// recorded on 2014-1-1 00:00:00
// unix time: 1388505600 
uint32 __magic_calendar_ts_year_month[10*12] = {
1388505600, 1391184000, 1393603200, 1396281600, 1398873600, 1401552000, 1404144000, 1406822400, 1409500800, 1412092800, 1414771200, 1417363200, 
1420041600, 1422720000, 1425139200, 1427817600, 1430409600, 1433088000, 1435680000, 1438358400, 1441036800, 1443628800, 1446307200, 1448899200, 
1451577600, 1454256000, 1456761600, 1459440000, 1462032000, 1464710400, 1467302400, 1469980800, 1472659200, 1475251200, 1477929600, 1480521600, 
1483200000, 1485878400, 1488297600, 1490976000, 1493568000, 1496246400, 1498838400, 1501516800, 1504195200, 1506787200, 1509465600, 1512057600, 
1514736000, 1517414400, 1519833600, 1522512000, 1525104000, 1527782400, 1530374400, 1533052800, 1535731200, 1538323200, 1541001600, 1543593600, 
1546272000, 1548950400, 1551369600, 1554048000, 1556640000, 1559318400, 1561910400, 1564588800, 1567267200, 1569859200, 1572537600, 1575129600, 
1577808000, 1580486400, 1582992000, 1585670400, 1588262400, 1590940800, 1593532800, 1596211200, 1598889600, 1601481600, 1604160000, 1606752000, 
1609430400, 1612108800, 1614528000, 1617206400, 1619798400, 1622476800, 1625068800, 1627747200, 1630425600, 1633017600, 1635696000, 1638288000, 
1640966400, 1643644800, 1646064000, 1648742400, 1651334400, 1654012800, 1656604800, 1659283200, 1661961600, 1664553600, 1667232000, 1669824000, 
1672502400, 1675180800, 1677600000, 1680278400, 1682870400, 1685548800, 1688140800, 1690819200, 1693497600, 1696089600, 1698768000, 1701360000,
};
struct calendar last_tc;
uint32 last_ts = 0;

//Timestamp debug
static uint32 debug_info = 0;

// Extern function declarations
extern uint32 halSleepReadTimer(void);

/*
 * Return UNIX time from calendar
 */
static uint32 __recorder_get_unix_ts_from_calendar(struct calendar *tc)
{
    uint32 ret_ts = 0;

    if ((tc->year < 2014 || tc->year >= 2024) ||
        (tc->month < 1 || tc->month > 12) ||
        (tc->day < 1 || tc->day > 31) ||
        (tc->hours > 23 || tc->minutes > 59 || tc->seconds > 59)) {
        return 0;
    }
    ret_ts = __magic_calendar_ts_year_month[(tc->year - 2014) * 12 + (tc->month - 1)] + ((uint32)tc->day - 1) * 86400 + ((uint32)tc->hours * 3600) + ((uint32)tc->minutes * 60) + (uint32)tc->seconds;
    return ret_ts;
}

static void __recorder_get_calendar_from_unix_ts(uint32 ts, struct calendar *tc)
{
    int idx = 0;
    uint8 delta_m = 0, delta_s = 0;

    if (last_ts != 0 && last_ts < ts && (ts - last_ts) < 3600) {
        // calculate based on last_ts
        delta_m = (ts - last_ts) / 60;
        delta_s = (ts - last_ts) - (uint32)delta_m * 60;
        tc->year = last_tc.year;
        tc->month = last_tc.month;
        tc->day = last_tc.day;
        tc->hours = last_tc.hours;
        tc->minutes = last_tc.minutes;
        tc->seconds = last_tc.seconds;
        tc->seconds += delta_s;
        if (tc->seconds > 59) {
            tc->seconds -= 60;
            tc->minutes += 1;
            if (tc->minutes > 59) {
                tc->minutes -= 60;
                tc->hours += 1;
                if (tc->hours > 23) {
                    goto recalculate;
                }
            }
        }
        tc->minutes += delta_m;
        if (tc->minutes > 59) {
            tc->minutes -= 60;
            tc->hours += 1;
            if (tc->hours > 23) {
                goto recalculate;
            }
        }
        last_ts = ts;
        last_tc.year = tc->year;
        last_tc.month = tc->month;
        last_tc.day = tc->day;
        last_tc.hours = tc->hours;
        last_tc.minutes = tc->minutes;
        last_tc.seconds = tc->seconds;
    } else {
recalculate:

        // search for the nearest year/month
        for (idx = 0; idx < 120; idx++) {
            if ((ts >= __magic_calendar_ts_year_month[idx]) && 
                ((idx <119) && ts < __magic_calendar_ts_year_month[idx + 1])) {
                break;
            }
        }
        if (idx >= 120) {
            tc->year = 0;
            tc->month = 0;
            tc->day = 0;
            tc->hours = 0;
            tc->minutes = 0;
            tc->seconds = 0;
            return;
        }

        tc->year = 2014 + (idx / 12);
        tc->month = 1 + (idx % 12);
        tc->day = 1 + (ts - __magic_calendar_ts_year_month[idx]) / 86400;
        tc->hours = (ts - __magic_calendar_ts_year_month[idx] - ((uint32)tc->day - 1) * 86400) / 3600;
        tc->minutes = (ts - __magic_calendar_ts_year_month[idx] - ((uint32)tc->day - 1) * 86400 - (uint32)tc->hours * 3600) / 60;
        tc->seconds = ts - __magic_calendar_ts_year_month[idx] - ((uint32)tc->day - 1) * 86400 - (uint32)tc->hours * 3600 - (uint32)tc->minutes * 60;
        last_ts = ts;
        last_tc.year = tc->year;
        last_tc.month = tc->month;
        last_tc.day = tc->day;
        last_tc.hours = tc->hours;
        last_tc.minutes = tc->minutes;
        last_tc.seconds = tc->seconds;
    }
}

static uint8 __recorder_get_first_page(void)
{
#if defined(HAL_IMAGE_A)
    return 8;
#else
    return 1;
#endif
}

/*
 * Get the next page idx
 */
static uint8 __recorder_get_next_page(uint8 pg_idx)
{
#if defined(HAL_IMAGE_A)
    if (pg_idx == 69) {
        return 8;
    }
#else
    if (pg_idx == 7) {
        return 70;
    } else if (pg_idx == 124) {
        return 1;
    }
#endif

    return (pg_idx + 1);
}

static int __recorder_get_record_buf(int8 page_idx, int8 record_entry_idx, int8 data_entry_idx, uint8 *buf_out)
{
    int ret = 0;

    if (rec_next_write.page_idx == page_idx && 
        rec_next_write.record_entry_idx == record_entry_idx) {
        if (rec_latest_entry_idx > data_entry_idx) {
            if (data_entry_idx == -1) {
                osal_memcpy(buf_out, (uint8 *)&rec_latest, 6);
            } else {
                osal_memcpy(buf_out, (uint8 *)&(rec_latest.entries[data_entry_idx]), sizeof(struct data_entry));
            }
        } else {
            ret = -1;
        }
    } else {
        if (data_entry_idx == -1) {
            HalFlashRead(page_idx, (int16)record_entry_idx * sizeof(struct record_entry), buf_out, 6);
        } else {
            HalFlashRead(page_idx, (int16)record_entry_idx * sizeof(struct record_entry) + REC_DATA_ENTRY_OFFSET + (int16)data_entry_idx * sizeof(struct data_entry), 
                        buf_out, sizeof(struct data_entry));
        }
    }
    return ret;
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

#if defined(HAL_IMAGE_A)
    /*
     * HAL_IMAGE_A is defined over:
     * OAD_IMG_A_PAGES: 1~7, 70~124 
     * OAD_IMG_A_TOTAL_PAGE_COUNT: 62
     *
     * The available space for IMAGE_A is:
     * page 8~69
     */
    for (int page_idx = 8; (page_idx <= 69 && available_page_idx == -1); page_idx++) {
       HalFlashRead(page_idx, 0, buf, 4);
       if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) {
           available_page_idx = page_idx;
       }
    }
    if (available_page_idx == -1) {
        available_page_idx = 8;
    }
#else
    /*
     * HAL_IMAGE_B is defined over:
     * OAD_IMG_B_PAGES: 8~69
     * OAD_IMG_A_TOTAL_PAGE_COUNT: 62
     *
     * The available space for IMAGE_B is:
     * page 1~7, 70~124
     */
    for (int page_idx = 1; (page_idx <= 7 && available_page_idx == -1); page_idx++) {
       HalFlashRead(page_idx, 0, buf, 4);
       if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) {
           available_page_idx = page_idx;
       }
    }
    for (int page_idx = 70; (page_idx <= 124 && available_page_idx == -1); page_idx++) {
       HalFlashRead(page_idx, 0, buf, 4);
       if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) {
           available_page_idx = page_idx;
       }
    }
    if (available_page_idx == -1) {
        available_page_idx = 1;
    }
#endif
    HalFlashErase(available_page_idx);
    HalFlashErase(__recorder_get_next_page(available_page_idx));
    rec_head.page_idx = available_page_idx;
    rec_head.record_entry_idx = -1;
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

    if (unix_ts == 0) {
        // If wall time is not initialized yet, skip recording
        return;
    }

    ts = unix_ts;

    if (rec_latest_entry_idx == -1) {
        rec_latest.unix_time = ts;
        rec_latest.base_temp = temp;
        rec_latest_entry_idx++;
    } else {
        rec_latest.entries[rec_latest_entry_idx].delta_time_temp_h = 
            (((uint8)((ts - latest_sample_unix_ts) & 0x1F)) << 3) | 
            ((uint8)(((temp - latest_sample_temp) >> 8) & 0x07));
        rec_latest.entries[rec_latest_entry_idx].delta_temp_l = 
            ((uint8)((temp - latest_sample_temp) & 0xFF));

        rec_latest_entry_idx++;
        if (rec_latest_entry_idx >= REC_DATA_PER_ENTRY) {
            // Latest record full, dump into flash
            addr = ((uint32)rec_next_write.page_idx * 2048 + (uint32)rec_next_write.record_entry_idx * sizeof(struct record_entry)) / 4;
            HalFlashWrite(addr, (uint8 *)&rec_latest, (sizeof(struct record_entry)/4));

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

    latest_sample_unix_ts = ts;
    latest_sample_temp = temp;
}

void recorder_rtc_tick(void)
{
    uint32 sleepTimeRegister = 0;

    if (unix_ts == 0) {
        // If wall time is not initialized yet, skip RTC update
        return;
    }

    sleepTimeRegister = halSleepReadTimer();

    if (((uint8 *)&sleepTimeRegister)[1] & 0x80) {
        if (last_st1_h != 0x80) {
            last_st1_h = 0x80;
            unix_ts++;
        }
    } else {
        if (last_st1_h == 0x80) {
            last_st1_h = 0x00;
            unix_ts++;
        }
    }
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
int16 recorder_get_temperature(struct calendar *tc_out)
{
    int16 ret_temp = 0, delta_temp = 0;
    uint8 buf[16];
    uint32 tmp_ts;

    // If data_entry_idx is [0, 124], then
    // current data entry must be valid
    if (rec_read.data_entry_idx >= 0 && 
        rec_read.data_entry_idx <= 124) {
        if (__recorder_get_record_buf(rec_read.page_idx, rec_read.record_entry_idx, rec_read.data_entry_idx, buf) != 0) {
            ret_temp = 0;
            return ret_temp;
        }
        if (buf[0] & 0x04) {
            // negative
            delta_temp = (((uint16)(buf[0] & 0x07) << 8) | 0xF800) | buf[1];
        } else {
            // positive
            delta_temp = ((uint16)(buf[0] & 0x07) << 8) | buf[1];
        }
        ret_temp = rec_read.last_data_entry_temp + delta_temp; 
        __recorder_get_calendar_from_unix_ts(rec_read.last_data_entry_unix_ts + ((buf[0] >> 3) & 0x1F), tc_out);
        rec_read.last_data_entry_unix_ts = rec_read.last_data_entry_unix_ts + ((buf[0] >> 3) & 0x1F);
        rec_read.last_data_entry_temp = ret_temp;
        rec_read.data_entry_idx++;
        if (rec_read.data_entry_idx > 124) {
            rec_read.data_entry_idx = -1;
            rec_read.record_entry_idx++;
            if (rec_read.record_entry_idx >= REC_ENTRY_PER_PAGE) {
                rec_read.record_entry_idx = 0;
                rec_read.page_idx = __recorder_get_next_page(rec_read.page_idx);
            }
        }
    } else if (rec_read.data_entry_idx == -1) {
        // The first data entry on the none-first record
        if (__recorder_get_record_buf(rec_read.page_idx, rec_read.record_entry_idx, rec_read.data_entry_idx, buf) != 0) {
            return 0;
        }
        ret_temp = ((uint16)buf[5] << 8) | buf[4];
        tmp_ts = (uint32)buf[3] << 24 | (uint32)buf[2] << 16 | (uint32)buf[1] << 8 | (uint32)buf[0];
        if (rec_read.last_data_entry_unix_ts != 0) {
            // For none-first record, check to make sure time is increased over last sample
            if (rec_read.last_data_entry_unix_ts > tmp_ts) {
                return 0;
            }
        } else {
            if (tmp_ts < __magic_calendar_ts_year_month[0] && tmp_ts > __magic_calendar_ts_year_month[119]) {
                return 0;
            }
        }
        rec_read.last_data_entry_temp = ret_temp;
        rec_read.last_data_entry_unix_ts = tmp_ts;
        __recorder_get_calendar_from_unix_ts(rec_read.last_data_entry_unix_ts, tc_out);
        rec_read.data_entry_idx = 0;
    } else if (rec_read.data_entry_idx > 124) {
        // BUG
        ret_temp = 0;
    }

    return ret_temp;
}

/*
 * Modify read pointer based on input
 */
void recorder_set_read_base_ts(struct calendar *tc)
{
    uint32 ts = 0;
    uint8 page_idx = 0, last_page_idx = 0;
    uint8 rec_idx = 0, last_rec_idx = 0;
    uint8 check_previous = 0;
    int skip_check = 1;
    uint8 buf[4];

    ts = __recorder_get_unix_ts_from_calendar(tc);
    // Search the ts over all flash pages
    for (page_idx = __recorder_get_first_page(); (skip_check == 1 || page_idx != __recorder_get_first_page()); page_idx = __recorder_get_next_page(page_idx)) {
        skip_check = 0;
        for (rec_idx = 0; rec_idx < REC_ENTRY_PER_PAGE; rec_idx++) {
            HalFlashRead(page_idx, (uint16)rec_idx * sizeof(struct record_entry), buf, 4);
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
}

void recorder_get_calendar_time(struct calendar *tc)
{
    __recorder_get_calendar_from_unix_ts(unix_ts, tc);
}

void recorder_set_calendar_time(struct calendar *tc)
{
    uint32 ts = __recorder_get_unix_ts_from_calendar(tc);

    halIntState_t intState;
    
    HAL_ENTER_CRITICAL_SECTION(intState);

    unix_ts = ts;

    HAL_EXIT_CRITICAL_SECTION(intState);
}

uint32 recorder_get_debug_info(void)
{
    return debug_info;
}

void recorder_get_debug_rec(struct debug_rec* dbg)
{
    dbg->read_page_idx = rec_read.page_idx;
    dbg->read_record_entry_idx = rec_read.record_entry_idx;
    dbg->read_data_entry_idx = rec_read.data_entry_idx;
    dbg->write_page_idx = rec_next_write.page_idx;
    dbg->write_record_entry_idx = rec_next_write.record_entry_idx;
    dbg->write_data_entry_idx = rec_latest_entry_idx;
    dbg->last_read_ts = rec_read.last_data_entry_unix_ts;
    dbg->last_read_temp = rec_read.last_data_entry_temp;
    dbg->current_ts = unix_ts;
    dbg->sleepsta = SLEEPSTA;
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

