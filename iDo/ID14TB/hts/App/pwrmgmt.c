/*
 * Power management and battery capacity estimation
 */
#include "pwrmgmt.h"
#include "hal_adc.h"
#include "customize.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "bcomdef.h"
#include "battservice.h"
#include "iDo.h"

#define PWR_VOLTAGE_DELAY               5000
#define PWR_VOLTAGE_CNT                 10
#define PWR_TX_HIGH_FACTOR              1.1     // FIXME: this factor should be measured
#define PWR_CONNECT_DUMMY_TX_H          55      // FIXME: need recalculate
#define PWR_CONNECT_DUMMY_TX_N          50
#define PWR_BATTERY_LIFE_PKT_CNT        16200000UL
#define PWR_BATTERY_LIFE_TEMP_SAMPLE    810000000UL // FIXME: need measure

/*
 * iOS connect: 50 packets single direction
 * Android connect: 
 */

static struct pwrmgmt_data pd;
static uint8 pcnt = PWR_VOLTAGE_CNT;
static uint32 pvsum = 0;
static uint16 pvavg = 0;
static uint8 pinit = 0;
static uint32 ptlast = 0;
static uint8 ptxmode = 0;       // TX power low: 0; TX power high: 1
//static uint8 prxmode = 0;       // RX power normal: 0; RX power high: 1   FIXME: ignore rx factor 
static uint8 pconn_status = 0;  // Not connected: 0; Connected: 1
/* 
 * connection interval shall be cleaned up per connection
 */
static uint32 conn_interval = 0;
static uint32 conn_latency = 0;

extern void iDo_ScheduleCheckVDD(uint16 delay);
extern void iDo_StopVDDCheck(void);

void pwrmgmt_tx_high(void);
void pwrmgmt_tx_low(void);
void pwrmgmt_rx_high(void);
void pwrmgmt_rx_low(void);
void pwrmgmt_connect(void);
void pwrmgmt_disconnect(void);
void pwrmgmt_temp_measure(void);

static pwrmgmt_callback_t pwr_call[PWR_MAX] = {
    pwrmgmt_tx_high,
    pwrmgmt_tx_low,
    pwrmgmt_rx_high,
    pwrmgmt_rx_low,
    pwrmgmt_connect,
    pwrmgmt_disconnect,
    pwrmgmt_temp_measure,
};

void pwrmgmt_init(void)
{
    if (custom_mgmt_initialized() == 1 && custom_mgmt_get(&pd) == 0) {
        pinit = 1;
    } else {
        osal_memset((uint8 *)&pd, 0, sizeof(pd));
        pinit = 0;
        pcnt = PWR_VOLTAGE_CNT;
        pvsum = 0;
        // Initial delay: 5s
        iDo_ScheduleCheckVDD(10000);
    }
    ptlast = osal_getRelativeClock();
}

void pwrmgmt_checkvdd_callback(void)
{
    uint16 v = 0;

    if (battGetMeasure(&v) == 1) {
        pvsum += v;
        pcnt--;
    }
    if (pcnt > 0) {
        iDo_ScheduleCheckVDD(PWR_VOLTAGE_DELAY);
    } else {
        iDo_StopVDDCheck();
        pvavg = (uint16)(pvsum / PWR_VOLTAGE_CNT);

        pd.initial_v_adc = pvavg;
        pinit = 1;

        custom_mgmt_set(&pd);
    }
}

static void pwrmgmt_glean_stats(void)
{
    uint32 now = osal_getRelativeClock();
    uint32 delta = now - ptlast;
    uint32 pkt_cnt = 0;
    uint32 tmp = 0;

    if (pconn_status == 0) {
        tmp = ((uint32)DEFAULT_ADVERTISING_INTERVAL * (uint32)625) / (uint32)1000;
        pkt_cnt = (delta * (uint32)1000) / tmp;
        if (ptxmode == 1) {
            pkt_cnt = (uint32)((float)pkt_cnt * (float)PWR_TX_HIGH_FACTOR);
        }
        pd.adv_pkt_cnt += pkt_cnt;
    } else {
        if (conn_interval > 0) {
            pkt_cnt = (delta * (uint32)800) / (conn_interval * (conn_latency + 1));
            if (ptxmode == 1) {
                pkt_cnt = (uint32)((float)pkt_cnt * (float)PWR_TX_HIGH_FACTOR);
            }
            pd.connected_pkt_cnt += pkt_cnt;
        }
    }

    // Update time
    ptlast = osal_getRelativeClock();
}

/*
 * If status changed, log result in pd
 */

void pwrmgmt_tx_high(void)
{
    if (ptxmode == 0) {
        pwrmgmt_glean_stats();
    } 
    ptxmode = 1;
}

void pwrmgmt_tx_low(void)
{
    if (ptxmode == 1) {
        pwrmgmt_glean_stats();
    }
    ptxmode = 0;
}

void pwrmgmt_rx_high(void)
{
    // FIXME
}

void pwrmgmt_rx_low(void)
{
    // FIXME
}

void pwrmgmt_connect(void)
{
    pwrmgmt_glean_stats();
    pd.connect_cnt++;
    if (ptxmode == 1) {
        pd.connected_pkt_cnt += PWR_CONNECT_DUMMY_TX_H;
    } else {
        pd.connected_pkt_cnt += PWR_CONNECT_DUMMY_TX_N;
    }
    pconn_status = 1;
}

void pwrmgmt_disconnect(void)
{
    pwrmgmt_glean_stats();

    // Disconnect cleanups 
    pconn_status = 0;
    conn_interval = 0;
    conn_latency = 0;    
}

void pwrmgmt_temp_measure(void)
{
    pd.temp_sample_cnt++;    
}

void pwrmgmt_event(uint8 evt)
{
    if (evt <= PWR_MAX) {
        (*pwr_call[evt])();
    }
}

void pwrmgmt_set_conn_param(uint16 interval, uint16 latency)
{
    conn_interval = (uint32)interval;
    conn_latency = (uint32)latency;
}

uint8 pwrmgmt_battery_usage(void)
{
    uint8 percent = 0;
    uint32 tmp = 0;

    if ((pd.adv_pkt_cnt + pd.connected_pkt_cnt) > 0) {
        tmp = PWR_BATTERY_LIFE_PKT_CNT / (pd.adv_pkt_cnt + pd.connected_pkt_cnt);
        percent += (uint8)((uint32)100 / tmp);
    }
    if (pd.temp_sample_cnt > 0) {
        tmp = PWR_BATTERY_LIFE_TEMP_SAMPLE / pd.temp_sample_cnt;
        percent += (uint8)((uint32)100 / tmp);
    }

    return percent;
}

uint8 pwrmgmt_battery_percent(void)
{
    uint8 base_capacity = 0;
    uint8 usage = 0;

    if (pinit == 0) {
        return 0xFF;
    } else {
        base_capacity = battGuessCapacity(pd.initial_v_adc);
        usage = pwrmgmt_battery_usage();
        if (base_capacity > usage) {
            return base_capacity - usage;
        } else {
            // FIXME: need check battery level and recalculate ?
            return 0;
        }
    }
}

void pwrmgmt_flash_dump(void)
{
    pwrmgmt_glean_stats();
    custom_mgmt_set(&pd);
}
