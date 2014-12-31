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
#include "hci.h"

#define PWR_VOLTAGE_DELAY               5000
#define PWR_VOLTAGE_CNT                 10
#define PWR_TX_HIGH_FACTOR              1.1     // FIXME: this factor should be measured
#define PWR_RX_HIGH_FACTOR              1.1     // FIXME: measure???
#define PWR_TX_RX_HIGH_FACTOR           1.21    // FIXME
#define PWR_CONNECT_DUMMY_TX_H          55      // FIXME: need recalculate
#define PWR_CONNECT_DUMMY_TX_N          50
#define PWR_BATTERY_LIFE_PKT_CNT        16200000UL
#define PWR_BATTERY_LIFE_TEMP_SAMPLE    6660000UL

#define PWR_TIMEOUT_RCD_MAX             6
#define PWR_TIMEOUT_CHECK_PERIOD        3600    // 1 hour

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
static uint8 prxmode = 0;       // RX power normal: 0; RX power high: 1   FIXME: ignore rx factor 
static uint8 pconn_status = 0;  // Not connected: 0; Connected: 1
static uint32 pwr_timeout_ts[PWR_TIMEOUT_RCD_MAX] = {0};
/* 
 * connection interval shall be cleaned up per connection
 */
static uint32 conn_interval = 0;
static uint32 conn_latency = 0;
static uint32 adv_param = ((uint32)DEFAULT_ADVERTISING_INTERVAL * (uint32)625) / (uint32)1000;

static uint32 lostConnectionTime = 0;

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
    uint16 v;

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
    uint32 pkt_cnt;

    if (pconn_status == 0) {
        pkt_cnt = (delta * (uint32)1000) / adv_param;
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
    if (prxmode == 0) {
        pwrmgmt_glean_stats();
    } 
    prxmode = 1;
}

void pwrmgmt_rx_low(void)
{
    if (prxmode == 1) {
        pwrmgmt_glean_stats();
    }
    prxmode = 0;
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
    lostConnectionTime = 0;
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
    uint32 tmp;

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
    uint8 base_capacity, guess_capacity, calculated_capacity;
    uint8 usage, ret;
    uint16 voltage_lvl;

    if (pinit == 0) {
        return 0xFF;
    } else {
        base_capacity = battGuessCapacity(pd.initial_v_adc);
        usage = pwrmgmt_battery_usage();

        if (base_capacity > usage) {
            calculated_capacity = base_capacity - usage;
        } else {
            calculated_capacity = 0;
        }
        
        // Double check battery voltage
        ret = battGetMeasure(&voltage_lvl);
        if (ret == 1) {
            guess_capacity = battGuessCapacity(voltage_lvl);
            if ((guess_capacity == 100) && (calculated_capacity < 40)) {
                osal_memset((uint8 *)&pd, 0, sizeof(pd));
                pd.initial_v_adc = voltage_lvl;
                custom_mgmt_set(&pd);
                calculated_capacity = 100;
            } else if ((guess_capacity <= 20) && (calculated_capacity > guess_capacity)) {
                // In case battery voltage is *VERY LOW*, trust guess result
                calculated_capacity = guess_capacity;
            }
        }
        return calculated_capacity;
    }
}

void pwrmgmt_flash_dump(void)
{
    pwrmgmt_glean_stats();
    custom_mgmt_set(&pd);
}

/*
 * Dynamic tune rx power settings here, this callback will
 * be triggered every 60 secs
 */
// This is the state machine to decide when we shall lower rx power.
// Rule of thumb is: 
// if no more than PWR_TIMEOUT_RCD_MAX timeout happened during last hour,
// set rx power to normal in this callback
uint8 pwrmgmt_hb(void)
{
    uint32 ts_threshold = osal_getRelativeClock() - PWR_TIMEOUT_CHECK_PERIOD;
    uint8 cnt = 0;
    
    for (uint8 idx = 0; idx < PWR_TIMEOUT_RCD_MAX; idx++) {
        if (pwr_timeout_ts[idx] >= ts_threshold) {
           cnt++;
        } 
    }
    if ((cnt < PWR_TIMEOUT_RCD_MAX) && (prxmode == 1) && (pconn_status == 1)) {
        HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
        pwrmgmt_event(RX_LOW);
    }

    // If disconnected due to timeout after 10 minutes, lower tx/rx power
    if ((lostConnectionTime != 0) && (osal_getRelativeClock() - lostConnectionTime) >= 600) {        
        HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_23_DBM);
        pwrmgmt_event(TX_LOW);
        HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
        pwrmgmt_event(RX_LOW);        
        return 1;       // stop me!
    }
    
    return 0;   // continue
}

void pwrmgmt_timeout(void)
{
    uint32 min_ts = 0xFFFFFFFF;
    uint8 min_idx = 0;
    
    for (uint8 idx = 0; idx < PWR_TIMEOUT_RCD_MAX; idx++) {
        if (pwr_timeout_ts[idx] < min_ts) {
            min_idx = idx;
            min_ts = pwr_timeout_ts[idx];
        }
    }
    pwr_timeout_ts[min_idx] = osal_getRelativeClock();
    
    lostConnectionTime = osal_getRelativeClock();
}

