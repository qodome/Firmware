/*
 * Power management and battery capacity estimation
 */
#include <string.h>
#include "pwrmgmt.h"
#include "ble_time.h"
#include "persistent.h"
#include "battery.h"
#include "app_adv.h"

#define PWR_VOLTAGE_DELAY               5000
#define PWR_VOLTAGE_CNT                 10
#ifdef OPTIMIZE_POWER
#define PWR_TX_HIGH_FACTOR              1.1     // FIXME: this factor should be measured
#define PWR_CONNECT_DUMMY_TX_H          55      // FIXME: need recalculate
#endif
#define PWR_CONNECT_DUMMY_TX_N          50
#define PWR_BATTERY_LIFE_PKT_CNT        16200000UL
#define PWR_BATTERY_LIFE_TEMP_SAMPLE    6660000UL

#define PWR_TIMEOUT_RCD_MAX             6
#define PWR_TIMEOUT_CHECK_PERIOD        3600    // 1 hour

struct pwrmgmt_data pd;
uint8 pcnt = PWR_VOLTAGE_CNT;
uint32 pvsum = 0;
uint16 pvavg = 0;
uint8 pinit_done = 0;
uint32 ptlast = 0;
uint8 pconn_status = 0;  // Not connected: 0; Connected: 1
#ifdef OPTIMIZE_POWER
uint8 ptxmode = 0;       // TX power low: 0; TX power high: 1
uint32 pwr_timeout_ts[PWR_TIMEOUT_RCD_MAX] = {0};
#endif
/* 
 * connection interval shall be cleaned up per connection
 */
uint32 conn_interval = 0;
uint32 conn_latency = 0;
uint32 adv_param = ((uint32)APP_ADV_INTERVAL * (uint32)625) / (uint32)1000;

extern void schedule_vdd_check(uint16_t delay);
extern void iDo_StopVDDCheck(void);

#ifdef OPTIMIZE_POWER
void pwrmgmt_tx_high(void);
void pwrmgmt_tx_low(void);
#endif
void pwrmgmt_connect(void);
void pwrmgmt_disconnect(void);
void pwrmgmt_temp_measure(void);

static pwrmgmt_callback_t pwr_call[PWR_MAX] = {
#ifdef OPTIMIZE_POWER
    pwrmgmt_tx_high,
    pwrmgmt_tx_low,
#endif
    pwrmgmt_connect,
    pwrmgmt_disconnect,
    pwrmgmt_temp_measure,
};

void pwrmgmt_init(void)
{
	// Do we have pwr management records in persistent storage?
	if (persistent_pwrmgmt_get_latest(&pd) == SUCCESS) {
		pinit_done = 1;
	} else {
        memset((uint8 *)&pd, 0, sizeof(pd));
        pinit_done = 0;
        pcnt = PWR_VOLTAGE_CNT;
        pvsum = 0;
        // Initial delay: 10s
        schedule_vdd_check(10000);
	}
    ptlast = date_time_get_relative();
}

void pwrmgmt_checkvdd_callback(void)
{
    uint16 v;

    battery_request_measure();
    v = battery_get_last_measure();
    if (v > 0) {
        pvsum += v;
        pcnt--;
    }
    if (pcnt > 0) {
    	schedule_vdd_check(PWR_VOLTAGE_DELAY);
    } else {
        pvavg = (uint16)(pvsum / PWR_VOLTAGE_CNT);

        pd.initial_v_adc = pvavg;
        pinit_done = 1;

        pwrmgmt_flash_dump();
    }
}

static void pwrmgmt_glean_stats(void)
{
    uint32 now = date_time_get_relative();
    uint32 delta = now - ptlast;
    uint32 pkt_cnt;

    if (pconn_status == 0) {
        pkt_cnt = (delta * (uint32)1000) / adv_param;
#ifdef OPTIMIZE_POWER
        if (ptxmode == 1) {
            pkt_cnt = pkt_cnt * (uint32)11 / (uint32)10;
        }
#endif
        pd.adv_pkt_cnt += pkt_cnt;
    } else {
        if (conn_interval > 0) {
            pkt_cnt = (delta * (uint32)800) / (conn_interval * (conn_latency + 1));
#ifdef OPTIMIZE_POWER
            if (ptxmode == 1) {
                pkt_cnt = pkt_cnt * (uint32)11 / (uint32)10;
            }
#endif
            pd.connected_pkt_cnt += pkt_cnt;
        }
    }

    // Update time
    ptlast = date_time_get_relative();
}

/*
 * If status changed, log result in pd
 */
#ifdef OPTIMIZE_POWER
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
#endif

void pwrmgmt_connect(void)
{               
    pwrmgmt_glean_stats();
    pd.connect_cnt++;
#ifdef OPTIMIZE_POWER
    if (ptxmode == 1) {
        pd.connected_pkt_cnt += PWR_CONNECT_DUMMY_TX_H;
    } else {
        pd.connected_pkt_cnt += PWR_CONNECT_DUMMY_TX_N;
    }
#else
    pd.connected_pkt_cnt += PWR_CONNECT_DUMMY_TX_N;
#endif
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
    uint8 usage;
    uint16 voltage_lvl;

    if (pinit_done == 0) {
        return 0xFF;
    } else {
        base_capacity = battery_level_in_percent(pd.initial_v_adc);
        usage = pwrmgmt_battery_usage();

        if (base_capacity > usage) {
            calculated_capacity = base_capacity - usage;
        } else {
            calculated_capacity = 0;
        }
        
        // Double check battery voltage
        voltage_lvl = battery_get_last_measure();
        if (voltage_lvl > 0) {
            guess_capacity = battery_level_in_percent(voltage_lvl);
            if ((guess_capacity == 100) && (calculated_capacity < 40)) {
                memset((uint8 *)&pd, 0, sizeof(pd));
                pd.initial_v_adc = voltage_lvl;
                pwrmgmt_flash_dump();
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
    if (pinit_done == 1) {
        pwrmgmt_glean_stats();
        pd.battery_voltage = battery_get_last_measure();
        persistent_pwrmgmt_set_latest(&pd);
    }
}

void pwrmgmt_on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        conn_interval = p_evt->conn_interval;
        conn_latency = p_evt->slave_latency;
    }
}

#ifdef OPTIMIZE_POWER
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
    uint32 ts_threshold = date_time_get_relative() - PWR_TIMEOUT_CHECK_PERIOD;
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
    pwr_timeout_ts[min_idx] = date_time_get_relative();
}
#endif

