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

#define PWR_VOLTAGE_DELAY       2000
#define PWR_VOLTAGE_CNT         10
#define PWR_DUMP_PERIOD         3600    // seconds
#define PWR_TX_HIGH_FACTOR      1.1     // FIXME: this factor should be measured

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
static uint8 ptarget = 0;       // Connected with Android; Connected with iOS
static uint8 pspeed = 0;        // Speed slow: 0; Speed burst: 1
static uint16 conn_interval = 0;
static uint16 conn_latency = 0;

extern void iDo_ScheduleCheckVDD(uint16 delay);
extern void iDo_StopVDDCheck(void);

void pwrmgmt_tx_high(void);
void pwrmgmt_tx_low(void);
void pwrmgmt_rx_high(void);
void pwrmgmt_rx_low(void);
void pwrmgmt_connect(void);
void pwrmgmt_disconnect(void);
void pwrmgmt_connect_android(void);
void pwrmgmt_connect_ios(void);
void pwrmgmt_speed_high(void);
void pwrmgmt_temp_measure(void);

static pwrmgmt_callback_t pwr_call[PWR_MAX] = {
    pwrmgmt_tx_high,
    pwrmgmt_tx_low,
    pwrmgmt_rx_high,
    pwrmgmt_rx_low,
    pwrmgmt_connect,
    pwrmgmt_disconnect,
    pwrmgmt_connect_android,
    pwrmgmt_connect_ios,
    pwrmgmt_speed_high,
    pwrmgmt_temp_measure,
};

void pwrmgmt_init(void)
{
    if (custom_mgmt_initialized() == 1) {
        custom_mgmt_get(&pd);
        pinit = 1;
    } else {
        osal_memset((uint8 *)&pd, 0, sizeof(pd));
        pinit = 0;
        pcnt = PWR_VOLTAGE_CNT;
        pvsum = 0;
        // Initial delay: 5s
        iDo_ScheduleCheckVDD(5000);
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
    }
}

static void pwrmgmt_glean_stats(void)
{
    uint32 now = osal_getRelativeClock();
    uint32 delta = now - ptlast;
    uint32 adv_cnt = 0;
    uint32 tmp = 0;

    if (pconn_status == 0) {
        tmp = (DEFAULT_ADVERTISING_INTERVAL * 625) / 1000;
        adv_cnt = (delta * 1000) / tmp;
        if (ptxmode == 1) {
            adv_cnt = (uint32)((float)adv_cnt * PWR_TX_HIGH_FACTOR);
        }
        pd.adv_pkt_cnt += adv_cnt;
    } else {
        //ptarget
        //pspeed
    }
}

/*
 * If status changed, log result in pd
 */

void pwrmgmt_tx_high(void)
{
    if (ptxmode == 0) {

    } 
    ptxmode = 1;
}

void pwrmgmt_tx_low(void)
{
    
}

void pwrmgmt_rx_high(void)
{
    
}

void pwrmgmt_rx_low(void)
{
    
}

void pwrmgmt_connect(void)
{
    
}

void pwrmgmt_disconnect(void)
{
    
}

void pwrmgmt_connect_android(void)
{
    
}

void pwrmgmt_connect_ios(void)
{
    
}

void pwrmgmt_speed_high(void)
{
    
}

void pwrmgmt_temp_measure(void)
{
    
}

void pwrmgmt_event(uint8 evt)
{
    if (evt <= PWR_MAX) {
        (*pwr_call[evt])();
    }
}

void pwrmgmt_set_conn_param(uint16 interval, uint16 latency)
{
    conn_interval = interval;
    conn_latency = latency;
}

uint8 pwrmgmt_battery_percent(void)
{
    return 0xFF;
}
