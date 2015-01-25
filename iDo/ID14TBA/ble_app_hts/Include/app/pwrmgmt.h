/*
 * Power management and battery capacity estimation
 */
#ifndef __PWR_MGMT__
#define __PWR_MGMT__

#include <stdint.h>
#include "nrf_types.h"
#include "ble_conn_params.h"

#ifdef OPTIMIZE_POWER
#define TX_HIGH         0
#define TX_LOW          1
#endif
#define CONNECT         0
#define DISCONNECT      1
#define MEASURE_TEMP    2
#define PWR_MAX         3


struct pwrmgmt_data {
    uint16  initial_v_adc;
    uint16  battery_voltage;
    uint32  adv_pkt_cnt; 
    uint32  connect_cnt;
    uint32  connected_pkt_cnt;
    uint32  temp_sample_cnt;
};

typedef void (*pwrmgmt_callback_t)(void);

void pwrmgmt_init(void);
void pwrmgmt_checkvdd_callback(void);
void pwrmgmt_event(uint8 evt);
void pwrmgmt_set_conn_param(uint16 interval, uint16 latency);
uint8 pwrmgmt_battery_percent(void);
void pwrmgmt_flash_dump(void);
uint8 pwrmgmt_hb(void);
void pwrmgmt_timeout(void);
void pwrmgmt_on_conn_params_evt(ble_conn_params_evt_t * p_evt);

#endif
