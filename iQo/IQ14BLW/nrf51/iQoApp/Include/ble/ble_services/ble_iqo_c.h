/*
 * Connects to iQo peer
 */
#ifndef __BLE_IQO_C__
#define __BLE_IQO_C__

#include <stdint.h>
#include "ble.h"

typedef enum {
    BLE_IQO_C_EVT_DISCOVERY_TEMP_COMPLETE = 1,
    BLE_IQO_C_EVT_DISCOVERY_ACC_COMPLETE,
    BLE_IQO_C_EVT_IT_NOTIFY,
    BLE_IQO_C_EVT_ACC_NOTIFY,
} ble_iqo_c_evt_type_t;

typedef struct {
    uint32_t temp_data;
} ble_iqo_c_temp_t;

typedef struct
{
    ble_iqo_c_evt_type_t evt_type;
    union {
        ble_iqo_c_temp_t temp;
    } params;
} ble_iqo_c_evt_t;

typedef struct ble_iqo_c_s ble_iqo_c_t;

typedef void (* ble_iqo_c_evt_handler_t) (ble_iqo_c_t * p_ble_iqo_c, ble_iqo_c_evt_t * p_evt);

/* 
 * iQo peer structure 
 */
typedef struct ble_iqo_c_s
{
    uint16_t                conn_handle;
    uint16_t                iqo_c_temp_cccd_handle;
    uint16_t                iqo_c_temp_handle;
    uint16_t                iqo_c_acc_cccd_handle;
    uint16_t                iqo_c_acc_handle;
    ble_iqo_c_evt_handler_t evt_handler;
} ble_iqo_c_t;

typedef struct
{
    ble_iqo_c_evt_handler_t evt_handler;
} ble_iqo_c_init_t;

uint32_t ble_iqo_c_init(uint16_t conn_handle, ble_iqo_c_t * p_ble_iqo_c, ble_iqo_c_init_t * p_ble_iqo_c_init);

void ble_iqo_c_on_ble_evt(ble_iqo_c_t * p_ble_iqo_c, const ble_evt_t * p_ble_evt);

uint32_t ble_iqo_c_temp_enable(ble_iqo_c_t * p_ble_iqo_c);
uint32_t ble_iqo_c_acc_enable(ble_iqo_c_t * p_ble_iqo_c);

// Setup DB discovery
uint32_t ble_iqo_c_setup(void);

#endif
