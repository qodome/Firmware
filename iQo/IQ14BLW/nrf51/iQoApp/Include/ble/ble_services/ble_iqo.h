/*
 * ble_iqo.h
 */

#ifndef __BLE_IQO__
#define __BLE_IQO__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_IQO_SERVICE    	0x90D0
#define BLE_UUID_IQO_CMD_CHAR   	0x9002
#define BLE_UUID_IQO_IDENTIFY_CHAR  0x9003
#define BLE_UUID_IQO_LED_CHAR  		0x9004

typedef struct ble_iqo_s ble_iqo_t;
typedef void (*ble_iqo_evt_handler_t) (ble_iqo_t * p_iqo, ble_evt_t * p_ble_evt);

typedef struct
{
	ble_iqo_evt_handler_t       evt_handler;
} ble_iqo_init_t;

typedef struct ble_iqo_s {
    ble_iqo_evt_handler_t       evt_handler;
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    iqo_cmd_handle;
    ble_gatts_char_handles_t    iqo_identify_handle;
    uint16_t                    conn_handle;
} ble_iqo_t;

typedef struct ble_iqo_cmd_s
{
	uint32_t cmd;
} __attribute__((packed)) ble_iqo_cmd_t;

typedef struct ble_iqo_id_s
{
	uint8_t id[20];
} __attribute__((packed)) ble_iqo_id_t;

// Initialize iQo BLE service
uint32_t ble_iqo_init(ble_iqo_t * p_iqo);

// Event handler
void ble_iqo_on_ble_evt(ble_iqo_t * p_iqo, ble_evt_t * p_ble_evt);

#endif
