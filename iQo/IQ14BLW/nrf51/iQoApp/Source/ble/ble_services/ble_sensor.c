/*
 * sensor.c
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#include "ble_sensor.h"
#include "ble_iqo.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "sensor_service.h"

extern bool is_central(uint16_t conn_handle);
bool m_is_notification_enabled = false;

static void on_connect(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
	if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH) {
		p_sensor->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
	}
}

static void on_disconnect(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);

    if (is_central(p_ble_evt->evt.gap_evt.conn_handle)) {
    	p_sensor->conn_handle = BLE_CONN_HANDLE_INVALID;
    }
}

static void on_write(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if ((p_evt_write->handle == p_sensor->sensor_handle.cccd_handle) && (p_evt_write->len == 2)) {
		if (p_sensor->evt_handler != NULL) {
			ble_sensor_evt_t  evt;
			m_is_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

			if (m_is_notification_enabled) {
				evt.evt_type = BLE_SENSOR_EVT_NOTIFICATION_ENABLED;
			} else {
				evt.evt_type = BLE_SENSOR_EVT_NOTIFICATION_DISABLED;
			}
			p_sensor->evt_handler(p_sensor, &evt);
		}
	}
}

void ble_sensor_on_ble_evt(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sensor, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sensor, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sensor, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t sensor_char_add(ble_sensor_t * p_sensor)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	uint8_t             buffer[4] = {0};

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.notify = 1;
	char_md.p_char_user_desc    = NULL;
	char_md.p_char_pf           = NULL;
	char_md.p_user_desc_md      = NULL;
	char_md.p_cccd_md           = &cccd_md;
	char_md.p_sccd_md           = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_IQO_SENSOR_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 4;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 4;
	attr_char_value.p_value   = buffer;

    return sd_ble_gatts_characteristic_add(p_sensor->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_sensor->sensor_handle);
}


uint32_t ble_sensor_init(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_sensor->evt_handler = p_sensor_init->evt_handler;
    p_sensor->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_IQO_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sensor->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add measurement characteristic
    err_code = sensor_char_add(p_sensor);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_sensor_send_isl(ble_sensor_t * p_sensor, uint8_t *pBuf)
{
    uint32_t err_code;
    uint16_t hvx_len;
    ble_gatts_hvx_params_t hvx_params;

    // Send value if connected and notifying
    if (p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) {
        hvx_len = 4;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_sensor->sensor_handle.value_handle;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len = &hvx_len;
        hvx_params.p_data = pBuf;

        err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != 4)) {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    } else {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}
