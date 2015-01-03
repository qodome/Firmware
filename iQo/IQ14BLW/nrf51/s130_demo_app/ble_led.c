/*
 * led.c
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#include "ble_led.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf_pwm.h"

uint32_t led_addr = 0;

static void on_connect(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
	p_led->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

static void on_disconnect(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_led->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_led->led_handles.value_handle) {
        if (p_evt_write->len == sizeof(uint32_t)) {
        	led_addr = (((uint32_t)p_evt_write->data[0]) << 24 |
        					((uint32_t)p_evt_write->data[1]) << 16 |
        					((uint32_t)p_evt_write->data[2]) << 8 |
        					((uint32_t)p_evt_write->data[3]));
        	// FIXME
        }
    }
}

static void on_rw_authorize_request(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
	ble_led_v_t mem_v;
    ble_gatts_evt_rw_authorize_request_t * evt_rw_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (evt_rw_auth->type != BLE_GATTS_AUTHORIZE_TYPE_READ)
    {
        // Unexpected operation
        return;
    }

    if (evt_rw_auth->request.read.handle == p_led->led_handles.value_handle) {
        ble_gatts_rw_authorize_reply_params_t   auth_params;
        ble_gatts_rw_authorize_reply_params_t * p_auth_params = &auth_params;
        uint8_t idx = 0;

        ((uint8_t *)&(mem_v.addr))[0] = (uint8_t)(((led_addr & 0xFF000000) >> 24) & 0xFF);
        ((uint8_t *)&(mem_v.addr))[1] = (uint8_t)(((led_addr & 0x00FF0000) >> 16) & 0xFF);
        ((uint8_t *)&(mem_v.addr))[2] = (uint8_t)(((led_addr & 0x0000FF00) >> 8) & 0xFF);
        ((uint8_t *)&(mem_v.addr))[3] = (uint8_t)(led_addr & 0xFF);
        for (idx = 0; idx < 16; idx++) {
        	mem_v.v[idx] = ((const uint8_t *)led_addr)[idx];
        }
        memset((void *)p_auth_params, 0, sizeof(auth_params));
        auth_params.type =  BLE_GATTS_AUTHORIZE_TYPE_READ;
        auth_params.params.read.p_data = (uint8_t *)&mem_v;
        auth_params.params.read.len    = sizeof(mem_v);
        auth_params.params.read.update = 1;

        APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_led->conn_handle, p_auth_params));
    }
}

void ble_led_on_ble_evt(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_led, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_led, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_led, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	on_rw_authorize_request(p_led, p_ble_evt);
        	break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t hts_led_char_add(ble_led_t * p_led)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_led_v_t		mem_v;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_LED_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    memset((void *)&mem_v, 0, sizeof(ble_led_v_t));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_led_v_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(ble_led_v_t);
    attr_char_value.p_value   = (uint8_t *)&mem_v;

    return sd_ble_gatts_characteristic_add(p_led->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_led->led_handles);
}


uint32_t ble_led_init(ble_led_t * p_led)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_led->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_LED_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_led->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add measurement characteristic
    err_code = hts_led_char_add(p_led);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}
