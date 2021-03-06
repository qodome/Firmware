/*
 * acc.c
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#include <stdint.h>
#include <string.h>
#include "softdevice_handler.h"
#include "app_util_platform.h"
#include "ble_acc.h"
#include "ble_ido.h"
#include "acc_service.h"

extern uint8_t acc_data_enabled;

uint8_t acc_addr = 0;
uint8_t acc_value = 0;
ble_acc_t *acc_ptr = NULL;

static void on_connect(ble_acc_t * p_acc, ble_evt_t * p_ble_evt)
{
	p_acc->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

static void on_disconnect(ble_acc_t * p_acc, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_acc->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_acc_t * p_acc, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_acc->acc_handles.value_handle) {
        if (p_evt_write->len == 2 * sizeof(uint8_t)) {
        	acc_addr = (p_evt_write->data[0]) & 0x7F;
        	if ((p_evt_write->data[0]) & 0x80) {
        		acc_value = p_evt_write->data[1];
        		acc_service_spi_write(acc_addr, acc_value);
        	}
        }
    } else if ((p_evt_write->handle == p_acc->acc_data_handle.cccd_handle) && (p_evt_write->len == 2)) {
    	if (ble_srv_is_notification_enabled(p_evt_write->data)) {
    		acc_data_enabled = 1;
    	} else {
    		//APP_ERROR_CHECK(app_timer_stop(m_acc_timer_id));
    		acc_data_enabled = 0;
    	}
	}
}

static void on_rw_authorize_request(ble_acc_t * p_acc, ble_evt_t * p_ble_evt)
{
	ble_acc_v_t v;
    ble_gatts_evt_rw_authorize_request_t * evt_rw_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (evt_rw_auth->type != BLE_GATTS_AUTHORIZE_TYPE_READ)
    {
        // Unexpected operation
        return;
    }

    if (evt_rw_auth->request.read.handle == p_acc->acc_handles.value_handle) {
        ble_gatts_rw_authorize_reply_params_t   auth_params;
        ble_gatts_rw_authorize_reply_params_t * p_auth_params = &auth_params;

        v.addr = acc_addr;
        acc_value = acc_service_spi_read(acc_addr);
        v.value = acc_value;
        memset((void *)p_auth_params, 0, sizeof(auth_params));
        auth_params.type =  BLE_GATTS_AUTHORIZE_TYPE_READ;
        auth_params.params.read.p_data = (uint8_t *)&v;
        auth_params.params.read.len    = sizeof(v);
        auth_params.params.read.update = 1;

        APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_acc->conn_handle, p_auth_params));
    }
}

void ble_acc_on_ble_evt(ble_acc_t * p_acc, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_acc, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_acc, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_acc, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	on_rw_authorize_request(p_acc, p_ble_evt);
        	break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t acc_register_char_add(ble_acc_t * p_acc)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_acc_v_t		mem_v;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ACC_REG_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    memset((void *)&mem_v, 0, sizeof(ble_acc_v_t));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_acc_v_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(ble_acc_v_t);
    attr_char_value.p_value   = (uint8_t *)&mem_v;

    return sd_ble_gatts_characteristic_add(p_acc->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_acc->acc_handles);
}

// Intermediate temperature
static uint32_t acc_data_char_add(ble_acc_t * p_acc)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_hts_meas_t      initial_htm;
    uint8_t             data[20] = {0};

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

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ACC_DATA_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    memset(&initial_htm, 0, sizeof(initial_htm));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 20;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = data;

    return sd_ble_gatts_characteristic_add(p_acc->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_acc->acc_data_handle);
}

uint32_t ble_acc_init(ble_acc_t * p_acc)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    acc_ptr = p_acc;

    // Initialize service structure
    p_acc->conn_handle = BLE_CONN_HANDLE_INVALID;
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ACC_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_acc->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add memory dump characteristic
    err_code = acc_register_char_add(p_acc);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = acc_data_char_add(p_acc);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

