/*
 * Qodome Public BLE Service
 */
#include "ble_qodome_public.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"

ble_qodome_t  m_qodome;

static void on_connect(ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt)
{
	p_qodome->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

static void on_disconnect(ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_qodome->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_qodome->qodome_handles.value_handle) {
        if (p_evt_write->len == sizeof(uint32_t)) {
            APP_ERROR_CHECK(NRF_ERROR_SVC_HANDLER_MISSING);
        }
    }
}

static void on_rw_authorize_request(ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_rw_authorize_request_t * evt_rw_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (evt_rw_auth->type != BLE_GATTS_AUTHORIZE_TYPE_READ)
    {
        // Unexpected operation
        return;
    }
}

void ble_qodome_on_ble_evt(ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_qodome, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_qodome, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_qodome, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	on_rw_authorize_request(p_qodome, p_ble_evt);
        	break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t hts_qodome_char_add(ble_qodome_t * p_qodome)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, QODOME_PUBLIC_SET_NAME);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 4;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_qodome->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_qodome->qodome_handles);
}

uint32_t ble_qodome_init(void)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    // Initialize service structure
    ble_qodome_t  * p_qodome = &m_qodome;

    p_qodome->conn_handle = BLE_CONN_HANDLE_INVALID;
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, QODOME_PUBLIC_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_qodome->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add memory dump characteristic
    err_code = hts_qodome_char_add(p_qodome);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}
