/*
 * ble_iqo.c
 */

#include <stdint.h>
#include <string.h>
#include "softdevice_handler.h"
#include "app_util_platform.h"
#include "ble_iqo.h"
#include "intermcu_spi.h"

extern bool is_central(uint16_t conn_handle);

ble_iqo_cmd_t iqo_tgt_cmd;
ble_iqo_id_t iqo_tgt_identify;
ble_iqo_t *iqo_ptr = NULL;

extern void scan_start(void);
extern void disconnect_peer(void);

static void on_connect(ble_iqo_t * p_iqo, ble_evt_t * p_ble_evt)
{
	if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH) {
		p_iqo->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
	}
}

static void on_disconnect(ble_iqo_t * p_iqo, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);

    if (is_central(p_ble_evt->evt.gap_evt.conn_handle)) {
    	p_iqo->conn_handle = BLE_CONN_HANDLE_INVALID;
    }
}

static void on_write(ble_iqo_t * p_iqo, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_iqo->iqo_identify_handle.value_handle) {
        memcpy(&(iqo_tgt_identify.id[0]), p_evt_write->data, p_evt_write->len);
        iqo_tgt_identify.id[p_evt_write->len] = 0;
    } else if (p_evt_write->handle == p_iqo->iqo_cmd_handle.value_handle) {
    	memcpy((uint8_t *)&iqo_tgt_cmd, p_evt_write->data, p_evt_write->len);
    	if (iqo_tgt_cmd.cmd == 0) {
    		// Connect to peer
    		scan_start();
    	} else if (iqo_tgt_cmd.cmd == 1) {
    		// Disconnect from peer
    		disconnect_peer();
    	} else if (iqo_tgt_cmd.cmd == 2) {
    		// Set WiFi SSID
    		intermcu_notify(NRF_WIFI_SSID_NOTIFY, (p_evt_write->data + 1), (p_evt_write->len - 1));
    	} else if (iqo_tgt_cmd.cmd == 3) {
    		// Set WiFi Password
    		intermcu_notify(NRF_WIFI_PSWD_NOTIFY, (p_evt_write->data + 1), (p_evt_write->len - 1));
    	} else if (iqo_tgt_cmd.cmd == 4) {
    		// Reset RT5350
    		trigger_rt5350_reset();
    	}
	}
}

void ble_iqo_on_ble_evt(ble_iqo_t * p_iqo, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_iqo, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_iqo, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_iqo, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t iqo_cmd_char_add(ble_iqo_t * p_iqo)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_IQO_CMD_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&iqo_tgt_cmd, 0, sizeof(ble_iqo_cmd_t));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_iqo_cmd_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(ble_iqo_cmd_t);
    attr_char_value.p_value   = (uint8_t *)&iqo_tgt_cmd;

    return sd_ble_gatts_characteristic_add(p_iqo->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_iqo->iqo_cmd_handle);
}

// Intermediate temperature
static uint32_t iqo_identify_char_add(ble_iqo_t * p_iqo)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_IQO_IDENTIFY_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&iqo_tgt_identify, 0, sizeof(ble_iqo_id_t));
    memcpy(&(iqo_tgt_identify.id[0]), "Ting's ACC1", 11);

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 20;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = (uint8_t *)&iqo_tgt_identify;

    return sd_ble_gatts_characteristic_add(p_iqo->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_iqo->iqo_identify_handle);
}

uint32_t ble_iqo_init(ble_iqo_t * p_iqo)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    iqo_ptr = p_iqo;

    // Initialize service structure
    p_iqo->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_IQO_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_iqo->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add memory dump characteristic
    err_code = iqo_cmd_char_add(p_iqo);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = iqo_identify_char_add(p_iqo);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

